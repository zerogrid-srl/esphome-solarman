#include "solarman_minimal.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"
#include "esphome/components/wifi/wifi_component.h"
#ifdef USE_ESP32
#include <esp_netif.h>
#endif

#include <sys/socket.h>
#include <sys/select.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cinttypes>
#include <cerrno>
#include <cstring>
#include <cstdlib>

namespace esphome {
namespace solarman_minimal {

static const char *const TAG = "solarman";

// ── Register map – Deye hybrid (SUN-xK-SG0xLPx) ───────────────────────────
//
// Taken from the `deye_hybrid` profile of the ha-solarman integration, which
// is in production use against a Deye 8K SG05LP1. Verified against that
// project's own parser for the multi-register (32-bit) decoding order.
//
// Read in TWO batches rather than one connection per quantity. update() runs on
// the main loop, and four sequential TCP round-trips measured 1427 ms on real
// hardware — long enough for ESPHome to warn, and long enough to disturb a BLE
// stack sharing the same device.
//
// Batch 1  0x003B..0x006F  (53 regs) - state, daily counters, energy totals
//   [ 0]      0x003B = Device State, enum: 0 standby, 1 self-test, 2 normal,
//                      3 alarm, 4 fault
//   [11]      0x0046 = Daily Battery Charge    x0.1 kWh
//   [12]      0x0047 = Daily Battery Discharge x0.1 kWh
//   [17]      0x004C = Daily Energy Bought     x0.1 kWh
//   [18]      0x004D = Daily Energy Sold       x0.1 kWh
//   [25]      0x0054 = Daily Load Consumption  x0.1 kWh
//   [26]+[27] 0x0055/0x0056 = Total Load Consumption x0.1 kWh, 32-bit UNSIGNED,
//                             first register is the LOW word.
//   [37]+[38] 0x0060/0x0061 = Total Production      x0.1 kWh, 32-bit UNSIGNED,
//                             first register is the LOW word.
//   [49]      0x006C = Daily Production x0.1 kWh
//   [50]      0x006D = PV1 Voltage      x0.1 V
//   [52]      0x006F = PV2 Voltage      x0.1 V
//
// Batch 2  0x00A9..0x00C2  (26 regs)
//   [0]  0x00A9 = Grid Power      W  signed
//   [9]  0x00B2 = Load Power      W  signed
//   [14] 0x00B7 = Battery Voltage ×0.01 V
//   [15] 0x00B8 = Battery SOC     %
//   [17] 0x00BA = PV1 Power       W
//   [18] 0x00BB = PV2 Power       W
//   [21] 0x00BE = Battery Power   W  signed, POSITIVE = DISCHARGING
//   [25] 0x00C2 = Grid connected, non-zero = tied to the utility
//
// Both ranges were WIDENED rather than adding a third batch. A Modbus read
// costs one TCP round trip whatever its length, so 53 registers cost what 16
// cost, while a third connection would add roughly 350 ms to the main loop -
// the very budget the two-batch split was made to protect.
//
// Confirmed against a Deye 8K SG05LP1 on 2026-08-22:
//   * Battery voltage scale 0.01 (read 52.3 V on a 48 V bank).
//   * Grid/Load power scale is 1, not the 10 the upstream profile's
//     `scale: [1, 10, 10]` hints at — 2705 W of load on an 8 kW inverter.
//   * 32-bit word order (760.5 kWh total; reversed would read in the millions).
//   * Battery power sign: at 19:39 with PV at 2 W, grid at 0 W and load at
//     2705 W, the battery must have been discharging, and the register read
//     +2813 W. The 108 W difference is inverter self-consumption. So a
//     positive value means DISCHARGE — the opposite of what was documented
//     here before.
//
// Still open: the Grid Power sign convention. It read 0 W during the test, so
// nothing could be inferred. Reading the upstream source does not settle it
// either — the profile marks the sensor `attributes: [inverse]` while the
// parser tests a differently-named key, `inverted`. To be settled by comparing
// against a working integration during grid import or export.
static const uint16_t REG_BATCH1_START = 0x003B;
static const uint8_t  REG_BATCH1_COUNT = 53;  // through 0x006F
static const uint16_t REG_BATCH2_START = 0x00A9;
static const uint8_t  REG_BATCH2_COUNT = 26;  // through 0x00C2

// Fixed NVS key for the serial preference (independent of the serial value)
static const uint32_t SERIAL_PREFS_KEY = 0x534C4D41;  // "SLMA"

// READ-ONLY BY DESIGN. This is the only Modbus function code this component
// ever emits, and it is a read. Nothing here can alter inverter settings,
// which matters because it runs against live customer installations.
// Introducing a write function code (0x05/0x06/0x0F/0x10) would break that
// guarantee and must be a deliberate, reviewed decision — not a side effect.
static const uint8_t MODBUS_FC_READ_HOLDING_REGISTERS = 0x03;

// ── setup ─────────────────────────────────────────────────────────────────

void SolarmanMinimal::setup() {
  serial_prefs_ = global_preferences->make_preference<SolarmanSerialPrefs>(SERIAL_PREFS_KEY);

  // Load serial from flash if not provided in YAML (serial_ == 0)
  if (serial_ == 0) {
    SolarmanSerialPrefs sp{};
    if (serial_prefs_.load(&sp) && sp.serial != 0) {
      serial_ = sp.serial;
      ESP_LOGI(TAG, "Loaded serial %" PRIu32 " from flash", serial_);
    } else {
      // Not an error: discovery will adopt a logger and its serial on the
      // first update. Typing one in only matters when several sticks share
      // the LAN and a specific one is wanted.
      ESP_LOGI(TAG, "No serial stored - will adopt the first logger discovered");
      return;
    }
  }

  prefs_ = global_preferences->make_preference<SolarmanPrefs>(serial_);
  if (!host_override_) {
    SolarmanPrefs saved{};
    if (prefs_.load(&saved) && (saved.ip[0] | saved.ip[1] | saved.ip[2] | saved.ip[3])) {
      memcpy(&host_addr_, saved.ip, 4);
      ESP_LOGI(TAG, "Restored cached IP %s for serial %" PRIu32, host_str().c_str(), serial_);
    }
  }
}

void SolarmanMinimal::dump_config() {
  ESP_LOGCONFIG(TAG, "Solarman:");
  ESP_LOGCONFIG(TAG, "  Serial: %" PRIu32, serial_);
  if (host_override_) {
    ESP_LOGCONFIG(TAG, "  Host: %s (manual)", host_str().c_str());
  } else if (has_host()) {
    ESP_LOGCONFIG(TAG, "  Host: %s (discovered)", host_str().c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  Host: pending UDP discovery");
  }
}

std::string SolarmanMinimal::host_str() const {
  char buf[INET_ADDRSTRLEN] = {0};
  struct in_addr a;
  a.s_addr = host_addr_;
  inet_ntop(AF_INET, &a, buf, sizeof(buf));
  return std::string(buf);
}

void SolarmanMinimal::save_host() {
  SolarmanPrefs p{};
  memcpy(p.ip, &host_addr_, 4);
  prefs_.save(&p);
}

void SolarmanMinimal::set_host(const std::string &host) {
  struct in_addr a;
  if (inet_pton(AF_INET, host.c_str(), &a) == 1) {
    host_addr_ = a.s_addr;
    host_override_ = true;
    // Same reason as the serial: make it survive an unclean reset.
    global_preferences->sync();
    ESP_LOGI(TAG, "Host set to %s (manual)", host_str().c_str());
  } else {
    ESP_LOGE(TAG, "Invalid host IP: '%s'", host.c_str());
  }
}

void SolarmanMinimal::set_serial_from_text(const std::string &s) {
  uint32_t val = strtoul(s.c_str(), nullptr, 10);
  if (val == 0) {
    ESP_LOGW(TAG, "Invalid serial number: '%s'", s.c_str());
    return;
  }
  serial_ = val;
  SolarmanSerialPrefs sp{val};
  serial_prefs_.save(&sp);
  // save() only queues; ESPHome flushes preferences on a timer and on a clean
  // shutdown. A watchdog reset is neither, so a serial typed seconds before a
  // crash was silently lost - which is exactly the situation someone is in
  // while trying to configure a device that keeps rebooting. Flush now.
  global_preferences->sync();
  // Re-bind the IP prefs slot to the new serial key
  prefs_ = global_preferences->make_preference<SolarmanPrefs>(serial_);
  // Clear any cached IP so discovery runs fresh for the new serial
  if (!host_override_)
    host_addr_ = 0;
  ESP_LOGI(TAG, "Serial set to %" PRIu32 " - saved to flash", serial_);
}

// ── update ────────────────────────────────────────────────────────────────

// The manual scan runs here, not in update(): one batch per loop pass. Each
// batch blocks for SCAN_WAIT, so everything else on the device runs between
// batches rather than being shut out for the whole sweep. A sweep that finds
// nothing is left failed - the person who pressed the button decides whether
// to try again, rather than the device retrying on its own.
void SolarmanMinimal::loop() {
  if (!scan_fast_)
    return;
  if (!wifi::global_wifi_component->is_connected())
    return;

  if (tcp_scan_step()) {
    scan_fast_ = false;
    scan_state_ = SCAN_FOUND;
    ESP_LOGI(TAG, "Search finished: %s", status_line().c_str());
    return;
  }
  if (scan_wrapped_) {
    scan_fast_ = false;
    scan_state_ = SCAN_FAILED;
    ESP_LOGW(TAG, "Search finished: nothing answering on port %u", SOLARMAN_PORT);
  }
}

void SolarmanMinimal::update() {
  if (!wifi::global_wifi_component->is_connected()) {
    ESP_LOGD(TAG, "WiFi down - skipping poll");
    return;
  }

  // Find the logger if we do not have it. Two mechanisms, tried in order and
  // both optional: the UDP probe, which no dongle has ever answered, and the
  // subnet scan, which has.
  //
  // Rewritten because the first version ran the UDP probe even after the scan
  // had just succeeded - a wasted second and a "not found on LAN" warning
  // logged immediately below "Logger confirmed at", which reads like a
  // contradiction.
  if (serial_ == 0 || !has_host()) {
    uint32_t now = millis();
    // A failed probe is not retried on the next poll: it blocks the main loop
    // while it waits, and at a 10 s interval that spent a second in ten on a
    // device that also runs a BLE stack.
    bool backing_off = last_discovery_ms_ != 0 &&
                       (now - last_discovery_ms_) < DISCOVERY_RETRY;
    if (backing_off) {
      ESP_LOGD(TAG, "Discovery backing off - %u s to the next attempt",
               (unsigned) ((DISCOVERY_RETRY - (now - last_discovery_ms_)) / 1000));
    } else {
      last_discovery_ms_ = now;
      if (discover_host())
        last_discovery_ms_ = 0;
      else
        ESP_LOGW(TAG, "Discovery failed - next attempt in %u s",
                 (unsigned) (DISCOVERY_RETRY / 1000));
    }

    // No automatic subnet scan here: it used to run a few addresses every
    // poll, forever, while unconfigured - real TCP connections to every host
    // on someone else's LAN, indefinitely. The scan now runs only from
    // start_scan() (see loop()), a single sweep per button press.

    // Still nothing to talk to, or nothing to identify ourselves with.
    if (!has_host() || serial_ == 0)
      return;
  }

  // ── Batch 2 first: grid, load, battery, PV power ────────────────────────
  // Read first because it drives the failure counter and carries the values
  // that matter most; if the inverter is unreachable, skip the rest.
  std::vector<uint16_t> b2;
  if (!read_registers(REG_BATCH2_START, REG_BATCH2_COUNT, b2)) {
    consecutive_failures_++;
    ESP_LOGW(TAG, "Read failed (%u/%u)", consecutive_failures_, MAX_FAILURES);
    if (consecutive_failures_ >= MAX_FAILURES && !host_override_) {
      ESP_LOGW(TAG, "Dropping cached IP - re-discovering next cycle");
      host_addr_ = 0;
      consecutive_failures_ = 0;
    }
    return;
  }
  consecutive_failures_ = 0;
  clear_error();
  if (grid_power_)      publish_signed(grid_power_, b2[0]);
  if (load_power_)      publish_signed(load_power_, b2[9]);
  if (battery_voltage_) battery_voltage_->publish_state(b2[14] * 0.01f);
  if (battery_soc_)     battery_soc_->publish_state(b2[15]);
  if (pv1_power_)       pv1_power_->publish_state(b2[17]);
  if (pv2_power_)       pv2_power_->publish_state(b2[18]);
  if (battery_power_)   publish_signed(battery_power_, b2[21]);
  // Published as 0/1 rather than a binary_sensor: this component builds its own
  // entities, and a numeric sensor keeps every consumer on one code path.
  if (grid_connected_)  grid_connected_->publish_state(b2[25] != 0 ? 1.0f : 0.0f);

  // ── Batch 1: state, daily counters, energy totals, PV voltages ──────────
  std::vector<uint16_t> b1;
  if (read_registers(REG_BATCH1_START, REG_BATCH1_COUNT, b1)) {
    if (device_state_) device_state_->publish_state(b1[0]);
    if (daily_battery_charge_)    daily_battery_charge_->publish_state(b1[11] * 0.1f);
    if (daily_battery_discharge_) daily_battery_discharge_->publish_state(b1[12] * 0.1f);
    if (daily_energy_bought_)     daily_energy_bought_->publish_state(b1[17] * 0.1f);
    if (daily_energy_sold_)       daily_energy_sold_->publish_state(b1[18] * 0.1f);
    if (daily_consumption_)       daily_consumption_->publish_state(b1[25] * 0.1f);
    if (total_consumption_) {
      uint32_t raw = (uint32_t) b1[26] | ((uint32_t) b1[27] << 16);
      total_consumption_->publish_state(raw * 0.1f);
    }
    if (total_production_) {
      uint32_t raw = (uint32_t) b1[37] | ((uint32_t) b1[38] << 16);
      total_production_->publish_state(raw * 0.1f);
    }
    if (daily_production_) daily_production_->publish_state(b1[49] * 0.1f);
    if (pv1_voltage_)      pv1_voltage_->publish_state(b1[50] * 0.1f);
    if (pv2_voltage_)      pv2_voltage_->publish_state(b1[52] * 0.1f);
  }
}

// ── Register scanner ──────────────────────────────────────────────────────
//
// Deye firmware variants shift the register map by a few offsets. This dumps
// the interesting ranges so the real addresses can be identified from the log.
// Values are printed unsigned, signed and ×0.1 to make the unit obvious.

void SolarmanMinimal::scan_registers() {
  if (serial_ == 0) {
    ESP_LOGW(TAG, "Cannot scan: serial not configured");
    return;
  }
  if (!has_host() && !discover_host()) {
    ESP_LOGW(TAG, "Cannot scan: inverter not found");
    return;
  }

  struct Range {
    uint16_t start;
    uint8_t count;
    const char *label;
  };
  // The first four ranges cover the whole verified deye_hybrid map, so a scan
  // confirms every sensor this component publishes. The rest are a wider sweep
  // for other Deye variants whose layout differs.
  static const Range ranges[] = {
      {0x003B, 10, "Time / status"},
      {0x005C, 12, "Energy totals (incl. 0x0060 total production)"},
      {0x0068, 12, "Daily yield + PV voltage / current"},
      {0x00A5, 28, "Grid power, load power, battery, PV power"},
      {0x00C0, 16, "Battery / temperature extras"},
      {0x00E0, 16, "Grid meter"},
      {0x0202, 24, "Alternative layout: energy"},
      {0x0244, 20, "Alternative layout: battery / PV"},
      {0x0258, 24, "Alternative layout: grid / load"},
  };

  ESP_LOGI(TAG, "===== REGISTER SCAN START =====");
  for (const auto &r : ranges) {
    std::vector<uint16_t> regs;
    if (!read_registers(r.start, r.count, regs)) {
      ESP_LOGW(TAG, "[%s] 0x%04X x%u -> READ FAILED", r.label, r.start, r.count);
      delay(200);
      continue;
    }
    ESP_LOGI(TAG, "--- %s (0x%04X, %u regs) ---", r.label, r.start, r.count);
    for (size_t i = 0; i < regs.size(); i++) {
      if (regs[i] == 0)
        continue;  // skip empty registers to keep the log readable
      ESP_LOGI(TAG, "  0x%04X = %5u | signed %6d | x0.1 %8.1f", (unsigned) (r.start + i), regs[i],
               (int) (int16_t) regs[i], regs[i] * 0.1f);
    }
    delay(200);  // let the logger stick breathe between connections
  }
  ESP_LOGI(TAG, "===== REGISTER SCAN END =====");
}

// ── UDP Discovery ─────────────────────────────────────────────────────────
//
// Logger sticks answer a broadcast probe on UDP port 48899 with a three-field
// reply: "<ip>,<mac>,<serial>".
//
// TWO probes are sent, because they are not interchangeable. Many sticks are
// built on Hi-Flying WiFi modules that answer HF-A11ASSISTHREAD and ignore
// WIFIKIT-214028-READ entirely — a Deye 8K SG05LP1 in the field stayed silent
// on the latter alone, including when probed unicast, while its TCP port 8899
// was open and serving. Sending only one probe looks exactly like "no logger
// on this network", which is a misleading failure. Upstream ha-solarman sends
// both for the same reason.
//
// When no serial is configured, the first well-formed reply is adopted along
// with the serial it carries. That makes a zero-configuration deploy possible:
// no IP, no serial, nothing to type in.

// Our own address, then the /24 broadcast derived from it. connect() on a UDP
// socket sends no packet: it only asks the stack which source address it would
// use, which is the portable way to learn our IP without an ESPHome API that
// differs between frameworks.
// The broadcast address of the network we are on.
//
// Two earlier attempts got this wrong in instructive ways. connect() plus
// getsockname() is the usual portable trick and returns 0.0.0.0 on LWIP for a
// socket that was never bound. ESPHome's network::IPAddress does expose str(),
// but only in the variant compiled with IPv6 enabled - so the code built here
// and failed at a customer whose build has it off.
//
// esp_netif is neither: it is stable across ESPHome versions, and it hands
// back the netmask as well, so the broadcast is the real one instead of a /24
// assumed on the network's behalf.
uint32_t SolarmanMinimal::subnet_broadcast() {
#ifdef USE_ESP32
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_netif_ip_info_t info{};
  if (netif != nullptr && esp_netif_get_ip_info(netif, &info) == ESP_OK && info.ip.addr != 0) {
    uint32_t bcast = (info.ip.addr & info.netmask.addr) | ~info.netmask.addr;
    char a[INET_ADDRSTRLEN] = {0}, m[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &info.ip.addr, a, sizeof(a));
    inet_ntop(AF_INET, &info.netmask.addr, m, sizeof(m));
    ESP_LOGD(TAG, "Own address %s netmask %s", a, m);
    return bcast;
  }
#endif
  ESP_LOGW(TAG, "No IPv4 address yet - cannot compute the subnet broadcast");
  return 0;
}


// ── TCP subnet scan ───────────────────────────────────────────────────────
//
// The last resort, and the only discovery that does not require the dongle to
// implement anything: find who has port 8899 open. Five dongles across four
// sites never answered a UDP probe, while every one of them served 8899.
//
// Incremental by design. One batch per loop pass, never blocking the main
// loop for more than SCAN_WAIT, watchdog fed throughout - the lesson of a
// device that rebooted every 30 seconds because a single connect() was left
// blocking.

bool SolarmanMinimal::verify_candidate(uint32_t addr) {
  uint32_t saved = host_addr_;
  host_addr_ = addr;
  std::vector<uint16_t> probe;
  // One register is enough to tell a Solarman logger from anything else that
  // happens to listen on 8899.
  if (read_registers(REG_BATCH2_START, 1, probe)) {
    ESP_LOGI(TAG, "Logger confirmed at %s", host_str().c_str());
    clear_error();
    save_host();
    return true;
  }

  // No Modbus payload came back. If we do not know the serial then the request
  // went out carrying zero, which these loggers refuse - but they refuse it
  // with a V5 frame of their own, and that frame names them. Take the serial
  // from it and ask again properly. This is what makes the scan genuinely
  // zero-config: address and serial both come from the dongle.
  if (serial_ == 0) {
    uint32_t announced = frame_serial(last_reply_);
    if (announced != 0) {
      serial_ = announced;
      SolarmanSerialPrefs sp{serial_};
      serial_prefs_.save(&sp);
      prefs_ = global_preferences->make_preference<SolarmanPrefs>(serial_);
      global_preferences->sync();
      ESP_LOGI(TAG, "Logger announced serial %" PRIu32 " - retrying the read",
               serial_);
      // A genuine V5 frame is a much stronger signal than a merely-open
      // port - nothing else on this LAN has ever produced one - so a
      // candidate that gets this far is worth a few tries, not one. The
      // dongle has been seen to take several seconds to answer while busy
      // polling the inverter over RS485; each attempt already carries its
      // own multi-second connect/read budget, so three attempts is the
      // several-seconds-per-try, up-to-half-a-minute-total investment this
      // address has earned by proving it is the real logger, before the
      // sweep moves on and that chance is gone for this pass.
      for (uint8_t attempt = 1; attempt <= 3; attempt++) {
        if (read_registers(REG_BATCH2_START, 1, probe)) {
          ESP_LOGI(TAG, "Logger confirmed at %s", host_str().c_str());
          clear_error();
          save_host();
          return true;
        }
        ESP_LOGW(TAG, "Still no answer with the announced serial (try %u/3)",
                 (unsigned) attempt);
      }
    } else {
      ESP_LOGW(TAG, "Reply carried no usable serial (%u bytes)",
               (unsigned) last_reply_.size());
    }
  }

  host_addr_ = saved;
  return false;
}

bool SolarmanMinimal::tcp_scan_step() {
  uint32_t bcast = subnet_broadcast();
  if (bcast == 0)
    return false;
  uint32_t network = bcast & htonl(0xFFFFFF00);

  uint32_t found = 0;
  int fds[SCAN_PARALLEL];
  uint32_t addrs[SCAN_PARALLEL];
  int count = 0, maxfd = -1;
  uint32_t began = millis();
  uint8_t first = scan_next_;
  int refused = 0;

  for (int i = 0; i < SCAN_PARALLEL && scan_next_ < 255; i++) {
    uint32_t addr = network | htonl((uint32_t) scan_next_);
    scan_next_++;
    // bcast is computed once per batch now. Calling subnet_broadcast() in here
    // logged our own address four extra times per batch and asked the network
    // stack the same question five times for no reason.
    if (addr == bcast || addr == network)
      continue;

    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      // The pool is exhausted - stop asking for more this round rather than
      // spinning on a failure we cannot fix.
      ESP_LOGW(TAG, "scan: no sockets available (%d)", errno);
      break;
    }
    int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in dest {};
    dest.sin_family = AF_INET;
    dest.sin_addr.s_addr = addr;
    dest.sin_port = htons(SOLARMAN_PORT);
    int rc = ::connect(fd, (struct sockaddr *) &dest, sizeof(dest));
    if (rc < 0 && errno != EINPROGRESS) {
      // Counted, not ignored. A batch where every connect is rejected up front
      // looks exactly like a batch that ran and found nothing, and telling
      // those apart is the whole point of a sweep.
      refused++;
      ::close(fd);
      continue;
    }
    fds[count] = fd;
    addrs[count] = addr;
    if (fd > maxfd)
      maxfd = fd;
    count++;
  }

  if (count > 0) {
    uint32_t deadline = millis() + SCAN_WAIT;
    while (millis() < deadline && found == 0) {
      App.feed_wdt();
      fd_set wfds;
      FD_ZERO(&wfds);
      for (int i = 0; i < count; i++)
        FD_SET(fds[i], &wfds);
      struct timeval sel {};
      sel.tv_sec = 0;
      sel.tv_usec = 50000;  // 50 ms per pass, so the watchdog is fed often
      if (::select(maxfd + 1, nullptr, &wfds, nullptr, &sel) <= 0)
        continue;
      for (int i = 0; i < count; i++) {
        if (!FD_ISSET(fds[i], &wfds))
          continue;
        int err = 0;
        socklen_t len = sizeof(err);
        ::getsockopt(fds[i], SOL_SOCKET, SO_ERROR, &err, &len);
        if (err == 0) {
          found = addrs[i];
          break;
        }
      }
    }
    for (int i = 0; i < count; i++)
      ::close(fds[i]);
  }

  // One line per batch, so a sweep that is not actually sweeping is visible
  // immediately rather than after ten minutes of silence.
  // INFO, not DEBUG: this is the line that says whether a sweep is actually
  // sweeping, and a config at INFO would have the DEBUG version stripped at
  // compile time - leaving a silent scan that looks identical to a broken one.
  ESP_LOGI(TAG, "scan: .%u-.%u  %d open attempts, %d refused at once, %u ms%s",
           (unsigned) first, (unsigned) (scan_next_ - 1), count, refused,
           (unsigned) (millis() - began), found != 0 ? "  HIT" : "");

  if (scan_next_ >= 255) {
    scan_next_ = 1;
    scan_wrapped_ = true;
    ESP_LOGW(TAG, "scan: full sweep finished, nothing on port %u", SOLARMAN_PORT);
  } else if ((scan_next_ % 40) < SCAN_PARALLEL) {
    ESP_LOGI(TAG, "scan: at .%u of 254", (unsigned) scan_next_);
  }

  if (found == 0)
    return false;

  char a[INET_ADDRSTRLEN] = {0};
  inet_ntop(AF_INET, &found, a, sizeof(a));
  ESP_LOGI(TAG, "scan: port %u open at %s - checking whether it speaks V5",
           SOLARMAN_PORT, a);
  if (verify_candidate(found))
    return true;
  // Something else lives on 8899 here. Say so and keep going: silence would
  // leave the sweep looking identical to one that found nothing at all.
  ESP_LOGW(TAG, "scan: %s did not answer V5 - not a Solarman logger", a);
  return false;
}

bool SolarmanMinimal::discover_host() {
  static const char *const PROBES[] = {"WIFIKIT-214028-READ", "HF-A11ASSISTHREAD"};

  if (serial_ == 0)
    ESP_LOGI(TAG, "UDP discovery: looking for any logger on the LAN ...");
  else
    ESP_LOGI(TAG, "UDP discovery for serial %" PRIu32 " ...", serial_);

  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    ESP_LOGW(TAG, "UDP socket() failed: %d", errno);
    return false;
  }

  int broadcast = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  // Bind so replies sent back to the discovery port reach us
  struct sockaddr_in local {};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  local.sin_port = htons(DISCOVERY_PORT);
  if (::bind(fd, (struct sockaddr *) &local, sizeof(local)) < 0) {
    ESP_LOGW(TAG, "UDP bind on port %u failed: %d", DISCOVERY_PORT, errno);
    ::close(fd);
    return false;
  }

  // A second socket, deliberately NOT bound, to send from.
  //
  // ha-solarman creates its endpoint without local_addr, so its probes leave
  // from an ephemeral port; ours left from 48899 because the same socket was
  // bound to receive on it. Sending to both broadcast addresses from the fixed
  // port was tested at a customer and answered nothing, which leaves the
  // source port as the last difference between the two implementations.
  //
  // Both sockets are then watched for the reply: the ephemeral one catches a
  // dongle that answers its sender, the bound one catches a dongle that
  // answers to 48899 or broadcasts. Neither case is given up on.
  int tx = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (tx >= 0)
    ::setsockopt(tx, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

  struct sockaddr_in dest {};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(DISCOVERY_PORT);

  uint32_t targets[2] = {htonl(INADDR_BROADCAST), 0};
  targets[1] = subnet_broadcast();

  for (uint32_t addr : targets) {
    if (addr == 0) {
      // Say so. A destination skipped in silence is indistinguishable from one
      // that was tried and ignored, and that is exactly the ambiguity that
      // made a field test prove nothing.
      ESP_LOGW(TAG, "  subnet broadcast unavailable - own address unknown");
      continue;
    }
    char a[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &addr, a, sizeof(a));
    int sent = 0, sent_eph = 0;
    dest.sin_addr.s_addr = addr;
    for (const char *probe : PROBES) {
      if (::sendto(fd, probe, strlen(probe), 0, (struct sockaddr *) &dest, sizeof(dest)) > 0)
        sent++;
      if (tx >= 0 &&
          ::sendto(tx, probe, strlen(probe), 0, (struct sockaddr *) &dest, sizeof(dest)) > 0)
        sent_eph++;
    }
    ESP_LOGI(TAG, "  probes -> %s : %d/2 from :%u, %d/2 from ephemeral", a, sent,
             DISCOVERY_PORT, sent_eph);
  }

  uint32_t deadline = millis() + DISCOVERY_WAIT;
  while (millis() < deadline) {
    App.feed_wdt();

    // Watch both sockets: the ephemeral one catches a dongle that answers its
    // sender, the bound one catches a dongle that answers to 48899 or
    // broadcasts. select() in short passes rather than a blocking recvfrom,
    // so the watchdog keeps being fed and neither socket starves the other.
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    if (tx >= 0)
      FD_SET(tx, &rfds);
    struct timeval sel {};
    sel.tv_sec = 0;
    sel.tv_usec = 100000;
    int ready = ::select((tx > fd ? tx : fd) + 1, &rfds, nullptr, nullptr, &sel);
    if (ready <= 0)
      continue;
    int src = FD_ISSET(fd, &rfds) ? fd : tx;

    char buf[256];
    struct sockaddr_in from {};
    socklen_t from_len = sizeof(from);
    int n = ::recvfrom(src, buf, sizeof(buf) - 1, 0, (struct sockaddr *) &from, &from_len);
    if (n <= 0)
      continue;  // timeout or error, keep waiting until the deadline
    buf[n] = '\0';

    char from_str[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &from.sin_addr, from_str, sizeof(from_str));
    ESP_LOGD(TAG, "UDP reply from %s on %s: %s", from_str,
             src == fd ? "the bound port" : "the ephemeral port", buf);

    uint32_t reply_serial = parse_discovery_serial(buf);
    if (reply_serial == 0) {
      ESP_LOGD(TAG, "  ignored: no serial in reply");
      continue;
    }

    if (serial_ != 0 && reply_serial != serial_) {
      ESP_LOGD(TAG, "  ignored: serial %" PRIu32 " is not the configured %" PRIu32, reply_serial, serial_);
      continue;
    }

    if (serial_ == 0) {
      // Zero-config path: adopt this logger and remember its serial, so a
      // later run can tell it apart from a second stick appearing on the LAN.
      serial_ = reply_serial;
      SolarmanSerialPrefs sp{serial_};
      serial_prefs_.save(&sp);
      prefs_ = global_preferences->make_preference<SolarmanPrefs>(serial_);
      ESP_LOGI(TAG, "Adopted logger serial %" PRIu32 " (discovered)", serial_);
    }

    host_addr_ = from.sin_addr.s_addr;
    ESP_LOGI(TAG, "Logger found at %s", host_str().c_str());
    save_host();
    ::close(fd);
    if (tx >= 0)
      ::close(tx);
    return true;
  }

  ::close(fd);
  if (tx >= 0)
    ::close(tx);
  if (serial_ == 0)
    ESP_LOGW(TAG, "No logger answered either discovery probe on this LAN");
  else
    ESP_LOGW(TAG, "Logger serial %" PRIu32 " not found on LAN", serial_);
  return false;
}

// Parse "<ip>,<mac>,<serial>" and return the serial, or 0 if the reply does
// not have that shape. Strict on purpose: a malformed reply adopted as a
// logger would send every later read to the wrong address.
uint32_t SolarmanMinimal::parse_discovery_serial(const char *reply) {
  const char *first = strchr(reply, ',');
  if (first == nullptr)
    return 0;
  const char *second = strchr(first + 1, ',');
  if (second == nullptr)
    return 0;
  const char *serial_field = second + 1;
  if (*serial_field == '\0')
    return 0;
  for (const char *p = serial_field; *p != '\0'; p++)
    if (*p < '0' || *p > '9')
      return 0;  // trailing junk or a non-numeric hostname: not a serial
  return (uint32_t) strtoul(serial_field, nullptr, 10);
}

// ── TCP register read ─────────────────────────────────────────────────────

bool SolarmanMinimal::read_registers(uint16_t start, uint8_t count, std::vector<uint16_t> &out) {
  // Cleared up front, not just overwritten at the end: every early return
  // below (socket(), connect(), send() failing) used to leave last_reply_
  // holding whatever the PREVIOUS call - on a different address - had
  // received. During the scan that meant a candidate that never answered at
  // all could inherit the real logger's serial from an earlier, unrelated
  // read and have it misreported as its own "announced serial" a few lines
  // down in verify_candidate().
  last_reply_.clear();

  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    ESP_LOGW(TAG, "TCP socket() failed: %d", errno);
    return false;
  }

  struct timeval tv;
  tv.tv_sec = CONNECT_TIMEOUT / 1000;
  tv.tv_usec = 0;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  struct sockaddr_in dest {};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = host_addr_;
  dest.sin_port = htons(SOLARMAN_PORT);

  // connect() must NOT be left blocking. SO_SNDTIMEO does not bound it on
  // LWIP, so it holds the main loop until the TCP handshake gives up - and
  // nothing feeds the task watchdog while it waits, so the device reboots
  // instead of merely polling slowly. Seen in the field the moment a dongle
  // was present but its port 8899 was already taken by another poller: on the
  // bench, with no host at all, connect fails instantly and hides the bug.
  int flags = ::fcntl(fd, F_GETFL, 0);
  ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  int rc = ::connect(fd, (struct sockaddr *) &dest, sizeof(dest));
  if (rc < 0 && errno != EINPROGRESS) {
    ESP_LOGW(TAG, "TCP connect to %s:%u failed: %d", host_str().c_str(), SOLARMAN_PORT, errno);
    note_error("8899", errno);
    ::close(fd);
    return false;
  }
  if (rc < 0) {
    bool connected = false;
    uint32_t deadline = millis() + CONNECT_TIMEOUT;
    while (millis() < deadline) {
      App.feed_wdt();
      fd_set wfds;
      FD_ZERO(&wfds);
      FD_SET(fd, &wfds);
      struct timeval sel {};
      sel.tv_sec = 0;
      sel.tv_usec = 100000;  // 100 ms per pass, so the watchdog is fed often
      int n = ::select(fd + 1, nullptr, &wfds, nullptr, &sel);
      if (n > 0) {
        int err = 0;
        socklen_t len = sizeof(err);
        ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
        connected = (err == 0);
        if (!connected)
          errno = err;
        break;
      }
      if (n < 0 && errno != EINTR)
        break;
    }
    if (!connected) {
      ESP_LOGW(TAG, "TCP connect to %s:%u failed: %d", host_str().c_str(), SOLARMAN_PORT, errno);
      note_error("8899", errno);
      ::close(fd);
      return false;
    }
  }
  ::fcntl(fd, F_SETFL, flags);  // back to blocking for send/recv

  std::vector<uint8_t> req = build_v5_request(start, count);
  if (::send(fd, req.data(), req.size(), 0) < 0) {
    ESP_LOGW(TAG, "TCP send failed: %d", errno);
    ::close(fd);
    return false;
  }

  // Minimum expected frame: 11 (header) + 15 (payload overhead)
  //                       + 1 (byte_count) + count*2 + 2 (checksum + end)
  const size_t expected = 29 + count * 2;
  std::vector<uint8_t> resp;
  resp.reserve(expected);

  uint32_t deadline = millis() + READ_TIMEOUT;
  while (resp.size() < expected && millis() < deadline) {
    App.feed_wdt();
    uint8_t chunk[128];
    int n = ::recv(fd, chunk, sizeof(chunk), 0);
    if (n > 0) {
      resp.insert(resp.end(), chunk, chunk + n);
    } else if (n == 0) {
      break;  // peer closed
    } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      ESP_LOGW(TAG, "TCP recv failed: %d", errno);
      break;
    }
  }
  ::close(fd);

  last_reply_ = resp;
  if (!parse_v5_response(resp, out)) {
    note_error("V5", resp.empty() ? 0 : -1);
    return false;
  }

  // Guard the callers' fixed indexes: a short reply must not be treated as valid
  if (out.size() < count) {
    ESP_LOGW(TAG, "Short reply for 0x%04X: got %u regs, expected %u", start, (unsigned) out.size(), count);
    return false;
  }
  return true;
}

// ── V5 frame builder ──────────────────────────────────────────────────────
//
// Frame layout (multi-byte fields are little-endian):
//
//  [0xA5]                      start
//  [len_lo][len_hi]            V5 payload length (= 15 + modbus_len)
//  [0x10][0x45]                control code 0x4510
//  [seq_lo][seq_hi]            sequence counter
//  [serial: 4 bytes LE]        logger serial number
//  ── V5 payload ──
//  [0x02]                      frame type: inverter data
//  [0x00][0x00]                sensor type
//  [0x00 x 12]                 delivery / power-on / offset timestamps
//  [modbus RTU frame: 8 bytes] slave + FC03 + addr + count + CRC16
//  ── end payload ──
//  [checksum]                  sum of bytes [1 .. N-2], truncated to uint8
//  [0x15]                      end

std::vector<uint8_t> SolarmanMinimal::build_v5_request(uint16_t reg_start, uint8_t reg_count) {
  // Modbus RTU: read holding registers. See MODBUS_FC_READ_HOLDING_REGISTERS —
  // this component never writes to the inverter.
  uint8_t mb[8] = {
      0x01,
      MODBUS_FC_READ_HOLDING_REGISTERS,
      (uint8_t) ((reg_start >> 8) & 0xFF),
      (uint8_t) (reg_start & 0xFF),
      0x00,
      reg_count,
      0x00,
      0x00,
  };
  uint16_t crc = crc16(mb, 6);
  mb[6] = crc & 0xFF;  // Modbus CRC is little-endian
  mb[7] = (crc >> 8) & 0xFF;

  const uint8_t MB_LEN = 8;
  const uint8_t V5_OVERHEAD = 15;  // frame_type(1) + sensor_type(2) + times(12)
  uint16_t payload_len = V5_OVERHEAD + MB_LEN;

  std::vector<uint8_t> f;
  f.reserve(payload_len + 13);

  f.push_back(0xA5);
  f.push_back(payload_len & 0xFF);
  f.push_back((payload_len >> 8) & 0xFF);
  f.push_back(0x10);  // control lo (0x4510 LE)
  f.push_back(0x45);  // control hi
  f.push_back(sequence_ & 0xFF);
  f.push_back((sequence_ >> 8) & 0xFF);
  sequence_++;
  f.push_back((serial_) & 0xFF);
  f.push_back((serial_ >> 8) & 0xFF);
  f.push_back((serial_ >> 16) & 0xFF);
  f.push_back((serial_ >> 24) & 0xFF);

  f.push_back(0x02);  // frame type: inverter data
  f.push_back(0x00);  // sensor type lo
  f.push_back(0x00);  // sensor type hi
  for (int i = 0; i < 12; i++)
    f.push_back(0x00);  // timestamps
  for (int i = 0; i < MB_LEN; i++)
    f.push_back(mb[i]);

  uint8_t cs = 0;
  for (size_t i = 1; i < f.size(); i++)
    cs += f[i];
  f.push_back(cs);
  f.push_back(0x15);

  return f;
}

// ── V5 response parser ────────────────────────────────────────────────────

uint32_t SolarmanMinimal::frame_serial(const std::vector<uint8_t> &frame) {
  // 0xA5 [len 2] [control 2] [sequence 2] [serial 4] ... [checksum] 0x15
  if (frame.size() < 13 || frame.front() != 0xA5 || frame.back() != 0x15)
    return 0;
  return (uint32_t) frame[7] | ((uint32_t) frame[8] << 8) |
         ((uint32_t) frame[9] << 16) | ((uint32_t) frame[10] << 24);
}

bool SolarmanMinimal::parse_v5_response(const std::vector<uint8_t> &resp, std::vector<uint16_t> &regs) {
  if (resp.size() < 20) {
    ESP_LOGW(TAG, "Response too short (%u bytes)", (unsigned) resp.size());
    return false;
  }
  if (resp.front() != 0xA5 || resp.back() != 0x15) {
    ESP_LOGW(TAG, "Invalid V5 delimiters (0x%02X .. 0x%02X)", resp.front(), resp.back());
    return false;
  }

  // Scan for the embedded Modbus reply: slave=0x01, FC=0x03, byte_count
  for (size_t i = 11; i + 4 < resp.size(); i++) {
    if (resp[i] != 0x01 || resp[i + 1] != 0x03)
      continue;
    uint8_t byte_count = resp[i + 2];
    if (byte_count == 0 || byte_count % 2 != 0)
      continue;
    if (i + 3 + byte_count + 2 > resp.size())
      continue;

    uint8_t n = byte_count / 2;
    regs.resize(n);
    for (uint8_t r = 0; r < n; r++)
      regs[r] = ((uint16_t) resp[i + 3 + r * 2] << 8) | resp[i + 4 + r * 2];
    return true;
  }

  ESP_LOGW(TAG, "Modbus reply not found in V5 frame (%u bytes)", (unsigned) resp.size());
  return false;
}

// ── Helpers ───────────────────────────────────────────────────────────────

void SolarmanMinimal::publish_signed(sensor::Sensor *s, uint16_t raw) {
  if (s)
    s->publish_state((float) (int16_t) raw);
}

uint16_t SolarmanMinimal::crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

}  // namespace solarman_minimal
}  // namespace esphome
