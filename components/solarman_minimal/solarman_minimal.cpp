#include "solarman_minimal.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

namespace esphome {
namespace solarman_minimal {

static const char *const TAG = "solarman";

// ── Register map – Deye hybrid (SUN-xK-SG0xLPx) ───────────────────────────
//
// Taken from the `deye_hybrid` profile of the ha-solarman integration, which
// is in production use against a Deye 8K SG05LP1. Verified against that
// project's own parser for the multi-register (32-bit) decoding order.
//
// Batch A  0x0060..0x0061  (2 regs)
//   Total Production  ×0.1 kWh, 32-bit UNSIGNED.
//   First register is the LOW word: value = reg[0] | (reg[1] << 16).
//
// Batch B  0x006C..0x006F  (4 regs)
//   [0] 0x006C = Daily Production  ×0.1 kWh
//   [1] 0x006D = PV1 Voltage       ×0.1 V
//   [3] 0x006F = PV2 Voltage       ×0.1 V
//
// Batch C  0x00A9..0x00B2  (10 regs)
//   [0] 0x00A9 = Grid Power  W  signed
//   [9] 0x00B2 = Load Power  W  signed
//
// Batch D  0x00B7..0x00BE  (8 regs)
//   [0] 0x00B7 = Battery Voltage  ×0.01 V
//   [1] 0x00B8 = Battery SOC      %
//   [3] 0x00BA = PV1 Power        W
//   [4] 0x00BB = PV2 Power        W
//   [7] 0x00BE = Battery Power    W  signed
//
// Two things the upstream profile leaves genuinely uncertain, both to settle
// on real hardware rather than by guessing:
//   * Grid and Load Power carry `scale: [1, 10, 10]` with the comment
//     "out of date docs". Raw watts (scale 1) is used here; if the readings
//     come out 10x off, that is the reason.
//   * Grid Power additionally carries an `inverse` attribute upstream, so the
//     sign convention below (+ import / - export) may be the other way round.
static const uint16_t REG_TOTAL_PRODUCTION = 0x0060;
static const uint16_t REG_DAILY_PRODUCTION = 0x006C;
static const uint16_t REG_GRID_POWER       = 0x00A9;
static const uint16_t REG_BATT_VOLTAGE     = 0x00B7;

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
      ESP_LOGI(TAG, "Loaded serial %u from flash", serial_);
    } else {
      ESP_LOGW(TAG, "Serial not configured - set it via the web UI");
      return;
    }
  }

  prefs_ = global_preferences->make_preference<SolarmanPrefs>(serial_);
  if (!host_override_) {
    SolarmanPrefs saved{};
    if (prefs_.load(&saved) && (saved.ip[0] | saved.ip[1] | saved.ip[2] | saved.ip[3])) {
      memcpy(&host_addr_, saved.ip, 4);
      ESP_LOGI(TAG, "Restored cached IP %s for serial %u", host_str().c_str(), serial_);
    }
  }
}

void SolarmanMinimal::dump_config() {
  ESP_LOGCONFIG(TAG, "Solarman:");
  ESP_LOGCONFIG(TAG, "  Serial: %u", serial_);
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
  // Re-bind the IP prefs slot to the new serial key
  prefs_ = global_preferences->make_preference<SolarmanPrefs>(serial_);
  // Clear any cached IP so discovery runs fresh for the new serial
  if (!host_override_)
    host_addr_ = 0;
  ESP_LOGI(TAG, "Serial set to %u - saved to flash", serial_);
}

// ── update ────────────────────────────────────────────────────────────────

void SolarmanMinimal::update() {
  if (serial_ == 0) {
    ESP_LOGD(TAG, "Waiting for serial number to be configured");
    return;
  }
  if (!has_host() && !discover_host()) {
    ESP_LOGW(TAG, "Discovery failed - will retry next cycle");
    return;
  }

  // ── Batch D first: battery + PV power ───────────────────────────────────
  // Read first because it drives the failure counter — if the inverter is
  // unreachable there is no point attempting the other three batches.
  std::vector<uint16_t> d;
  if (!read_registers(REG_BATT_VOLTAGE, 8, d)) {
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
  if (battery_voltage_) battery_voltage_->publish_state(d[0] * 0.01f);
  if (battery_soc_)     battery_soc_->publish_state(d[1]);
  if (pv1_power_)       pv1_power_->publish_state(d[3]);
  if (pv2_power_)       pv2_power_->publish_state(d[4]);
  if (battery_power_)   publish_signed(battery_power_, d[7]);

  // ── Batch B: daily yield + PV voltages ──────────────────────────────────
  std::vector<uint16_t> b;
  if (read_registers(REG_DAILY_PRODUCTION, 4, b)) {
    if (daily_production_) daily_production_->publish_state(b[0] * 0.1f);
    if (pv1_voltage_)      pv1_voltage_->publish_state(b[1] * 0.1f);
    if (pv2_voltage_)      pv2_voltage_->publish_state(b[3] * 0.1f);
  }

  // ── Batch C: grid + load power ──────────────────────────────────────────
  std::vector<uint16_t> c;
  if (read_registers(REG_GRID_POWER, 10, c)) {
    if (grid_power_) publish_signed(grid_power_, c[0]);
    if (load_power_) publish_signed(load_power_, c[9]);
  }

  // ── Batch A: total production (32-bit, low word first) ──────────────────
  if (total_production_) {
    std::vector<uint16_t> a;
    if (read_registers(REG_TOTAL_PRODUCTION, 2, a)) {
      uint32_t raw = (uint32_t) a[0] | ((uint32_t) a[1] << 16);
      total_production_->publish_state(raw * 0.1f);
    }
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
// Solarman logger sticks answer a broadcast probe on UDP port 48899.
// Reply looks like: "<ip>,<mac>,<serial>"  (exact format varies by firmware),
// so we match on the serial number appearing anywhere in the payload.

bool SolarmanMinimal::discover_host() {
  ESP_LOGI(TAG, "UDP discovery for serial %u ...", serial_);

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

  struct sockaddr_in dest {};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  dest.sin_port = htons(DISCOVERY_PORT);

  static const char PROBE[] = "WIFIKIT-214028-READ";
  ::sendto(fd, PROBE, strlen(PROBE), 0, (struct sockaddr *) &dest, sizeof(dest));

  char serial_str[12];
  snprintf(serial_str, sizeof(serial_str), "%u", serial_);

  uint32_t deadline = millis() + DISCOVERY_WAIT;
  while (millis() < deadline) {
    char buf[256];
    struct sockaddr_in from {};
    socklen_t from_len = sizeof(from);
    int n = ::recvfrom(fd, buf, sizeof(buf) - 1, 0, (struct sockaddr *) &from, &from_len);
    if (n <= 0)
      continue;  // timeout or error, keep waiting until the deadline
    buf[n] = '\0';

    char from_str[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &from.sin_addr, from_str, sizeof(from_str));
    ESP_LOGD(TAG, "UDP reply from %s: %s", from_str, buf);

    if (strstr(buf, serial_str) != nullptr) {
      host_addr_ = from.sin_addr.s_addr;
      ESP_LOGI(TAG, "Logger found at %s", host_str().c_str());
      save_host();
      ::close(fd);
      return true;
    }
  }

  ::close(fd);
  ESP_LOGW(TAG, "Logger serial %u not found on LAN", serial_);
  return false;
}

// ── TCP register read ─────────────────────────────────────────────────────

bool SolarmanMinimal::read_registers(uint16_t start, uint8_t count, std::vector<uint16_t> &out) {
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

  if (::connect(fd, (struct sockaddr *) &dest, sizeof(dest)) < 0) {
    ESP_LOGW(TAG, "TCP connect to %s:%u failed: %d", host_str().c_str(), SOLARMAN_PORT, errno);
    ::close(fd);
    return false;
  }

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

  if (!parse_v5_response(resp, out))
    return false;

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
