#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"

#include <string>
#include <vector>
#include <cstdint>

namespace esphome {
namespace solarman_minimal {

static const uint16_t SOLARMAN_PORT   = 8899;
static const uint16_t DISCOVERY_PORT  = 48899;
// Bounded by a non-blocking connect plus select, not by SO_SNDTIMEO,
// which LWIP ignores for connect().
static const uint32_t CONNECT_TIMEOUT = 3000;  // ms
static const uint32_t READ_TIMEOUT    = 3000;  // ms
// Two probes get answered in milliseconds on a LAN; the old 5 s was
// generosity paid for out of the main loop's budget.
static const uint32_t DISCOVERY_WAIT  = 2500;  // ms
// Re-run discovery only after this many consecutive read failures, so a single
// dropped packet does not throw away a working IP.
static const uint8_t MAX_FAILURES = 3;

struct SolarmanPrefs {
  uint8_t ip[4];
};

struct SolarmanSerialPrefs {
  uint32_t serial;
};

class SolarmanMinimal : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_serial(uint32_t serial) { serial_ = serial; }
  // Called from YAML on_value when the user types the serial in the web UI
  void set_serial_from_text(const std::string &s);
  void set_host(const std::string &host);
  std::string host_str() const;

  // ── Sensors ──────────────────────────────────────────────────────────────
  void set_pv1_power_sensor(sensor::Sensor *s)        { pv1_power_ = s; }
  void set_pv1_voltage_sensor(sensor::Sensor *s)      { pv1_voltage_ = s; }
  void set_pv2_power_sensor(sensor::Sensor *s)        { pv2_power_ = s; }
  void set_pv2_voltage_sensor(sensor::Sensor *s)      { pv2_voltage_ = s; }
  void set_battery_soc_sensor(sensor::Sensor *s)      { battery_soc_ = s; }
  void set_battery_voltage_sensor(sensor::Sensor *s)  { battery_voltage_ = s; }
  void set_battery_power_sensor(sensor::Sensor *s)    { battery_power_ = s; }
  void set_grid_power_sensor(sensor::Sensor *s)       { grid_power_ = s; }
  void set_load_power_sensor(sensor::Sensor *s)       { load_power_ = s; }
  void set_daily_production_sensor(sensor::Sensor *s) { daily_production_ = s; }
  void set_total_production_sensor(sensor::Sensor *s) { total_production_ = s; }
  void set_daily_consumption_sensor(sensor::Sensor *s)       { daily_consumption_ = s; }
  void set_total_consumption_sensor(sensor::Sensor *s)       { total_consumption_ = s; }
  void set_daily_energy_bought_sensor(sensor::Sensor *s)     { daily_energy_bought_ = s; }
  void set_daily_energy_sold_sensor(sensor::Sensor *s)       { daily_energy_sold_ = s; }
  void set_daily_battery_charge_sensor(sensor::Sensor *s)    { daily_battery_charge_ = s; }
  void set_daily_battery_discharge_sensor(sensor::Sensor *s) { daily_battery_discharge_ = s; }
  void set_device_state_sensor(sensor::Sensor *s)            { device_state_ = s; }
  void set_grid_connected_sensor(sensor::Sensor *s)          { grid_connected_ = s; }

  // What the component is actually talking to, for the screen. Without this
  // a failing site gives you nothing to go on: the panel says "unreachable"
  // and you cannot tell whether it has the wrong address, no address, or the
  // right one and a dead port.
  std::string status_line() const {
    if (serial_ == 0 && host_addr_ == 0)
      return "nessun logger trovato sulla rete";
    std::string h = host_addr_ == 0 ? std::string("indirizzo sconosciuto") : host_str();
    if (serial_ == 0)
      return h + "  (seriale non impostato)";
    char b[96];
    snprintf(b, sizeof(b), "%s  sn %u", h.c_str(), (unsigned) serial_);
    std::string out(b);
    if (!last_error_.empty())
      out += std::string(1, '\n') + last_error_;
    return out;
  }

  // Why the last attempt failed, in words. This has to reach the SCREEN and
  // not just the log: on a dongle whose access point accepts one client at a
  // time, connecting a phone to read the web log disconnects the display, so
  // the panel is the only channel left.
  void note_error(const char *what, int err) {
    const char *why;
    switch (err) {
      case 111: why = "porta chiusa"; break;          // ECONNREFUSED
      case 113: why = "host irraggiungibile"; break;  // EHOSTUNREACH
      case 118: why = "rete irraggiungibile"; break;  // ENETUNREACH
      case 110: why = "nessuna risposta"; break;      // ETIMEDOUT
      case 0:   why = "nessuna risposta"; break;      // our own select() timeout
      default:  why = "errore"; break;
    }
    char b[64];
    snprintf(b, sizeof(b), "%s: %s (%d)", what, why, err);
    last_error_ = b;
  }

  void clear_error() { last_error_.clear(); }

  // Reads key register ranges and logs every non-zero value. Use on unknown
  // firmware to identify the real register map before trusting the defaults.
  void scan_registers();

 protected:
  std::string last_error_;
  uint32_t serial_{0};
  uint32_t host_addr_{0};  // IPv4, network byte order; 0 = unknown
  bool host_override_{false};
  uint16_t sequence_{0};
  uint8_t consecutive_failures_{0};
  ESPPreferenceObject prefs_;
  ESPPreferenceObject serial_prefs_;

  sensor::Sensor *pv1_power_{nullptr};
  sensor::Sensor *pv1_voltage_{nullptr};
  sensor::Sensor *pv2_power_{nullptr};
  sensor::Sensor *pv2_voltage_{nullptr};
  sensor::Sensor *battery_soc_{nullptr};
  sensor::Sensor *battery_voltage_{nullptr};
  sensor::Sensor *battery_power_{nullptr};
  sensor::Sensor *grid_power_{nullptr};
  sensor::Sensor *load_power_{nullptr};
  sensor::Sensor *daily_production_{nullptr};
  sensor::Sensor *total_production_{nullptr};
  sensor::Sensor *daily_consumption_{nullptr};
  sensor::Sensor *total_consumption_{nullptr};
  sensor::Sensor *daily_energy_bought_{nullptr};
  sensor::Sensor *daily_energy_sold_{nullptr};
  sensor::Sensor *daily_battery_charge_{nullptr};
  sensor::Sensor *daily_battery_discharge_{nullptr};
  sensor::Sensor *device_state_{nullptr};
  sensor::Sensor *grid_connected_{nullptr};

  bool has_host() const { return host_addr_ != 0; }
  void save_host();

  bool discover_host();
  static uint32_t parse_discovery_serial(const char *reply);
  bool read_registers(uint16_t start, uint8_t count, std::vector<uint16_t> &out);
  std::vector<uint8_t> build_v5_request(uint16_t reg_start, uint8_t reg_count);
  bool parse_v5_response(const std::vector<uint8_t> &resp, std::vector<uint16_t> &regs);
  void publish_signed(sensor::Sensor *s, uint16_t raw);
  uint16_t crc16(const uint8_t *data, size_t len);
};

}  // namespace solarman_minimal
}  // namespace esphome
