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
static const uint32_t CONNECT_TIMEOUT = 5000;  // ms
static const uint32_t READ_TIMEOUT    = 3000;  // ms
static const uint32_t DISCOVERY_WAIT  = 5000;  // ms
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

  // Reads key register ranges and logs every non-zero value. Use on unknown
  // firmware to identify the real register map before trusting the defaults.
  void scan_registers();

 protected:
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

  bool has_host() const { return host_addr_ != 0; }
  std::string host_str() const;
  void save_host();

  bool discover_host();
  bool read_registers(uint16_t start, uint8_t count, std::vector<uint16_t> &out);
  std::vector<uint8_t> build_v5_request(uint16_t reg_start, uint8_t reg_count);
  bool parse_v5_response(const std::vector<uint8_t> &resp, std::vector<uint16_t> &regs);
  void publish_signed(sensor::Sensor *s, uint16_t raw);
  uint16_t crc16(const uint8_t *data, size_t len);
};

}  // namespace solarman_minimal
}  // namespace esphome
