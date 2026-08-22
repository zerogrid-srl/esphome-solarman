# esphome-solarman

ESPHome external component that reads **Deye hybrid inverters** over the local
network, through the Solarman V5 protocol spoken by the WiFi logger stick
(TCP port 8899). No cloud, no Home Assistant required.

The logger's IP is found automatically by UDP broadcast, so the only thing a
user has to supply is the **logger serial number** — and even that can be typed
in at runtime from the web UI instead of being hardcoded in YAML.

> **Status: the register map is not yet confirmed.** The addresses below are the
> commonly documented ones for the Deye SG series, but they have not been
> verified against real hardware. Use the `scan_registers()` action to dump the
> real map from your own inverter before trusting the sensor values. See
> [Confirming the register map](#confirming-the-register-map).

## Requirements

- An **ESP32** (the component uses POSIX sockets; ESP8266 is not supported)
- Either `arduino` or `esp-idf` framework
- The ESP32 on the **same LAN** as the inverter's WiFi logger stick

## Installation

```yaml
external_components:
  - source: github://zerogrid-srl/esphome-solarman@v0.1.0
    components: [solarman_minimal]
```

Pin a tag, not a branch. A branch moves under you, and the first sign is a build
that fails without you having changed anything.

## Usage

```yaml
solarman_minimal:
  id: solarman
  update_interval: 30s
  # serial: 2712345678   # optional — otherwise set at runtime, see below
  # host: 192.168.1.100  # optional — otherwise found by UDP discovery

  battery_soc:
    name: "Battery SOC"
  battery_power:
    name: "Battery power"       # signed: + charging, - discharging
  grid_power:
    name: "Grid power"          # signed: + import, - export
  load_power:
    name: "Load power"
  pv1_power:
    name: "PV1 power"
  pv2_power:
    name: "PV2 power"
  daily_production:
    name: "Daily production"
  total_production:
    name: "Total production"
```

All sensors are optional — declare only what you need. Also available:
`battery_voltage`, `pv1_voltage`, `pv2_voltage`.

### Setting the serial at runtime

Rather than hardcoding the serial, expose a text entity and let the user type it
once. It is persisted to flash (NVS) and survives reboots.

```yaml
web_server:
  port: 80

text:
  - platform: template
    name: "Logger Serial"
    mode: text
    restore_value: true
    optimistic: true
    on_value:
      - lambda: id(solarman).set_serial_from_text(x);
```

## Confirming the register map

Deye firmware variants place registers at different addresses. Single-phase
hybrids (`SUN-xK-SG05LP1`) keep battery, PV, grid and load around registers
580-700, while other models use a lower block. `scan_registers()` reads both
families and logs every **non-zero** register, so one run identifies the layout:

```yaml
button:
  - platform: template
    name: "Scan Registers"
    on_press:
      - lambda: id(solarman).scan_registers();
```

Set `logger: level: DEBUG`, press the button, and read the log. Each value is
printed unsigned, signed and ×0.1 so the scaling is obvious. Then adjust the
register constants at the top of `solarman_minimal.cpp`.

### Finding the serial number if you don't know it

It is printed on the logger stick and shown in the SolarmanPV app. Failing that:
enter any number, and watch the log at `DEBUG` level. Discovery logs **every**
UDP reply before filtering by serial, so the logger's real serial appears in:

```
[D][solarman] UDP reply from 192.168.1.50: ...,...,2712345678
```

## How it works

Solarman V5 wraps a standard Modbus RTU frame in its own envelope:

```
[0xA5][len LE][0x4510 LE][seq LE][logger serial LE]
  [0x02 frame type][sensor type][12 timestamp bytes]
  [Modbus RTU: slave + FC03 + address + count + CRC16]
[checksum][0x15]
```

Discovery broadcasts `WIFIKIT-214028-READ` on UDP port 48899 and matches the
configured serial against the replies. The resolved IP is cached in flash, and
re-discovered only after several consecutive read failures — so a single dropped
packet does not throw away a working address.

Registers are read in batches rather than one connection per value.

## Licence

MIT
