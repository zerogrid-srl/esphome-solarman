import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_KILOWATT_HOURS,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)

DEPENDENCIES = ["wifi"]
# This component creates sensors itself rather than being a sensor platform, so
# nothing else guarantees the sensor component gets pulled into the build. A
# config whose only sensors come from here fails to compile without this, with
# a misleading "esphome/components/sensor/sensor.h: No such file or directory".
AUTO_LOAD = ["sensor"]

solarman_minimal_ns = cg.esphome_ns.namespace("solarman_minimal")
SolarmanMinimal = solarman_minimal_ns.class_("SolarmanMinimal", cg.PollingComponent)

CONF_HOST             = "host"
CONF_TCP_SCAN         = "tcp_scan"
CONF_SERIAL           = "serial"
CONF_PV1_POWER        = "pv1_power"
CONF_PV1_VOLTAGE      = "pv1_voltage"
CONF_PV2_POWER        = "pv2_power"
CONF_PV2_VOLTAGE      = "pv2_voltage"
CONF_BATTERY_SOC      = "battery_soc"
CONF_BATTERY_VOLTAGE  = "battery_voltage"
CONF_BATTERY_POWER    = "battery_power"
CONF_GRID_POWER       = "grid_power"
CONF_LOAD_POWER       = "load_power"
CONF_DAILY_PRODUCTION = "daily_production"
CONF_TOTAL_PRODUCTION = "total_production"
CONF_DAILY_CONSUMPTION = "daily_consumption"
CONF_TOTAL_CONSUMPTION = "total_consumption"
CONF_DAILY_ENERGY_BOUGHT = "daily_energy_bought"
CONF_DAILY_ENERGY_SOLD = "daily_energy_sold"
CONF_DAILY_BATTERY_CHARGE = "daily_battery_charge"
CONF_DAILY_BATTERY_DISCHARGE = "daily_battery_discharge"
CONF_DEVICE_STATE = "device_state"
CONF_GRID_CONNECTED = "grid_connected"


def _energy_schema():
    """Every daily/total counter on this inverter is the same shape: kWh at one
    decimal, monotonic within its period."""
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
    )

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SolarmanMinimal),
        # Serial number of the WiFi logger stick.
        # Can be omitted if set at runtime via the "Logger Serial" text entity + web_server.
        cv.Optional(CONF_SERIAL, default=0): cv.uint32_t,
        # Optional: skip UDP discovery and use a fixed IP
        cv.Optional(CONF_HOST): cv.string,
        # Sweep the local /24 for an open port 8899 when UDP discovery finds
        # nothing. Off by default: it is a scan of someone else's network and
        # should be a deliberate choice, not a surprise. Five dongles across
        # four sites never answered a UDP probe while all of them served 8899,
        # so on those this is the only discovery with a chance of working.
        cv.Optional(CONF_TCP_SCAN, default=False): cv.boolean,

        cv.Optional(CONF_PV1_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PV1_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PV2_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PV2_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_SOC): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Signed. Positive = DISCHARGING, confirmed on a Deye 8K SG05LP1:
        # with PV at 2 W, grid at 0 W and load at 2705 W the register read
        # +2813 W, so a positive value is power leaving the battery.
        cv.Optional(CONF_BATTERY_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Signed, but the direction convention is NOT yet confirmed — it read
        # 0 W throughout the only field test so far. Verify against a known
        # reference during grid import or export before trusting the sign.
        cv.Optional(CONF_GRID_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_LOAD_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DAILY_PRODUCTION): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_TOTAL_PRODUCTION): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_DAILY_CONSUMPTION): _energy_schema(),
        cv.Optional(CONF_TOTAL_CONSUMPTION): _energy_schema(),
        cv.Optional(CONF_DAILY_ENERGY_BOUGHT): _energy_schema(),
        cv.Optional(CONF_DAILY_ENERGY_SOLD): _energy_schema(),
        cv.Optional(CONF_DAILY_BATTERY_CHARGE): _energy_schema(),
        cv.Optional(CONF_DAILY_BATTERY_DISCHARGE): _energy_schema(),
        # Enum from register 0x003B: 0 standby, 1 self-test, 2 normal,
        # 3 alarm, 4 fault. Published raw; label it where it is displayed.
        cv.Optional(CONF_DEVICE_STATE): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # 1 = tied to the utility, 0 = islanded. Numeric rather than a
        # binary_sensor so this component keeps a single entity type.
        cv.Optional(CONF_GRID_CONNECTED): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.polling_component_schema("30s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_serial(config[CONF_SERIAL]))  # 0 = load from flash
    cg.add(var.set_tcp_scan(config[CONF_TCP_SCAN]))
    if CONF_HOST in config:
        cg.add(var.set_host(config[CONF_HOST]))

    sensor_map = [
        (CONF_PV1_POWER,        "set_pv1_power_sensor"),
        (CONF_PV1_VOLTAGE,      "set_pv1_voltage_sensor"),
        (CONF_PV2_POWER,        "set_pv2_power_sensor"),
        (CONF_PV2_VOLTAGE,      "set_pv2_voltage_sensor"),
        (CONF_BATTERY_SOC,      "set_battery_soc_sensor"),
        (CONF_BATTERY_VOLTAGE,  "set_battery_voltage_sensor"),
        (CONF_BATTERY_POWER,    "set_battery_power_sensor"),
        (CONF_GRID_POWER,       "set_grid_power_sensor"),
        (CONF_LOAD_POWER,       "set_load_power_sensor"),
        (CONF_DAILY_PRODUCTION, "set_daily_production_sensor"),
        (CONF_TOTAL_PRODUCTION, "set_total_production_sensor"),
        (CONF_DAILY_CONSUMPTION, "set_daily_consumption_sensor"),
        (CONF_TOTAL_CONSUMPTION, "set_total_consumption_sensor"),
        (CONF_DAILY_ENERGY_BOUGHT, "set_daily_energy_bought_sensor"),
        (CONF_DAILY_ENERGY_SOLD, "set_daily_energy_sold_sensor"),
        (CONF_DAILY_BATTERY_CHARGE, "set_daily_battery_charge_sensor"),
        (CONF_DAILY_BATTERY_DISCHARGE, "set_daily_battery_discharge_sensor"),
        (CONF_DEVICE_STATE, "set_device_state_sensor"),
        (CONF_GRID_CONNECTED, "set_grid_connected_sensor"),
    ]
    for conf_key, setter in sensor_map:
        if conf_key in config:
            sens = await sensor.new_sensor(config[conf_key])
            cg.add(getattr(var, setter)(sens))
