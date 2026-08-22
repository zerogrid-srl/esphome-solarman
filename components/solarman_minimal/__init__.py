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

solarman_minimal_ns = cg.esphome_ns.namespace("solarman_minimal")
SolarmanMinimal = solarman_minimal_ns.class_("SolarmanMinimal", cg.PollingComponent)

CONF_HOST             = "host"
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

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SolarmanMinimal),
        # Serial number of the WiFi logger stick.
        # Can be omitted if set at runtime via the "Logger Serial" text entity + web_server.
        cv.Optional(CONF_SERIAL, default=0): cv.uint32_t,
        # Optional: skip UDP discovery and use a fixed IP
        cv.Optional(CONF_HOST): cv.string,

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
        # Signed: positive = charging, negative = discharging
        cv.Optional(CONF_BATTERY_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Signed: positive = import from grid, negative = export to grid
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
    }
).extend(cv.polling_component_schema("30s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_serial(config[CONF_SERIAL]))  # 0 = load from flash
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
    ]
    for conf_key, setter in sensor_map:
        if conf_key in config:
            sens = await sensor.new_sensor(config[conf_key])
            cg.add(getattr(var, setter)(sens))
