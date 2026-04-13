import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import fan, output, spi
from esphome.components import text_sensor
from esphome.const import (
    CONF_DIRECTION_OUTPUT,
    CONF_OSCILLATION_OUTPUT,
    CONF_OUTPUT,
    CONF_OUTPUT_ID,
)
from .. import quiet_cool_ns

# Additional pin configuration keys
CONF_GDO0_PIN = "gdo0_pin"
CONF_GDO2_PIN = "gdo2_pin"
CONF_REMOTE_ID = "remote_id"
CONF_FREQ_MHZ = "center_freq_mhz"
CONF_DEVIATION_KHZ = "deviation_khz"
CONF_SPEED_COUNT = "speed_count"
CONF_RX_PACKET_SENSOR = "rx_packet_sensor"

DEPENDENCIES = ["spi", "text_sensor"]

QuietCoolFan = quiet_cool_ns.class_("QuietCoolFan", cg.Component, fan.Fan, spi.SPIDevice)

CONFIG_SCHEMA = fan.fan_schema(QuietCoolFan).extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(QuietCoolFan),
        cv.Required(CONF_GDO0_PIN                      ): cv.uint8_t,
        cv.Required(CONF_GDO2_PIN                      ): cv.uint8_t,
        cv.Required(CONF_REMOTE_ID                     ): cv.ensure_list(cv.hex_uint8_t),
        cv.Optional(CONF_RX_PACKET_SENSOR               ): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_FREQ_MHZ     , default=433.897): cv.float_,
        cv.Optional(CONF_DEVIATION_KHZ, default=10.0   ): cv.float_,
        cv.Optional(CONF_SPEED_COUNT  , default=3      ): cv.int_range(min=2, max=3)
    }
).extend(cv.COMPONENT_SCHEMA).extend(spi.spi_device_schema(cs_pin_required=True))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])  # type: QuietCoolFan
    await cg.register_component(var, config)
    await fan.register_fan(var, config)
    await spi.register_spi_device(var, config)

    cs_num = config[spi.CONF_CS_PIN]["number"]
    cg.add(var.set_pins(cs_num, config[CONF_GDO0_PIN], config[CONF_GDO2_PIN]))
    cg.add(var.set_remote_id(config[CONF_REMOTE_ID]))
    cg.add(var.set_frequencies(config[CONF_FREQ_MHZ], config[CONF_DEVIATION_KHZ]))
    cg.add(var.set_speed_count(config[CONF_SPEED_COUNT]))
    if CONF_RX_PACKET_SENSOR in config:
        rx_sens = await text_sensor.new_text_sensor(config[CONF_RX_PACKET_SENSOR])
        cg.add(var.set_rx_packet_text_sensor(rx_sens))
