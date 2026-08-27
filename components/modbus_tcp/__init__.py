import esphome.codegen as cg
from esphome.components import modbus
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT, Framework

CODEOWNERS = ["@mikesnet"]
# The vendored copy of the core component, which is where ModbusClientHub and the
# whole ModbusClientDevice surface live. Nothing here reimplements them.
DEPENDENCIES = ["modbus", "network"]
MULTI_CONF = True

CONF_HOST = "host"
CONF_SEND_WAIT_TIME = "send_wait_time"
CONF_RECONNECT_INTERVAL = "reconnect_interval"

modbus_tcp_ns = cg.esphome_ns.namespace("modbus_tcp")

# Declared as a subclass of the hub the devices already bind to. That is the
# whole integration story: cv.use_id(modbus.ModbusClient) accepts a subclass, so
# growatt_master's bus keys take one of these without a line of Python changing.
ModbusTcpClientHub = modbus_tcp_ns.class_(
    "ModbusTcpClientHub", modbus.ModbusClient, cg.Component
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ModbusTcpClientHub),
            cv.Required(CONF_HOST): cv.string_strict,
            cv.Optional(CONF_PORT, default=502): cv.port,
            # Matches the hub's own default. Over TCP this is the only response
            # deadline that exists - there is no inter-character silence to fall
            # back on - so it is worth setting deliberately per link.
            cv.Optional(
                CONF_SEND_WAIT_TIME, default="2000ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_RECONNECT_INTERVAL, default="5s"
            ): cv.positive_time_period_milliseconds,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    cv.only_with_framework(Framework.ESP_IDF),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_host(config[CONF_HOST]))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_send_wait_time(config[CONF_SEND_WAIT_TIME]))
    cg.add(var.set_reconnect_interval(config[CONF_RECONNECT_INTERVAL]))
