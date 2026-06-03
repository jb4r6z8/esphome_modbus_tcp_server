import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import modbus

MULTI_CONF = True
CODEOWNERS = []


modbus_tcp_server_ns = cg.esphome_ns.namespace("modbus_tcp_server")
ModbusTcpServerComponent = modbus_tcp_server_ns.class_("ModbusTcpServerComponent", cg.Component)

CONF_PORT = "port"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ModbusTcpServerComponent),
            cv.Optional(CONF_PORT, default=502): cv.port,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
    )
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))
