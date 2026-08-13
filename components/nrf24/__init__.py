from esphome import automation, pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi
from esphome.const import (
    CONF_ADDRESS,
    CONF_CHANNEL,
    CONF_COUNT,
    CONF_DATA,
    CONF_DELAY,
    CONF_ID,
    CONF_IRQ_PIN,
)
from esphome.core import ID

CODEOWNERS = ["@ezpl"]
DEPENDENCIES = ["spi"]
MULTI_CONF = True

MAX_PAYLOAD = 32

nrf24_ns = cg.esphome_ns.namespace("nrf24")
NRF24Component = nrf24_ns.class_("NRF24Component", cg.Component, spi.SPIDevice)

NRF24SendAction = nrf24_ns.class_("NRF24SendAction", automation.Action)
NRF24SetChannelAction = nrf24_ns.class_("NRF24SetChannelAction", automation.Action)
NRF24SetTxAddressAction = nrf24_ns.class_("NRF24SetTxAddressAction", automation.Action)
NRF24StartListeningAction = nrf24_ns.class_(
    "NRF24StartListeningAction", automation.Action
)
NRF24StopListeningAction = nrf24_ns.class_(
    "NRF24StopListeningAction", automation.Action
)

# The register-level enums live in the vendor header's own namespace.
nrf24l01_ns = cg.global_ns.namespace("nRF24L01")

rf24_datarate_e = nrf24l01_ns.enum("rf24_datarate_e")
RF_DATA_RATES = {
    "250KBPS": rf24_datarate_e.RF24_250KBPS,
    "1MBPS": rf24_datarate_e.RF24_1MBPS,
    "2MBPS": rf24_datarate_e.RF24_2MBPS,
}

rf24_pa_dbm_e = nrf24l01_ns.enum("rf24_pa_dbm_e")
PA_LEVELS = {
    "MIN": rf24_pa_dbm_e.RF24_PA_MIN,
    "LOW": rf24_pa_dbm_e.RF24_PA_LOW,
    "HIGH": rf24_pa_dbm_e.RF24_PA_HIGH,
    "MAX": rf24_pa_dbm_e.RF24_PA_MAX,
}

rf24_crclength_e = nrf24l01_ns.enum("rf24_crclength_e")
CRC_LENGTHS = {
    "DISABLED": rf24_crclength_e.RF24_CRC_DISABLED,
    "8": rf24_crclength_e.RF24_CRC_8,
    "16": rf24_crclength_e.RF24_CRC_16,
}

CONF_CE_PIN = "ce_pin"
CONF_RF_DATA_RATE = "rf_data_rate"
CONF_PA_LEVEL = "pa_level"
CONF_CRC = "crc"
CONF_ADDRESS_WIDTH = "address_width"
CONF_AUTO_ACK = "auto_ack"
CONF_RETRIES = "retries"
CONF_PAYLOAD_SIZE = "payload_size"
CONF_DYNAMIC_PAYLOADS = "dynamic_payloads"
CONF_TX_ADDRESS = "tx_address"
CONF_LISTEN = "listen"
CONF_ON_PACKET = "on_packet"


def validate_raw_data(value):
    """Accept a quoted string or a list of bytes, exactly like uart.write."""
    if isinstance(value, str):
        return value.encode("utf-8")
    if isinstance(value, list):
        return cv.Schema([cv.hex_uint8_t])(value)
    raise cv.Invalid(
        "data must either be a string wrapped in quotes or a list of bytes"
    )


def validate_payload(value):
    data = validate_raw_data(value)
    if len(data) == 0:
        raise cv.Invalid("payload must not be empty")
    if len(data) > MAX_PAYLOAD:
        raise cv.Invalid(
            f"the nRF24L01+ cannot send more than {MAX_PAYLOAD} bytes in one packet, got {len(data)}"
        )
    return data


def validate_address(value):
    data = validate_raw_data(value)
    if not 2 <= len(data) <= 5:
        raise cv.Invalid(f"an address must be 2 to 5 bytes long, got {len(data)}")
    return data


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NRF24Component),
            cv.Required(CONF_CE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_IRQ_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_CHANNEL, default=76): cv.int_range(min=0, max=125),
            cv.Optional(CONF_RF_DATA_RATE, default="1MBPS"): cv.enum(
                RF_DATA_RATES, upper=True
            ),
            cv.Optional(CONF_PA_LEVEL, default="MAX"): cv.enum(PA_LEVELS, upper=True),
            cv.Optional(CONF_CRC, default="16"): cv.enum(CRC_LENGTHS, upper=True),
            cv.Optional(CONF_ADDRESS_WIDTH, default=5): cv.int_range(min=2, max=5),
            cv.Optional(CONF_AUTO_ACK, default=True): cv.boolean,
            cv.Optional(CONF_RETRIES): cv.Schema(
                {
                    cv.Optional(CONF_DELAY, default=5): cv.int_range(min=0, max=15),
                    cv.Optional(CONF_COUNT, default=15): cv.int_range(min=0, max=15),
                }
            ),
            cv.Optional(CONF_PAYLOAD_SIZE, default=MAX_PAYLOAD): cv.int_range(
                min=1, max=MAX_PAYLOAD
            ),
            cv.Optional(CONF_DYNAMIC_PAYLOADS, default=False): cv.boolean,
            cv.Optional(CONF_TX_ADDRESS): validate_address,
            cv.Optional(CONF_LISTEN, default=False): cv.boolean,
            cv.Optional(CONF_ON_PACKET): automation.validate_automation(),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    ce_pin = await cg.gpio_pin_expression(config[CONF_CE_PIN])
    cg.add(var.set_ce_pin(ce_pin))
    if irq_pin_config := config.get(CONF_IRQ_PIN):
        irq_pin = await cg.gpio_pin_expression(irq_pin_config)
        cg.add(var.set_irq_pin(irq_pin))

    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(var.set_rf_data_rate(config[CONF_RF_DATA_RATE]))
    cg.add(var.set_pa_level(config[CONF_PA_LEVEL], True))
    cg.add(var.set_crc_length(config[CONF_CRC]))
    cg.add(var.set_address_width(config[CONF_ADDRESS_WIDTH]))
    cg.add(var.set_auto_ack(config[CONF_AUTO_ACK]))
    cg.add(var.set_payload_size(config[CONF_PAYLOAD_SIZE]))
    cg.add(var.set_dynamic_payloads(config[CONF_DYNAMIC_PAYLOADS]))

    if retries := config.get(CONF_RETRIES):
        cg.add(var.set_retries(retries[CONF_DELAY], retries[CONF_COUNT]))

    if tx_address := config.get(CONF_TX_ADDRESS):
        arr_id = ID(f"{config[CONF_ID].id}_tx_address", is_declaration=True, type=cg.uint8)
        arr = cg.static_const_array(arr_id, cg.ArrayInitializer(*list(tx_address)))
        cg.add(var.set_tx_address(arr, len(tx_address)))

    if config[CONF_LISTEN]:
        cg.add(var.start_listening())

    for conf in config.get(CONF_ON_PACKET, []):
        await automation.build_callback_automation(
            var,
            "add_on_packet_callback",
            [(cg.std_vector.template(cg.uint8), "x")],
            conf,
        )


async def _byte_array_action_to_code(config, action_id, template_arg, args, key):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    data = config[key]
    if isinstance(data, bytes):
        data = list(data)

    if cg.is_template(data):
        templ = await cg.templatable(data, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_data_template(templ))
    else:
        # Literal bytes live in flash, not in a RAM copy.
        arr_id = ID(f"{action_id}_data", is_declaration=True, type=cg.uint8)
        arr = cg.static_const_array(arr_id, cg.ArrayInitializer(*data))
        cg.add(var.set_data_static(arr, len(data)))
    return var


@automation.register_action(
    "nrf24.send",
    NRF24SendAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(NRF24Component),
            cv.Required(CONF_DATA): cv.templatable(validate_payload),
        },
        key=CONF_DATA,
    ),
    synchronous=True,
)
async def nrf24_send_to_code(config, action_id, template_arg, args):
    return await _byte_array_action_to_code(
        config, action_id, template_arg, args, CONF_DATA
    )


@automation.register_action(
    "nrf24.set_tx_address",
    NRF24SetTxAddressAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(NRF24Component),
            cv.Required(CONF_ADDRESS): cv.templatable(validate_address),
        },
        key=CONF_ADDRESS,
    ),
    synchronous=True,
)
async def nrf24_set_tx_address_to_code(config, action_id, template_arg, args):
    return await _byte_array_action_to_code(
        config, action_id, template_arg, args, CONF_ADDRESS
    )


@automation.register_action(
    "nrf24.set_channel",
    NRF24SetChannelAction,
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(NRF24Component),
            cv.Required(CONF_CHANNEL): cv.templatable(cv.int_range(min=0, max=125)),
        },
        key=CONF_CHANNEL,
    ),
    synchronous=True,
)
async def nrf24_set_channel_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    # Must go through cg.templatable: TemplatableFn stores a function pointer only.
    templ = await cg.templatable(config[CONF_CHANNEL], args, cg.uint8)
    cg.add(var.set_channel(templ))
    return var


@automation.register_action(
    "nrf24.start_listening",
    NRF24StartListeningAction,
    automation.maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(NRF24Component),
        }
    ),
    synchronous=True,
)
async def nrf24_start_listening_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "nrf24.stop_listening",
    NRF24StopListeningAction,
    automation.maybe_simple_id(
        {
            cv.GenerateID(): cv.use_id(NRF24Component),
        }
    ),
    synchronous=True,
)
async def nrf24_stop_listening_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
