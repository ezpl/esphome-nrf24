# esphome-nrf24

A generic nRF24L01+ radio driver for ESPHome, as an external component.

It knows nothing about any application protocol. It moves raw byte arrays of
arbitrary length and hands received payloads to automations as
`std::vector<uint8_t>`. Build your protocol on top of it.

Targets **ESPHome 2026.7.4** or newer. Fork of
[loucks1/nrf24_espidf](https://github.com/loucks1/nrf24_espidf), whose driver
core runs on ESPHome's real `spi:` bus rather than Arduino's global `SPI`
object.

## Why this and not the usual nRF24 component

- **Payloads are byte vectors, never strings.** A `std::string` built from a C
  string truncates at the first `0x00`. Real payloads contain `0x00`.
- **Sends exactly the bytes you give it.** No padding to 32 bytes.
- **The main loop never blocks.** `RF24::write()` waits up to ~22.5 ms for 15
  retransmits; ESPHome starts warning past ~20 ms. Here `send()` queues and
  `loop()` drains the queue through a state machine.
- **It fails closed.** A radio that does not answer is marked failed and logged
  once. There is no `while (1) {}` and no untimed wait anywhere in the driver.
- **The SPI bus is ESPHome's.** Chip select comes from `spi_device_schema`, so
  the radio shares a bus with displays and SD cards the normal way. A component
  that declares its own `spi:` block and then calls Arduino's `SPI.begin()` is
  ignoring both.

## Wiring

The nRF24L01+ is a **3.3 V** part. Its inputs are 5 V tolerant, but `VCC` is
not — 5 V will destroy it.

| nRF24L01+ | Direction | ESP           | Notes                                                        |
| --------- | --------- | ------------- | ------------------------------------------------------------ |
| `GND`     |           | `GND`         | Pin 1, the corner nearest the notch on the 2×4 header         |
| `VCC`     |           | `3V3`         | 3.3 V only. Put 10 µF (or more) directly across VCC/GND       |
| `CE`      | in        | any GPIO      | `ce_pin:` — switches between standby and TX/RX                |
| `CSN`     | in        | any GPIO      | `cs_pin:` — SPI chip select, handled by `spi_device_schema`   |
| `SCK`     | in        | SPI `clk_pin` | from the `spi:` bus                                           |
| `MOSI`    | in        | SPI `mosi_pin`| from the `spi:` bus                                           |
| `MISO`    | out       | SPI `miso_pin`| from the `spi:` bus                                           |
| `IRQ`     | out       | any GPIO      | `irq_pin:`, optional. Active low, open drain                  |

`CSN` is **not** listed in the `spi:` block — it belongs to the `nrf24:` device,
because one bus can carry several chip selects.

The decoupling capacitor is not optional advice. A PA+LNA module at
`pa_level: MAX` pulls current spikes that brown out a bare module, and the
symptom looks exactly like bad wiring: the chip answers on SPI but never
receives anything.

## Installation

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/ezpl/esphome-nrf24
      ref: main
    components: [nrf24]
```

## Configuration

```yaml
spi:
  - id: radio_spi
    clk_pin: GPIO4
    mosi_pin: GPIO6
    miso_pin: GPIO5

nrf24:
  id: radio
  spi_id: radio_spi
  cs_pin: GPIO7
  ce_pin: GPIO10
  irq_pin: GPIO3
  channel: 76
  rf_data_rate: 1MBPS
  pa_level: MAX
  on_packet:
    - lambda: ESP_LOGD("radio", "%u bytes", x.size());
```

| Option              | Default    | Meaning                                                        |
| ------------------- | ---------- | -------------------------------------------------------------- |
| `ce_pin`            | *required* | CE                                                             |
| `cs_pin`            | *required* | CSN, from `spi_device_schema`                                  |
| `spi_id`            | inferred   | which `spi:` bus                                               |
| `data_rate`         | `1MHz`     | **SPI clock**, not the RF bitrate. The chip takes up to 10 MHz |
| `spi_mode`          | `MODE0`    | from `spi_device_schema`. The nRF24 needs mode 0               |
| `irq_pin`           | –          | optional, active low. Only skips the idle RX poll              |
| `channel`           | `76`       | 0–125, so 2400–2525 MHz                                        |
| `rf_data_rate`      | `1MBPS`    | `250KBPS`, `1MBPS`, `2MBPS`                                    |
| `pa_level`          | `MAX`      | `MIN`, `LOW`, `HIGH`, `MAX`                                    |
| `crc`               | `"16"`     | `"16"`, `"8"`, `DISABLED`. Quote the numbers                   |
| `address_width`     | `5`        | 2–5 bytes. 2 is for promiscuous reception                      |
| `auto_ack`          | `true`     | Enhanced ShockBurst on all pipes                               |
| `payload_size`      | `32`       | fixed RX width, 1–32. TX length is per packet                  |
| `dynamic_payloads`  | `false`    | use the chip's dynamic payload length instead                  |
| `tx_address`        | `E7:…`     | 2–5 bytes, LSByte first                                        |
| `listen`            | `false`    | enter RX as soon as setup finishes                             |
| `retries.delay`     | `5`        | ARD, 0–15, in 250 µs steps: 5 → 1500 µs                        |
| `retries.count`     | `15`       | ARC, 0–15 retransmits                                          |
| `on_packet`         | –          | automation, see below                                          |

`data_rate` is the SPI clock because `spi_device_schema` already claims that
key. The radio's over-the-air bitrate is `rf_data_rate`.

### Address byte order

Addresses are written to the chip in the order given, so **`address[0]` is the
LSByte** — the same convention RF24's byte-array API uses. Pipes 2–5 store only
the LSByte and inherit the rest from pipe 1.

## Triggers

### `on_packet`

Fires once per received payload. The variable `x` is a
`std::vector<uint8_t>` carrying the real length, so a `0x00` inside the payload
is just a byte.

```yaml
on_packet:
  - lambda: |-
      char hex[format_hex_pretty_size(32)];
      ESP_LOGI("radio", "%u bytes: %s", x.size(), format_hex_pretty_to(hex, x));
```

## Actions

### `nrf24.send`

Queues one packet. Returns immediately; the queue holds 8 packets and drains
across `loop()` iterations. `data` accepts a byte list, a quoted string, or a
lambda returning `std::vector<uint8_t>`.

```yaml
- nrf24.send: [0x01, 0x02, 0x03]      # shorthand
- nrf24.send:
    id: radio
    data: "hello"
- nrf24.send:
    id: radio
    data: !lambda return {0x01, 0x00, 0xFF};
- nrf24.send:                          # this packet only, no lasting change
    id: radio
    data: [0xAA, 0xBB]
    channel: 42
    address: [0x01, 0x02, 0x03, 0x04, 0x05]
```

`channel` and `address` are optional and templatable; omit them and the radio's
current settings are used. A literal list is placed in flash, not copied into
RAM. Length must be 1–32 for `data`, 2–5 for `address`.

**Queue entries are self-contained.** Each one carries its own payload, channel
and destination address, captured when it was queued — so changing the channel
or TX address afterwards never retargets a frame that is already in the queue.

### `nrf24.set_channel`

```yaml
- nrf24.set_channel: 2
- nrf24.set_channel:
    id: radio
    channel: !lambda return id(hop_index) % 126;
```

### `nrf24.set_tx_address`

Takes 2–5 bytes, LSByte first. Changeable between consecutive packets.

```yaml
- nrf24.set_tx_address: [0x01, 0x02, 0x03, 0x04, 0x05]
```

### `nrf24.start_listening` / `nrf24.stop_listening`

```yaml
- nrf24.stop_listening:
- nrf24.start_listening:
```

## Using it from C++

Everything a downstream component needs is public. Grab the instance with
`id(radio)` and call it:

```cpp
// transmit — never blocks, never pads
bool send(const uint8_t *buf, uint8_t len, uint8_t channel, const uint8_t *addr, uint8_t addr_len);
bool send(const uint8_t *buf, uint8_t len);   // snapshots the current channel + address
bool send(const std::vector<uint8_t> &data);
uint8_t tx_queue_free();
bool is_transmitting();

// receive
template<typename F> void add_on_packet_callback(F &&callback);  // void(std::vector<uint8_t>)
bool available(uint8_t *pipe_num = nullptr);
void read_payload(uint8_t *buf, uint8_t len);

// mode
void start_listening();
void stop_listening();
bool is_listening();

// addressing, all runtime
void set_tx_address(const uint8_t *addr, uint8_t len);
const uint8_t *get_tx_address();
uint8_t get_tx_address_len();
void open_reading_pipe(uint8_t pipe, const uint8_t *addr, uint8_t len);
void close_reading_pipe(uint8_t pipe);
void set_address_width(uint8_t width);   // 2..5

// radio config, all runtime
void set_channel(uint8_t channel);
void set_rf_data_rate(rf24_datarate_e rate);
void set_pa_level(rf24_pa_dbm_e level, bool lna_enable = true);
void set_crc_length(rf24_crclength_e length);
void disable_crc();
void set_auto_ack(bool enable);
void set_auto_ack(uint8_t pipe, bool enable);
void set_retries(uint8_t delay_steps, uint8_t count);
void set_payload_size(uint8_t size);
void set_dynamic_payloads(bool enable);

// FIFOs and low level
void flush_tx();
void flush_rx();
void power_up();
void power_down();
uint8_t read_register(uint8_t reg);
void write_register(uint8_t reg, uint8_t value);
uint8_t read_status();
bool is_chip_connected();
void ce(bool level);
bool test_rpd();
uint32_t get_tx_ok_count();
uint32_t get_tx_fail_count();
```

Every setter writes to the chip immediately once `setup()` has run, and also
records the value so a recovered link can be reconfigured without restarting.

To prove the chip is present from a lambda:

```yaml
- lambda: ESP_LOGI("radio", "SETUP_AW = 0x%02X", id(radio)->read_register(0x03));
```

## Promiscuous reception

Sniffing a foreign protocol needs CRC off (their CRC is not yours), auto-ACK
off, and a 2-byte address so the preamble itself acts as the address:

```yaml
- nrf24.stop_listening:
- lambda: |-
    id(radio)->disable_crc();
    id(radio)->set_auto_ack(false);
    id(radio)->set_address_width(2);
    static const uint8_t PREAMBLE[2] = {0x55, 0x00};
    id(radio)->open_reading_pipe(1, PREAMBLE, 2);
- nrf24.start_listening:
```

The datasheet calls `SETUP_AW = 0b00` illegal. The chip honours it anyway, and
this component lets you ask for it.

## How TX works

`send()` copies the payload, the channel and the destination address into a
ring buffer of 8 slots and returns. It never touches the SPI bus, and it never
allocates.

Because each slot is self-contained, this is safe:

```cpp
radio->set_tx_address(cabinet_a, 5);  radio->send(f1, n1);  radio->send(f2, n2);
radio->set_tx_address(cabinet_b, 5);  radio->send(f3, n3);  radio->send(f4, n4);
```

`f1` and `f2` still go to cabinet A even though the queue had not drained when
the address changed. Nothing here depends on drain timing.

Each `loop()`:

- **Idle with something queued** — leaves RX if needed, applies that packet's
  channel and address (skipping the SPI writes when the chip already matches,
  so the single-target case costs nothing), writes the payload with
  `W_TX_PAYLOAD` (exactly `len` bytes), pulses CE high for 15 µs, records the
  start time, and returns. About 150 µs.
- **Packet in flight** — one single-byte NOP transfer to read `STATUS`. On
  `TX_DS` the packet succeeded; on `MAX_RT` it failed, and the TX FIFO is
  flushed because `MAX_RT` otherwise blocks every later transmission. Past
  100 ms the packet is dropped with a warning. A few microseconds per loop.
- **Queue empty** — retunes to the configured channel and returns to RX if
  `listen` was on.

Nothing waits on the radio. `get_tx_ok_count()` and `get_tx_fail_count()` tell
you how it is going.

## When the radio does not answer

`setup()` writes two patterns to a scratch register and reads them back. If
either fails, the component logs one error, calls `mark_failed()`, and stops —
`loop()` is never called again, and nothing hangs. `dump_config()` says so.

A watchdog repeats that probe every 30 seconds while idle. If the chip goes
missing it raises a warning once; when it comes back the stored configuration is
rewritten and the warning clears.

## Licence

The register map in `nRF24L01.h` is MIT, © 2007 Stefan Engelke, portions
© 2011 Greg Copeland. The rest follows the upstream project's licence.
