#pragma once

#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"

#include "nRF24L01.h"

#include <cstdint>
#include <vector>

namespace esphome::nrf24 {

using rf24_pa_dbm_e = nRF24L01::rf24_pa_dbm_e;
using rf24_datarate_e = nRF24L01::rf24_datarate_e;
using rf24_crclength_e = nRF24L01::rf24_crclength_e;
using rf24_fifo_state_e = nRF24L01::rf24_fifo_state_e;

/// nRF24L01.h names bit *positions*. Never use those constants as masks.
constexpr uint8_t nbit(uint8_t pos) { return static_cast<uint8_t>(1u << pos); }

/// Largest payload the chip can hold.
static constexpr uint8_t NRF24_MAX_PAYLOAD = 32;
/// Queued TX packets. Each slot costs NRF24_MAX_PAYLOAD + 1 bytes.
static constexpr uint8_t NRF24_TX_QUEUE_DEPTH = 8;

/** Generic nRF24L01+ driver.
 *
 * Knows nothing about any application protocol — it moves raw byte arrays of
 * arbitrary length (1..32) and reports received payloads as std::vector<uint8_t>
 * so embedded 0x00 bytes survive.
 *
 * TX never blocks: send() enqueues and loop() drains the queue through a state
 * machine, one SPI status read per iteration while a packet is in flight.
 */
class NRF24Component : public Component,
                       public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                             spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // ==================== Wiring ====================
  void set_ce_pin(GPIOPin *pin) { this->ce_pin_ = pin; }
  /// Optional. Active low; used only to skip the RX status poll when idle.
  void set_irq_pin(GPIOPin *pin) { this->irq_pin_ = pin; }

  // ==================== Received packets ====================
  /** Register a receiver for complete payloads.
   *  The vector carries the real length, so 0x00 inside a payload is safe. */
  template<typename F> void add_on_packet_callback(F &&callback) {
    this->packet_callback_.add(std::forward<F>(callback));
  }

  // ==================== Transmit ====================
  /** Queue a packet of exactly @p len bytes. Never padded, never blocks.
   *  @return false if the queue is full or the radio failed. */
  bool send(const uint8_t *buf, uint8_t len);
  bool send(const std::vector<uint8_t> &data) { return this->send(data.data(), static_cast<uint8_t>(data.size())); }

  uint8_t tx_queue_size() const { return this->tx_count_; }
  uint8_t tx_queue_free() const { return static_cast<uint8_t>(NRF24_TX_QUEUE_DEPTH - this->tx_count_); }
  /// True while a packet is on the air or waiting for one.
  bool is_transmitting() const { return this->tx_state_ != TX_IDLE || this->tx_count_ != 0; }

  // ==================== Mode ====================
  void start_listening();
  void stop_listening();
  bool is_listening() const { return this->listening_; }

  // ==================== Addressing (all runtime) ====================
  /** Set the destination address. @p addr[0] is the LSByte, matching the
   *  register layout and the RF24 byte-array convention. */
  void set_tx_address(const uint8_t *addr, uint8_t len);
  void set_tx_address(const std::vector<uint8_t> &addr) {
    this->set_tx_address(addr.data(), static_cast<uint8_t>(addr.size()));
  }
  void open_reading_pipe(uint8_t pipe, const uint8_t *addr, uint8_t len);
  void open_reading_pipe(uint8_t pipe, const std::vector<uint8_t> &addr) {
    this->open_reading_pipe(pipe, addr.data(), static_cast<uint8_t>(addr.size()));
  }
  void close_reading_pipe(uint8_t pipe);
  /** 2..5. The datasheet calls SETUP_AW=0 illegal; the chip nevertheless
   *  accepts 2-byte addresses, which is what promiscuous reception needs. */
  void set_address_width(uint8_t width);
  uint8_t get_address_width() const { return this->addr_width_; }

  // ==================== Radio configuration (all runtime) ====================
  void set_channel(uint8_t channel);
  uint8_t get_channel() const { return this->channel_; }
  /// Named set_rf_data_rate, not set_data_rate: SPIDevice already owns that
  /// name for the SPI clock, and hiding it would break register_spi_device.
  void set_rf_data_rate(rf24_datarate_e rate);
  rf24_datarate_e get_rf_data_rate() const { return this->rf_data_rate_; }
  void set_pa_level(rf24_pa_dbm_e level, bool lna_enable = true);
  rf24_pa_dbm_e get_pa_level() const { return this->pa_level_; }
  void set_crc_length(rf24_crclength_e length);
  rf24_crclength_e get_crc_length() const { return this->crc_length_; }
  void disable_crc() { this->set_crc_length(nRF24L01::RF24_CRC_DISABLED); }
  void set_auto_ack(bool enable);
  void set_auto_ack(uint8_t pipe, bool enable);
  /// @param delay_steps ARD in 250 µs steps, 0..15. @param count ARC, 0..15.
  void set_retries(uint8_t delay_steps, uint8_t count);
  void set_payload_size(uint8_t size);
  uint8_t get_payload_size() const { return this->payload_size_; }
  void set_dynamic_payloads(bool enable);
  bool has_dynamic_payloads() const { return this->dynamic_payloads_; }

  // ==================== FIFOs ====================
  void flush_tx();
  void flush_rx();
  /// True when the RX FIFO holds a payload. Optionally reports the pipe.
  bool available(uint8_t *pipe_num = nullptr);
  /// Read one payload out of the RX FIFO. Prefer on_packet / the callback.
  void read_payload(uint8_t *buf, uint8_t len);

  // ==================== Power ====================
  void power_up();
  void power_down();

  // ==================== Low level (public so lambdas can probe) ====================
  uint8_t read_register(uint8_t reg);
  void read_register(uint8_t reg, uint8_t *buf, uint8_t len);
  void write_register(uint8_t reg, uint8_t value);
  void write_register(uint8_t reg, const uint8_t *buf, uint8_t len);
  /// Single NOP transfer: returns STATUS without a second byte on the bus.
  uint8_t read_status();
  /// Verifies the chip answers by writing two patterns and reading them back.
  bool is_chip_connected();
  /// Raise or lower CE. No settling delay — callers add what they need.
  void ce(bool level);
  bool test_rpd() { return (this->read_register(nRF24L01::RPD) & 0x01) != 0; }

  /// Successful / failed transmissions since boot.
  uint32_t get_tx_ok_count() const { return this->tx_ok_count_; }
  uint32_t get_tx_fail_count() const { return this->tx_fail_count_; }

 protected:
  enum TxState : uint8_t {
    TX_IDLE = 0,
    TX_IN_FLIGHT,
  };

  struct TxPacket {
    uint8_t len;
    uint8_t data[NRF24_MAX_PAYLOAD];
  };

  // --- SPI plumbing ---
  void begin_transaction_() { this->enable(); }
  void end_transaction_() { this->disable(); }
  void send_command_(uint8_t command);

  // --- setup / recovery ---
  uint8_t build_config_(bool prim_rx, bool powered = true) const;
  void write_config_(bool prim_rx, bool powered = true);
  void apply_config_();
  void enter_rx_();
  void leave_rx_();

  // --- loop halves ---
  void process_tx_();
  void process_rx_();
  void start_tx_(const TxPacket &packet);
  void finish_tx_(bool success);

  uint8_t dynamic_payload_size_();

  GPIOPin *ce_pin_{nullptr};
  GPIOPin *irq_pin_{nullptr};

  CallbackManager<void(std::vector<uint8_t>)> packet_callback_;

  // --- shadow of everything we ever wrote, so recovery can restore it ---
  uint8_t channel_{76};
  uint8_t payload_size_{NRF24_MAX_PAYLOAD};
  uint8_t addr_width_{5};
  uint8_t retry_delay_{5};
  uint8_t retry_count_{15};
  bool dynamic_payloads_{false};
  rf24_datarate_e rf_data_rate_{nRF24L01::RF24_1MBPS};
  rf24_pa_dbm_e pa_level_{nRF24L01::RF24_PA_MAX};
  rf24_crclength_e crc_length_{nRF24L01::RF24_CRC_16};
  bool lna_enable_{true};

  uint8_t tx_address_[5]{0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  uint8_t tx_address_len_{5};
  uint8_t pipe0_address_[5]{0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  uint8_t pipe0_address_len_{0};  ///< 0 = pipe 0 never opened for RX
  uint8_t pipe1_address_[5]{0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
  uint8_t pipe1_address_len_{0};
  uint8_t pipe_lsb_[6]{};   ///< LSByte for pipes 2..5
  uint8_t en_rxaddr_{0x00};  ///< mirror of EN_RXADDR
  uint8_t en_aa_{0x3F};      ///< mirror of EN_AA

  /// Whether the user wants RX; the chip may be out of RX during a TX burst.
  bool listening_{false};
  /// What PRIM_RX is actually set to right now.
  bool chip_rx_{false};
  /// setup() finished and the chip answered; before that setters only store.
  bool ready_{false};

  TxPacket tx_queue_[NRF24_TX_QUEUE_DEPTH];
  uint8_t tx_head_{0};
  uint8_t tx_tail_{0};
  uint8_t tx_count_{0};
  TxState tx_state_{TX_IDLE};
  uint32_t tx_started_ms_{0};
  uint32_t tx_ok_count_{0};
  uint32_t tx_fail_count_{0};
  bool tx_queue_full_logged_{false};

  uint32_t last_rx_poll_ms_{0};
  uint32_t last_watchdog_ms_{0};
  /// Set while the watchdog reports the chip missing, so it logs once per outage.
  bool link_warned_{false};
};

}  // namespace esphome::nrf24
