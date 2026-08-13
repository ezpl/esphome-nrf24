#include "nrf24.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cstring>

namespace esphome::nrf24 {

static const char *const TAG = "nrf24";

/// Worst case for 15 retransmits at ARD=4000 µs is ~60 ms; be generous, then give up.
static constexpr uint32_t TX_TIMEOUT_MS = 100;
/// Even with an IRQ pin wired, poll this often so a bad IRQ wire cannot mute RX.
static constexpr uint32_t RX_FALLBACK_POLL_MS = 250;
static constexpr uint32_t WATCHDOG_INTERVAL_MS = 30000;

// ====================== Component ======================

void NRF24Component::setup() {
  if (this->ce_pin_ != nullptr) {
    this->ce_pin_->setup();
    this->ce_pin_->digital_write(false);
  }
  if (this->irq_pin_ != nullptr) {
    this->irq_pin_->setup();
  }

  this->spi_setup();
  delay(5);  // the chip needs 100 ms from power-on; ESP boot has covered most of it

  if (!this->is_chip_connected()) {
    ESP_LOGE(TAG, "nRF24L01+ did not answer on SPI. Check CSN/SCK/MOSI/MISO wiring and the 3.3 V supply");
    this->mark_failed();
    return;
  }

  this->apply_config_();
  this->ready_ = true;
  ESP_LOGI(TAG, "nRF24L01+ ready on channel %u", this->channel_);
}

void NRF24Component::loop() {
  this->process_tx_();
  this->process_rx_();

  const uint32_t now = millis();
  if (now - this->last_watchdog_ms_ < WATCHDOG_INTERVAL_MS) {
    return;
  }
  this->last_watchdog_ms_ = now;
  if (this->tx_state_ != TX_IDLE) {
    return;  // never poke the scratch register while a packet is on the air
  }

  if (this->is_chip_connected()) {
    if (this->link_warned_) {
      ESP_LOGI(TAG, "nRF24L01+ responding again, restoring configuration");
      this->apply_config_();
      this->link_warned_ = false;
      this->status_clear_warning();
    }
  } else if (!this->link_warned_) {
    ESP_LOGW(TAG, "nRF24L01+ stopped answering on SPI");
    this->link_warned_ = true;
    this->status_set_warning();
  }
}

void NRF24Component::dump_config() {
  ESP_LOGCONFIG(TAG, "nRF24L01+:");
  LOG_PIN("  CE Pin: ", this->ce_pin_);
  LOG_PIN("  IRQ Pin: ", this->irq_pin_);
  LOG_SPI_DEVICE(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Radio did not respond - component disabled");
    return;
  }

  static const char *const DATA_RATES[] = {"1Mbps", "2Mbps", "250kbps"};
  static const char *const PA_LEVELS[] = {"MIN (-18 dBm)", "LOW (-12 dBm)", "HIGH (-6 dBm)", "MAX (0 dBm)"};
  static const char *const CRC_LENGTHS[] = {"disabled", "8 bit", "16 bit"};

  ESP_LOGCONFIG(TAG, "  Channel: %u (%d MHz)", this->channel_, 2400 + this->channel_);
  ESP_LOGCONFIG(TAG, "  RF data rate: %s", DATA_RATES[static_cast<uint8_t>(this->rf_data_rate_) % 3]);
  ESP_LOGCONFIG(TAG, "  PA level: %s", PA_LEVELS[static_cast<uint8_t>(this->pa_level_) % 4]);
  ESP_LOGCONFIG(TAG, "  CRC: %s", CRC_LENGTHS[static_cast<uint8_t>(this->crc_length_) % 3]);
  ESP_LOGCONFIG(TAG, "  Address width: %u bytes", this->addr_width_);
  ESP_LOGCONFIG(TAG, "  Auto ACK (EN_AA): 0x%02X", this->en_aa_);
  ESP_LOGCONFIG(TAG, "  Retries: %u x %d us", this->retry_count_, (this->retry_delay_ + 1) * 250);
  if (this->dynamic_payloads_) {
    ESP_LOGCONFIG(TAG, "  Payload: dynamic");
  } else {
    ESP_LOGCONFIG(TAG, "  Payload: %u bytes fixed on RX (TX length is per packet)", this->payload_size_);
  }
  char hex[format_hex_pretty_size(5)];
  ESP_LOGCONFIG(TAG, "  TX address (LSByte first): %s",
                format_hex_pretty_to(hex, this->tx_address_, this->tx_address_len_));
  ESP_LOGCONFIG(TAG, "  Enabled RX pipes (EN_RXADDR): 0x%02X", this->en_rxaddr_);
  ESP_LOGCONFIG(TAG, "  Listening: %s", YESNO(this->listening_));
  ESP_LOGCONFIG(TAG, "  TX queue depth: %u", NRF24_TX_QUEUE_DEPTH);

  // Read back what the chip actually holds — proof it is alive, not an echo of our shadow.
  ESP_LOGCONFIG(TAG, "  Chip registers: CONFIG=0x%02X EN_AA=0x%02X EN_RXADDR=0x%02X SETUP_AW=0x%02X",
                this->read_register(nRF24L01::CONFIG), this->read_register(nRF24L01::EN_AA),
                this->read_register(nRF24L01::EN_RXADDR), this->read_register(nRF24L01::SETUP_AW));
  ESP_LOGCONFIG(TAG, "                  SETUP_RETR=0x%02X RF_CH=0x%02X RF_SETUP=0x%02X FEATURE=0x%02X",
                this->read_register(nRF24L01::SETUP_RETR), this->read_register(nRF24L01::RF_CH),
                this->read_register(nRF24L01::RF_SETUP), this->read_register(nRF24L01::FEATURE));

  if (this->read_register(nRF24L01::RF_CH) != this->channel_) {
    ESP_LOGW(TAG, "  RF_CH readback does not match the configured channel - SPI may be unreliable");
  }
}

// ====================== SPI primitives ======================

void NRF24Component::send_command_(uint8_t command) {
  this->begin_transaction_();
  this->transfer_byte(command);
  this->end_transaction_();
}

uint8_t NRF24Component::read_status() {
  this->begin_transaction_();
  const uint8_t status = this->transfer_byte(nRF24L01::NOP);
  this->end_transaction_();
  return status;
}

uint8_t NRF24Component::read_register(uint8_t reg) {
  this->begin_transaction_();
  this->transfer_byte(nRF24L01::R_REGISTER | (reg & nRF24L01::REGISTER_MASK));
  const uint8_t result = this->transfer_byte(nRF24L01::NOP);
  this->end_transaction_();
  return result;
}

void NRF24Component::read_register(uint8_t reg, uint8_t *buf, uint8_t len) {
  this->begin_transaction_();
  this->transfer_byte(nRF24L01::R_REGISTER | (reg & nRF24L01::REGISTER_MASK));
  this->read_array(buf, len);
  this->end_transaction_();
}

void NRF24Component::write_register(uint8_t reg, uint8_t value) {
  this->begin_transaction_();
  this->transfer_byte(nRF24L01::W_REGISTER | (reg & nRF24L01::REGISTER_MASK));
  this->transfer_byte(value);
  this->end_transaction_();
}

void NRF24Component::write_register(uint8_t reg, const uint8_t *buf, uint8_t len) {
  this->begin_transaction_();
  this->transfer_byte(nRF24L01::W_REGISTER | (reg & nRF24L01::REGISTER_MASK));
  this->write_array(buf, len);
  this->end_transaction_();
}

void NRF24Component::ce(bool level) {
  if (this->ce_pin_ != nullptr) {
    this->ce_pin_->digital_write(level);
  }
}

bool NRF24Component::is_chip_connected() {
  // SETUP_RETR is plain R/W and idle between transactions, so it doubles as a
  // scratch register. Two patterns rule out a bus stuck high or low.
  const uint8_t restore = static_cast<uint8_t>((this->retry_delay_ & 0x0F) << 4 | (this->retry_count_ & 0x0F));
  static const uint8_t PATTERNS[2] = {0x5A, 0xA5};
  bool ok = true;
  for (const uint8_t pattern : PATTERNS) {
    this->write_register(nRF24L01::SETUP_RETR, pattern);
    if (this->read_register(nRF24L01::SETUP_RETR) != pattern) {
      ok = false;
      break;
    }
  }
  this->write_register(nRF24L01::SETUP_RETR, restore);
  return ok;
}

// ====================== Configuration ======================

uint8_t NRF24Component::build_config_(bool prim_rx, bool powered) const {
  uint8_t cfg = 0;
  if (powered) {
    cfg |= nbit(nRF24L01::PWR_UP);
  }
  if (this->crc_length_ == nRF24L01::RF24_CRC_16) {
    cfg |= nbit(nRF24L01::EN_CRC) | nbit(nRF24L01::CRCO);
  } else if (this->crc_length_ == nRF24L01::RF24_CRC_8) {
    cfg |= nbit(nRF24L01::EN_CRC);
  }
  if (prim_rx) {
    cfg |= nbit(nRF24L01::PRIM_RX);
  }
  return cfg;  // all three IRQ sources stay unmasked so the IRQ pin works
}

void NRF24Component::write_config_(bool prim_rx, bool powered) {
  this->write_register(nRF24L01::CONFIG, this->build_config_(prim_rx, powered));
  this->chip_rx_ = prim_rx;
}

void NRF24Component::apply_config_() {
  this->ce(false);
  this->write_config_(false);
  delay(5);  // Tpd2stby is 1.5 ms with the internal oscillator

  this->write_register(nRF24L01::SETUP_RETR,
                       static_cast<uint8_t>((this->retry_delay_ & 0x0F) << 4 | (this->retry_count_ & 0x0F)));
  this->write_register(nRF24L01::RF_CH, this->channel_);

  uint8_t rf_setup = static_cast<uint8_t>((this->pa_level_ & 0x03) << 1);
  if (this->lna_enable_) {
    rf_setup |= 0x01;
  }
  if (this->rf_data_rate_ == nRF24L01::RF24_250KBPS) {
    rf_setup |= nbit(nRF24L01::RF_DR_LOW);
  } else if (this->rf_data_rate_ == nRF24L01::RF24_2MBPS) {
    rf_setup |= nbit(nRF24L01::RF_DR_HIGH);
  }
  this->write_register(nRF24L01::RF_SETUP, rf_setup);

  this->write_register(nRF24L01::SETUP_AW, static_cast<uint8_t>(this->addr_width_ - 2));
  this->write_register(nRF24L01::EN_AA, this->en_aa_);

  // FEATURE/DYNPD need the ACTIVATE 0x73 unlock on some clones. Real nRF24L01+
  // parts accept the write directly, so only unlock when the write did not stick.
  const uint8_t feature = this->dynamic_payloads_ ? nbit(nRF24L01::EN_DPL) : 0x00;
  this->write_register(nRF24L01::FEATURE, feature);
  if (this->read_register(nRF24L01::FEATURE) != feature) {
    this->begin_transaction_();
    this->transfer_byte(nRF24L01::ACTIVATE);
    this->transfer_byte(0x73);
    this->end_transaction_();
    this->write_register(nRF24L01::FEATURE, feature);
  }
  this->write_register(nRF24L01::DYNPD, this->dynamic_payloads_ ? 0x3F : 0x00);

  for (uint8_t pipe = 0; pipe < 6; pipe++) {
    this->write_register(static_cast<uint8_t>(nRF24L01::RX_PW_P0 + pipe), this->payload_size_);
  }

  this->write_register(nRF24L01::TX_ADDR, this->tx_address_, this->tx_address_len_);
  if (this->pipe0_address_len_ > 0) {
    this->write_register(nRF24L01::RX_ADDR_P0, this->pipe0_address_, this->pipe0_address_len_);
  } else {
    this->write_register(nRF24L01::RX_ADDR_P0, this->tx_address_, this->tx_address_len_);
  }
  if (this->pipe1_address_len_ > 0) {
    this->write_register(nRF24L01::RX_ADDR_P1, this->pipe1_address_, this->pipe1_address_len_);
  }
  for (uint8_t pipe = 2; pipe < 6; pipe++) {
    this->write_register(static_cast<uint8_t>(nRF24L01::RX_ADDR_P0 + pipe), this->pipe_lsb_[pipe]);
  }
  this->write_register(nRF24L01::EN_RXADDR, this->en_rxaddr_);

  this->write_register(nRF24L01::STATUS, static_cast<uint8_t>(nRF24L01::RF24_IRQ_ALL));
  this->flush_tx();
  this->flush_rx();

  this->tx_state_ = TX_IDLE;
  if (this->listening_) {
    this->enter_rx_();
  }
}

void NRF24Component::set_channel(uint8_t channel) {
  if (channel > 125) {
    channel = 125;
  }
  this->channel_ = channel;
  if (this->ready_) {
    this->write_register(nRF24L01::RF_CH, channel);
  }
}

void NRF24Component::set_rf_data_rate(rf24_datarate_e rate) {
  this->rf_data_rate_ = rate;
  if (!this->ready_) {
    return;
  }
  uint8_t setup = this->read_register(nRF24L01::RF_SETUP) & ~(nbit(nRF24L01::RF_DR_LOW) | nbit(nRF24L01::RF_DR_HIGH));
  if (rate == nRF24L01::RF24_250KBPS) {
    setup |= nbit(nRF24L01::RF_DR_LOW);
  } else if (rate == nRF24L01::RF24_2MBPS) {
    setup |= nbit(nRF24L01::RF_DR_HIGH);
  }
  this->write_register(nRF24L01::RF_SETUP, setup);
  delayMicroseconds(200);  // let the PLL settle before the next command
}

void NRF24Component::set_pa_level(rf24_pa_dbm_e level, bool lna_enable) {
  if (level > nRF24L01::RF24_PA_MAX) {
    level = nRF24L01::RF24_PA_MAX;
  }
  this->pa_level_ = level;
  this->lna_enable_ = lna_enable;
  if (!this->ready_) {
    return;
  }
  uint8_t setup = this->read_register(nRF24L01::RF_SETUP) & 0xF8;
  setup |= static_cast<uint8_t>((level & 0x03) << 1) | (lna_enable ? 0x01 : 0x00);
  this->write_register(nRF24L01::RF_SETUP, setup);
}

void NRF24Component::set_crc_length(rf24_crclength_e length) {
  this->crc_length_ = length;
  if (this->ready_) {
    this->write_config_(this->chip_rx_);
  }
}

void NRF24Component::set_auto_ack(bool enable) {
  this->en_aa_ = enable ? 0x3F : 0x00;
  if (this->ready_) {
    this->write_register(nRF24L01::EN_AA, this->en_aa_);
  }
}

void NRF24Component::set_auto_ack(uint8_t pipe, bool enable) {
  if (pipe > 5) {
    return;
  }
  if (enable) {
    this->en_aa_ |= nbit(pipe);
  } else {
    this->en_aa_ &= static_cast<uint8_t>(~nbit(pipe));
  }
  if (this->ready_) {
    this->write_register(nRF24L01::EN_AA, this->en_aa_);
  }
}

void NRF24Component::set_retries(uint8_t delay_steps, uint8_t count) {
  this->retry_delay_ = delay_steps & 0x0F;
  this->retry_count_ = count & 0x0F;
  if (this->ready_) {
    this->write_register(nRF24L01::SETUP_RETR,
                         static_cast<uint8_t>(this->retry_delay_ << 4 | this->retry_count_));
  }
}

void NRF24Component::set_payload_size(uint8_t size) {
  if (size == 0) {
    size = 1;
  } else if (size > NRF24_MAX_PAYLOAD) {
    size = NRF24_MAX_PAYLOAD;
  }
  this->payload_size_ = size;
  if (!this->ready_) {
    return;
  }
  for (uint8_t pipe = 0; pipe < 6; pipe++) {
    this->write_register(static_cast<uint8_t>(nRF24L01::RX_PW_P0 + pipe), size);
  }
}

void NRF24Component::set_dynamic_payloads(bool enable) {
  this->dynamic_payloads_ = enable;
  if (!this->ready_) {
    return;
  }
  const uint8_t feature = enable ? nbit(nRF24L01::EN_DPL) : 0x00;
  this->write_register(nRF24L01::FEATURE, feature);
  this->write_register(nRF24L01::DYNPD, enable ? 0x3F : 0x00);
}

void NRF24Component::set_address_width(uint8_t width) {
  if (width < 2) {
    width = 2;
  } else if (width > 5) {
    width = 5;
  }
  this->addr_width_ = width;
  if (this->ready_) {
    // The datasheet calls SETUP_AW = 0b00 illegal, but the chip honours it as a
    // 2-byte address, which is how promiscuous reception is done.
    this->write_register(nRF24L01::SETUP_AW, static_cast<uint8_t>(width - 2));
  }
}

// ====================== Addressing ======================

void NRF24Component::set_tx_address(const uint8_t *addr, uint8_t len) {
  if (addr == nullptr || len == 0) {
    return;
  }
  if (len > 5) {
    len = 5;
  }
  memcpy(this->tx_address_, addr, len);
  this->tx_address_len_ = len;
  if (!this->ready_) {
    return;
  }
  this->write_register(nRF24L01::TX_ADDR, this->tx_address_, len);
  // The auto-ACK comes back on pipe 0, so its address has to match while transmitting.
  if ((this->en_aa_ & 0x01) != 0 && !this->chip_rx_) {
    this->write_register(nRF24L01::RX_ADDR_P0, this->tx_address_, len);
  }
}

void NRF24Component::open_reading_pipe(uint8_t pipe, const uint8_t *addr, uint8_t len) {
  if (pipe > 5 || addr == nullptr || len == 0) {
    return;
  }
  if (len > 5) {
    len = 5;
  }

  if (pipe == 0) {
    memcpy(this->pipe0_address_, addr, len);
    this->pipe0_address_len_ = len;
  } else if (pipe == 1) {
    memcpy(this->pipe1_address_, addr, len);
    this->pipe1_address_len_ = len;
  } else {
    // Pipes 2..5 store only the LSByte and inherit the rest from pipe 1.
    this->pipe_lsb_[pipe] = addr[0];
  }
  this->en_rxaddr_ |= nbit(pipe);

  if (!this->ready_) {
    return;
  }
  const uint8_t reg = static_cast<uint8_t>(nRF24L01::RX_ADDR_P0 + pipe);
  if (pipe < 2) {
    this->write_register(reg, addr, len);
  } else {
    this->write_register(reg, addr[0]);
  }
  this->write_register(static_cast<uint8_t>(nRF24L01::RX_PW_P0 + pipe), this->payload_size_);
  this->write_register(nRF24L01::EN_RXADDR, this->en_rxaddr_);
}

void NRF24Component::close_reading_pipe(uint8_t pipe) {
  if (pipe > 5) {
    return;
  }
  this->en_rxaddr_ &= static_cast<uint8_t>(~nbit(pipe));
  if (pipe == 0) {
    this->pipe0_address_len_ = 0;
  } else if (pipe == 1) {
    this->pipe1_address_len_ = 0;
  }
  if (this->ready_) {
    this->write_register(nRF24L01::EN_RXADDR, this->en_rxaddr_);
  }
}

// ====================== Mode ======================

void NRF24Component::start_listening() {
  this->listening_ = true;
  if (this->ready_ && this->tx_state_ == TX_IDLE && this->tx_count_ == 0) {
    this->enter_rx_();
  }
}

void NRF24Component::stop_listening() {
  this->listening_ = false;
  if (this->ready_ && this->chip_rx_) {
    this->leave_rx_();
  }
}

void NRF24Component::enter_rx_() {
  if (this->pipe0_address_len_ > 0) {
    this->write_register(nRF24L01::RX_ADDR_P0, this->pipe0_address_, this->pipe0_address_len_);
  }
  this->write_config_(true);
  this->write_register(nRF24L01::STATUS, static_cast<uint8_t>(nRF24L01::RF24_IRQ_ALL));
  this->flush_rx();
  this->ce(true);
  delayMicroseconds(130);  // Tstby2a
}

void NRF24Component::leave_rx_() {
  this->ce(false);
  delayMicroseconds(130);
  this->write_config_(false);
}

void NRF24Component::power_up() {
  this->write_config_(this->chip_rx_, true);
  delay(5);
}

void NRF24Component::power_down() {
  this->ce(false);
  this->write_config_(this->chip_rx_, false);
}

// ====================== FIFOs ======================

void NRF24Component::flush_tx() { this->send_command_(nRF24L01::FLUSH_TX); }

void NRF24Component::flush_rx() { this->send_command_(nRF24L01::FLUSH_RX); }

bool NRF24Component::available(uint8_t *pipe_num) {
  const uint8_t status = this->read_status();
  const uint8_t pipe = static_cast<uint8_t>((status >> 1) & 0x07);
  if (pipe > 5) {
    // 0b111 means the RX FIFO is empty. A latched RX_DR with an empty FIFO
    // would otherwise keep the IRQ line asserted forever.
    if ((status & nbit(nRF24L01::RX_DR)) != 0) {
      this->write_register(nRF24L01::STATUS, nbit(nRF24L01::RX_DR));
    }
    return false;
  }
  if (pipe_num != nullptr) {
    *pipe_num = pipe;
  }
  return true;
}

void NRF24Component::read_payload(uint8_t *buf, uint8_t len) {
  this->begin_transaction_();
  this->transfer_byte(nRF24L01::R_RX_PAYLOAD);
  this->read_array(buf, len);
  this->end_transaction_();
  this->write_register(nRF24L01::STATUS, nbit(nRF24L01::RX_DR));
}

uint8_t NRF24Component::dynamic_payload_size_() {
  // R_RX_PL_WID is a command, not a register — masking it with REGISTER_MASK
  // (as upstream did) turns it into a read of CONFIG.
  this->begin_transaction_();
  this->transfer_byte(nRF24L01::R_RX_PL_WID);
  const uint8_t width = this->transfer_byte(nRF24L01::NOP);
  this->end_transaction_();
  return width;
}

// ====================== Transmit ======================

bool NRF24Component::send(const uint8_t *buf, uint8_t len) {
  if (this->is_failed() || !this->ready_) {
    return false;
  }
  if (buf == nullptr || len == 0 || len > NRF24_MAX_PAYLOAD) {
    ESP_LOGW(TAG, "Refusing to send %u bytes (must be 1..%u)", len, NRF24_MAX_PAYLOAD);
    return false;
  }
  if (this->tx_count_ >= NRF24_TX_QUEUE_DEPTH) {
    if (!this->tx_queue_full_logged_) {
      ESP_LOGW(TAG, "TX queue full (%u packets), dropping", NRF24_TX_QUEUE_DEPTH);
      this->tx_queue_full_logged_ = true;
    }
    return false;
  }

  TxPacket &slot = this->tx_queue_[this->tx_head_];
  slot.len = len;
  memcpy(slot.data, buf, len);
  this->tx_head_ = static_cast<uint8_t>((this->tx_head_ + 1) % NRF24_TX_QUEUE_DEPTH);
  this->tx_count_++;
  return true;
}

void NRF24Component::process_tx_() {
  if (this->tx_state_ == TX_IN_FLIGHT) {
    const uint8_t status = this->read_status();
    if ((status & (nbit(nRF24L01::TX_DS) | nbit(nRF24L01::MAX_RT))) != 0) {
      this->finish_tx_((status & nbit(nRF24L01::TX_DS)) != 0);
    } else if (millis() - this->tx_started_ms_ > TX_TIMEOUT_MS) {
      ESP_LOGW(TAG, "TX did not complete within %" PRIu32 " ms, dropping packet", TX_TIMEOUT_MS);
      this->finish_tx_(false);
    }
    return;
  }

  if (this->tx_count_ == 0 || !this->ready_) {
    return;
  }
  this->start_tx_(this->tx_queue_[this->tx_tail_]);
}

void NRF24Component::start_tx_(const TxPacket &packet) {
  if (this->chip_rx_) {
    this->leave_rx_();
  }
  if ((this->en_aa_ & 0x01) != 0) {
    this->write_register(nRF24L01::RX_ADDR_P0, this->tx_address_, this->tx_address_len_);
  }
  this->write_register(nRF24L01::STATUS, nbit(nRF24L01::TX_DS) | nbit(nRF24L01::MAX_RT));

  // Exactly packet.len bytes go out. No padding to payload_size_, ever.
  this->begin_transaction_();
  this->transfer_byte(nRF24L01::W_TX_PAYLOAD);
  this->write_array(packet.data, packet.len);
  this->end_transaction_();

  this->ce(true);
  delayMicroseconds(15);  // CE must stay high at least 10 µs to launch the packet
  this->ce(false);

  this->tx_started_ms_ = millis();
  this->tx_state_ = TX_IN_FLIGHT;
}

void NRF24Component::finish_tx_(bool success) {
  this->ce(false);
  this->write_register(nRF24L01::STATUS, nbit(nRF24L01::TX_DS) | nbit(nRF24L01::MAX_RT));
  if (success) {
    this->tx_ok_count_++;
  } else {
    // MAX_RT leaves the payload in the FIFO and blocks every later transmission.
    this->flush_tx();
    this->tx_fail_count_++;
  }

  this->tx_tail_ = static_cast<uint8_t>((this->tx_tail_ + 1) % NRF24_TX_QUEUE_DEPTH);
  this->tx_count_--;
  this->tx_state_ = TX_IDLE;
  this->tx_queue_full_logged_ = false;

  if (this->tx_count_ == 0 && this->listening_) {
    this->enter_rx_();
  }
}

// ====================== Receive ======================

void NRF24Component::process_rx_() {
  if (!this->ready_ || !this->chip_rx_ || this->tx_state_ != TX_IDLE) {
    return;
  }

  const uint32_t now = millis();
  if (this->irq_pin_ != nullptr && this->irq_pin_->digital_read() &&
      now - this->last_rx_poll_ms_ < RX_FALLBACK_POLL_MS) {
    return;  // IRQ is active low; nothing pending and the fallback poll is not due
  }
  this->last_rx_poll_ms_ = now;

  // The RX FIFO holds three payloads; draining more than that per loop would
  // only mean the radio is faster than we are.
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t pipe = 0;
    if (!this->available(&pipe)) {
      return;
    }

    const uint8_t len = this->dynamic_payloads_ ? this->dynamic_payload_size_() : this->payload_size_;
    if (len == 0 || len > NRF24_MAX_PAYLOAD) {
      ESP_LOGW(TAG, "Payload width %u out of range, flushing RX FIFO", len);
      this->flush_rx();
      this->write_register(nRF24L01::STATUS, nbit(nRF24L01::RX_DR));
      return;
    }

    std::vector<uint8_t> payload(len);
    this->read_payload(payload.data(), len);
#ifdef ESPHOME_LOG_HAS_VERBOSE
    char hex[format_hex_pretty_size(NRF24_MAX_PAYLOAD)];
    ESP_LOGV(TAG, "RX pipe %u, %u bytes: %s", pipe, len, format_hex_pretty_to(hex, payload.data(), payload.size()));
#endif
    this->packet_callback_.call(payload);
  }
}

}  // namespace esphome::nrf24
