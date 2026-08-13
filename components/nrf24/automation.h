#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include "nrf24.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome::nrf24 {

/** A byte array coming from YAML: unset, a pointer into flash, or a stateless
 *  lambda. Same tagged union uart.write uses, held as a member so one action
 *  can carry several of them.
 */
template<typename... Ts> class ByteArrayValue {
 public:
  void set_template(std::vector<uint8_t> (*func)(Ts...)) {
    this->code_.func = func;
    this->len_ = TEMPLATE_;
  }

  /// Points at a static array in flash — no RAM copy.
  void set_static(const uint8_t *data, size_t len) {
    this->code_.data = data;
    this->len_ = static_cast<int16_t>(len);
  }

  bool has_value() const { return this->len_ != UNSET_; }
  bool is_static() const { return this->len_ >= 0; }
  const uint8_t *static_data() const { return this->code_.data; }
  uint8_t static_len() const { return static_cast<uint8_t>(this->len_); }
  std::vector<uint8_t> call(const Ts &...x) const { return this->code_.func(x...); }

 protected:
  static constexpr int16_t UNSET_ = -2;
  static constexpr int16_t TEMPLATE_ = -1;

  int16_t len_{UNSET_};
  union Code {
    std::vector<uint8_t> (*func)(Ts...);
    const uint8_t *data;
  } code_{nullptr};
};

/** nrf24.send — queues one packet of exactly the bytes given, never padded.
 *  `channel` and `address` are optional; when omitted the radio's current
 *  settings are snapshotted at enqueue time, inside send().
 */
template<typename... Ts> class NRF24SendAction final : public Action<Ts...>, public Parented<NRF24Component> {
 public:
  void set_data_static(const uint8_t *data, size_t len) { this->data_.set_static(data, len); }
  void set_data_template(std::vector<uint8_t> (*func)(Ts...)) { this->data_.set_template(func); }
  void set_address_static(const uint8_t *data, size_t len) { this->address_.set_static(data, len); }
  void set_address_template(std::vector<uint8_t> (*func)(Ts...)) { this->address_.set_template(func); }

  TEMPLATABLE_VALUE(uint8_t, channel)

  void play(const Ts &...x) override {
    if (!this->data_.has_value()) {
      return;
    }
    NRF24Component *radio = this->parent_;

    const uint8_t channel = this->channel_.has_value() ? this->channel_.value(x...) : radio->get_channel();

    const uint8_t *addr = radio->get_tx_address();
    uint8_t addr_len = radio->get_tx_address_len();
    std::vector<uint8_t> addr_buf;  // stays empty, and allocation-free, unless address is a lambda
    if (this->address_.is_static()) {
      addr = this->address_.static_data();
      addr_len = this->address_.static_len();
    } else if (this->address_.has_value()) {
      addr_buf = this->address_.call(x...);
      addr = addr_buf.data();
      addr_len = static_cast<uint8_t>(addr_buf.size());
    }

    if (this->data_.is_static()) {
      radio->send(this->data_.static_data(), this->data_.static_len(), channel, addr, addr_len);
    } else {
      const std::vector<uint8_t> payload = this->data_.call(x...);
      radio->send(payload.data(), static_cast<uint8_t>(payload.size()), channel, addr, addr_len);
    }
  }

 protected:
  ByteArrayValue<Ts...> data_;
  ByteArrayValue<Ts...> address_;
};

/// nrf24.set_tx_address — changes the configured address for later bare sends.
template<typename... Ts> class NRF24SetTxAddressAction final : public Action<Ts...>, public Parented<NRF24Component> {
 public:
  void set_address_static(const uint8_t *data, size_t len) { this->address_.set_static(data, len); }
  void set_address_template(std::vector<uint8_t> (*func)(Ts...)) { this->address_.set_template(func); }

  void play(const Ts &...x) override {
    if (this->address_.is_static()) {
      this->parent_->set_tx_address(this->address_.static_data(), this->address_.static_len());
    } else if (this->address_.has_value()) {
      this->parent_->set_tx_address(this->address_.call(x...));
    }
  }

 protected:
  ByteArrayValue<Ts...> address_;
};

/// nrf24.set_channel — writes RF_CH immediately when no packet is in flight.
template<typename... Ts> class NRF24SetChannelAction final : public Action<Ts...>, public Parented<NRF24Component> {
 public:
  TEMPLATABLE_VALUE(uint8_t, channel)

  void play(const Ts &...x) override { this->parent_->set_channel(this->channel_.value(x...)); }
};

template<typename... Ts>
class NRF24StartListeningAction final : public Action<Ts...>, public Parented<NRF24Component> {
 public:
  void play(const Ts &...x) override { this->parent_->start_listening(); }
};

template<typename... Ts> class NRF24StopListeningAction final : public Action<Ts...>, public Parented<NRF24Component> {
 public:
  void play(const Ts &...x) override { this->parent_->stop_listening(); }
};

}  // namespace esphome::nrf24
