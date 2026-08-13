#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include "nrf24.h"

#include <cstddef>
#include <vector>

namespace esphome::nrf24 {

/** Byte-array payload for an action: either a pointer into flash or a stateless
 *  lambda returning a vector. Same tagged union uart.write uses, so a literal
 *  list costs no RAM and a !lambda still works.
 */
template<typename... Ts> class ByteArrayAction : public Action<Ts...>, public Parented<NRF24Component> {
 public:
  void set_data_template(std::vector<uint8_t> (*func)(Ts...)) {
    this->code_.func = func;
    this->len_ = -1;  // sentinel: template mode
  }

  void set_data_static(const uint8_t *data, size_t len) {
    this->code_.data = data;
    this->len_ = static_cast<int16_t>(len);
  }

 protected:
  int16_t len_{-1};  // -1 = template mode, >= 0 = static mode
  union Code {
    std::vector<uint8_t> (*func)(Ts...);
    const uint8_t *data;
  } code_;
};

/// nrf24.send — queues a packet of exactly the bytes given. Never pads.
template<typename... Ts> class NRF24SendAction final : public ByteArrayAction<Ts...> {
 public:
  void play(const Ts &...x) override {
    if (this->len_ >= 0) {
      this->parent_->send(this->code_.data, static_cast<uint8_t>(this->len_));
    } else {
      this->parent_->send(this->code_.func(x...));
    }
  }
};

/// nrf24.set_tx_address — changeable between consecutive packets.
template<typename... Ts> class NRF24SetTxAddressAction final : public ByteArrayAction<Ts...> {
 public:
  void play(const Ts &...x) override {
    if (this->len_ >= 0) {
      this->parent_->set_tx_address(this->code_.data, static_cast<uint8_t>(this->len_));
    } else {
      this->parent_->set_tx_address(this->code_.func(x...));
    }
  }
};

/// nrf24.set_channel — writes RF_CH immediately.
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
