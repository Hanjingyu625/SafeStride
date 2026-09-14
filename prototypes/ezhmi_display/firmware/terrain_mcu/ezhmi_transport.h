#pragma once
#include <Arduino.h>
#include "hmi_state.h"

// Set to 1 ONLY after vendor confirms M-LE Modbus slave/FC16, register map,
// 19200 8N1, ACK format, and LCD-local boot/watchdog event implementation.
#ifndef EZHMI_MODBUS_MAP_CONFIRMED
#define EZHMI_MODBUS_MAP_CONFIRMED 0
#endif
#ifndef EZHMI_SLAVE_ID
#define EZHMI_SLAVE_ID 1
#endif
#ifndef EZHMI_REGISTER_BASE
#define EZHMI_REGISTER_BASE 0x0000
#endif

#if EZHMI_MODBUS_MAP_CONFIRMED
#include <AltSoftSerial.h>
// Uno ATmega328P only: RX=D8, TX=D9. Timer1 and PWM D10 unavailable.
static_assert(EZHMI_SLAVE_ID>0 && EZHMI_SLAVE_ID<248, "No broadcast writes");
static_assert(EZHMI_REGISTER_BASE<=65520, "Register block exceeds 16-bit address space");
#endif

class EzhmiTransport {
 public:
  void begin(uint32_t now) {
    started_=now;
#if EZHMI_MODBUS_MAP_CONFIRMED
    uart_.begin(19200);
#endif
  }
  void tick(uint32_t now, const display_draft::State& state) {
#if EZHMI_MODBUS_MAP_CONFIRMED
    // Fixed buffers, one request in flight; no delay(), no dynamic String.
    uint8_t budget=32;
    while (budget-- && uart_.available()>0) {
      const int b=uart_.read();
      if (b<0) break;
      if (static_cast<uint32_t>(now-last_byte_)>5) rx_n_=0;
      last_byte_=now;
      if (waiting_ && rx_n_<sizeof(rx_)) rx_[rx_n_++]=static_cast<uint8_t>(b);
      else rx_n_=0;
    }
    if (waiting_ && rx_n_ && static_cast<uint32_t>(now-last_byte_)>=5) {
      const bool crc_ok=rx_n_>=5 && display_draft::modbusCrc(rx_,rx_n_)==0;
      if (crc_ok && rx_[0]==EZHMI_SLAVE_ID) {
        if (rx_n_==8 && rx_[1]==0x10 && rx_[2]==(EZHMI_REGISTER_BASE>>8) &&
            rx_[3]==(EZHMI_REGISTER_BASE&255) && rx_[4]==0 && rx_[5]==display_draft::WORDS) {
          last_ack_=now; ack_seen_=true; waiting_=false; ++ack_count;
        } else if(rx_n_==5 && rx_[1]==0x90) {
          exception_code=rx_[2]; waiting_=false; ++error_count;
        }
      }
      rx_n_=0;
    }
    if(waiting_ && static_cast<uint32_t>(now-last_tx_)>=150) {
      waiting_=false; ++error_count;
    }
    // LCD boot is about 2 s per product sheet; allow a 3 s settling window.
    // Later LCD-only power cycles recover through periodic full snapshots.
    if(static_cast<uint32_t>(now-started_)<3000 || waiting_ ||
       static_cast<uint32_t>(now-last_tx_)<200 ||
       static_cast<uint32_t>(now-last_byte_)<5) return;
    uint8_t frame[41];
    const size_t n=display_draft::buildWrite(frame,EZHMI_SLAVE_ID,EZHMI_REGISTER_BASE,state.words);
    // 41 bytes fit the unmodified AltSoftSerial TX ring (68 bytes).
    // The previous request has drained by the 150/200 ms transaction limits.
    uart_.write(frame,n); last_tx_=now; waiting_=true; rx_n_=0;
#else
    (void)now; (void)state; // Deliberately emit no guessed manufacturer commands.
#endif
  }
  bool linkOk(uint32_t now) const { return ack_seen_ && static_cast<uint32_t>(now-last_ack_)<1000; }
  uint32_t ack_count=0, error_count=0;
  uint8_t exception_code=0;
 private:
  uint32_t started_=0, last_tx_=0, last_ack_=0, last_byte_=0;
  bool waiting_=false, ack_seen_=false;
#if EZHMI_MODBUS_MAP_CONFIRMED
  AltSoftSerial uart_;
  uint8_t rx_[16]={0};
  uint8_t rx_n_=0;
#endif
};
