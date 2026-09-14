#pragma once
#include <stdint.h>
#include <stddef.h>

// Project-defined words, NOT manufacturer-assigned ezHMI register addresses.
namespace display_draft {
constexpr uint8_t PACKET_TYPE = 0x30;
constexpr uint32_t CAPABILITY = 1UL << 10;
constexpr size_t WORDS = 16;
constexpr uint32_t HOST_TIMEOUT_MS = 600;
enum Index { VERSION, HEARTBEAT, VALID, SPEED, HANDS, CROSSWALK,
  SECONDS, DISTANCE, PITCH, TOF, HAZARD, WALKER, BRAKING, FAULTS,
  HOST_LINK, RESERVED };
enum Valid { SPEED_VALID=1, HANDS_VALID=2, CROSS_VALID=4,
  PITCH_VALID=8, TOF_VALID=16, WALKER_VALID=32 };

class State {
 public:
  uint16_t words[WORDS] = {1,0,0,65535,0,0,65535,65535,0,5,0,0,0,0,0,0};
  bool accept(const uint8_t* p, size_t n, uint16_t seq, uint32_t now) {
    if (n != WORDS*2 || read(p) != 1 || (read(p+4) & ~63U) ||
        read(p+8)>3 || read(p+10)>6 || read(p+18)>5 ||
        read(p+20)>1 || read(p+22)>5 || read(p+24)>1 ||
        read(p+28)>1 || read(p+30)!=0) return false;
    const uint16_t delta = static_cast<uint16_t>(seq-last_seq_);
    if (seen_ && (delta==0 || delta>=0x8000)) return false;
    for (size_t i=0; i<WORDS; ++i) words[i]=read(p+2*i);
    // Never clear an already reported confirmed hazard on invalid ToF data.
    if (words[HAZARD] || ((words[VALID]&TOF_VALID) &&
        (words[TOF]==3 || words[TOF]==4))) latched_hazard_=true;
    else if ((words[VALID]&TOF_VALID) && words[TOF]==0) latched_hazard_=false;
    words[HAZARD]=latched_hazard_ ? 1 : 0;
    last_seq_=seq; last_rx_=now; seen_=true; words[HOST_LINK]=1;
    return true;
  }
  void newSession() { seen_=false; invalidate(); }
  void tick(uint32_t now) {
    if (!seen_ || static_cast<uint32_t>(now-last_rx_)>=HOST_TIMEOUT_MS) invalidate();
  }
 private:
  bool seen_=false, latched_hazard_=false;
  uint16_t last_seq_=0;
  uint32_t last_rx_=0;
  static uint16_t read(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | static_cast<uint16_t>(p[1])<<8);
  }
  void invalidate() {
    words[VALID]=0; words[SPEED]=65535; words[HANDS]=0;
    words[CROSSWALK]=0; words[SECONDS]=65535; words[DISTANCE]=65535;
    words[PITCH]=0; words[TOF]=5; words[HOST_LINK]=0;
    words[WALKER]=0; words[BRAKING]=0; words[FAULTS]=0;
    words[HAZARD]=latched_hazard_ ? 1 : 0;
  }
};
inline uint16_t modbusCrc(const uint8_t* p, size_t n) {
  uint16_t crc=0xffff;
  while(n--) { crc^=*p++; for(uint8_t i=0;i<8;i++) crc=(crc&1)?(crc>>1)^0xa001:crc>>1; }
  return crc;
}
inline size_t buildWrite(uint8_t* out, uint8_t slave, uint16_t base, const uint16_t* words) {
  out[0]=slave; out[1]=0x10; out[2]=base>>8; out[3]=base&255;
  out[4]=0; out[5]=WORDS; out[6]=WORDS*2;
  for(size_t i=0;i<WORDS;i++) { out[7+i*2]=words[i]>>8; out[8+i*2]=words[i]&255; }
  const size_t n=7+WORDS*2;
  const uint16_t crc=modbusCrc(out,n); out[n]=crc&255; out[n+1]=crc>>8;
  return n+2;
}
}
