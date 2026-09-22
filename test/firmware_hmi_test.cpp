#include <assert.h>
#include <string.h>
#include "../firmware/terrain_mcu/ezhmi_transport.h"

namespace hmi = safestride_hmi;

static void pack(const uint16_t* words, uint8_t* out) {
  for (size_t i=0; i<hmi::WORDS; ++i) {
    out[i*2]=words[i]&255; out[i*2+1]=words[i]>>8;
  }
}
static void ack(bool corrupt=false) {
  uint8_t bytes[8]={1,0x10,0,0,0,16,0,0};
  uint16_t crc=hmi::modbusCrc(bytes,6);
  bytes[6]=crc&255; bytes[7]=crc>>8;
  if(corrupt) bytes[6]^=1;
  hmi_uart_test::rx().assign(bytes,bytes+8);
}

static void verifyScenario(const uint16_t* expected) {
  uint8_t payload[32];
  pack(expected,payload);

  hmi::State state;
  assert(state.accept(payload,sizeof(payload),1,100));
  for (size_t i=0; i<hmi::WORDS; ++i) assert(state.words[i]==expected[i]);

  uint8_t frame[41];
  assert(hmi::buildWrite(frame,1,0,state.words)==sizeof(frame));
  assert(frame[0]==1 && frame[1]==0x10);
  assert(frame[2]==0 && frame[3]==0 && frame[4]==0 && frame[5]==16);
  assert(frame[6]==32);
  for (size_t i=0; i<hmi::WORDS; ++i) {
    assert(frame[7+i*2]==(expected[i]>>8));
    assert(frame[8+i*2]==(expected[i]&255));
  }
  assert(hmi::modbusCrc(frame,sizeof(frame))==0);
}

int main() {
  // Same six PC-only operating states covered by test_hmi_scenarios.py.
  const uint16_t scenarios[][hmi::WORDS] = {
    {2,1,63,125,3,3,15,42,0,0,0,2,0,0,1,83},
    {2,1,63,85,3,2,65535,35,85,0,0,2,0,0,1,3},
    {2,1,63,70,3,4,9,21,65466,0,0,2,0,0,1,67},
    {2,1,63,20,3,2,5,15,0,0,0,2,1,0,1,67},
    {2,1,63,0,3,2,65535,15,0,3,1,2,0,0,1,3},
    {2,1,0,65535,0,0,65535,65535,0,5,0,0,0,0,1,0},
  };
  for (size_t i=0; i<sizeof(scenarios)/sizeof(scenarios[0]); ++i)
    verifyScenario(scenarios[i]);

  hmi::State state;
  uint16_t words[16]={2,1,63,125,3,3,6,42,65526,3,1,2,0,0,1,19};
  uint8_t payload[32]; pack(words,payload);
  assert(state.accept(payload,32,65535,100));
  assert(!state.accept(payload,32,65535,101));
  assert(state.accept(payload,32,0,102)); // sequence wraps
  assert(!state.accept(payload,31,1,103));
  payload[0]=1; assert(!state.accept(payload,32,1,103)); payload[0]=2;
  state.tick(702);
  assert(state.words[hmi::VALID]==0 && state.words[hmi::SPEED]==65535);
  assert(state.words[hmi::FLAGS]==0 && state.words[hmi::HAZARD]==1);
  state.newSession();
  assert(state.words[hmi::HAZARD]==1);
  words[hmi::TOF]=0; words[hmi::HAZARD]=0; pack(words,payload);
  assert(state.accept(payload,32,5,703));
  assert(state.words[hmi::HAZARD]==0);

  uint8_t frame[41];
  assert(hmi::buildWrite(frame,1,0,words)==41);
  assert(frame[0]==1 && frame[1]==0x10 && frame[6]==32);
  assert(frame[7]==0 && frame[8]==2);
  assert(frame[23]==255 && frame[24]==246); // signed pitch -1.0 deg
  assert(hmi::modbusCrc(frame,41)==0);
  const uint8_t known[]={1,3,0,0,0,10};
  assert(hmi::modbusCrc(known,6)==0xcdc5);

  EzhmiTransport lcd;
  lcd.begin(0); lcd.tick(2999,state);
  assert(hmi_uart_test::tx().empty());
  lcd.tick(3000,state);
  assert(hmi_uart_test::tx().size()==41);
  ack(); lcd.tick(3030,state); lcd.tick(3036,state);
  assert(lcd.linkOk(3036) && lcd.ack_count==1);
  lcd.tick(3200,state); ack(true); lcd.tick(3230,state); lcd.tick(3236,state);
  lcd.tick(3350,state);
  assert(lcd.error_count==1 && lcd.ack_count==1);
  assert(!lcd.linkOk(4036));
  lcd.tick(4200,state); ack(); lcd.tick(4230,state); lcd.tick(4236,state);
  assert(lcd.linkOk(4236) && lcd.ack_count==2);
  return 0;
}
