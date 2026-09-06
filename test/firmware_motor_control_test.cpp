#include <assert.h>
#include <stdio.h>

#include "config.h"
#include "motor_control.h"

namespace cfg = safestride_config;

HardwareSerial Serial;

namespace {

int g_motor_pwm = 0;
uint8_t g_motor_in1_level = LOW;
uint8_t g_motor_in2_level = LOW;

HallSample sample(uint32_t pulses, uint32_t period_us = 6283185UL) {
  HallSample value = {pulses, period_us, 0UL};
  return value;
}

void primeFeedback(DriveController& drive) {
  const HallSample stopped = {0UL, 0UL, 0xFFFFFFFFUL};
  drive.update(5000UL, stopped, stopped, 0L, false);
  drive.update(5000UL, stopped, stopped, 0L, false);
  assert(drive.feedbackReady());
}

}  // namespace

void pinMode(uint8_t, uint8_t) {}

void digitalWrite(uint8_t pin, uint8_t value) {
  if (pin == cfg::MOTOR_IN1_PIN) {
    g_motor_in1_level = value;
  } else if (pin == cfg::MOTOR_IN2_PIN) {
    g_motor_in2_level = value;
  }
}

int digitalRead(uint8_t) { return LOW; }

void analogWrite(uint8_t pin, int value) {
  if (pin == cfg::MOTOR_PWM_PIN) {
    g_motor_pwm = value;
  }
}

int analogRead(uint8_t) { return 0; }
int digitalPinToInterrupt(uint8_t) { return 0; }
void attachInterrupt(int, void (*)(), int) {}
void noInterrupts() {}
void interrupts() {}
uint32_t millis() { return 0UL; }
uint32_t micros() { return 0UL; }

void HardwareSerial::begin(uint32_t) {}
int HardwareSerial::available() { return 0; }
int HardwareSerial::read() { return -1; }
size_t HardwareSerial::write(uint8_t) { return 1U; }
size_t HardwareSerial::write(const uint8_t*, size_t length) {
  return length;
}
size_t HardwareSerial::print(const char*) { return 1U; }
size_t HardwareSerial::println(const char*) { return 1U; }

void run(DriveController& d, int ticks, int32_t target, uint32_t period,
         int16_t slope=0, uint8_t cap=100, bool brake=false) {
  static uint32_t pulses=0;
  for (int i=0;i<ticks;++i) {
    HallSample h = {++pulses,period,0};
    d.update(5000UL,h,h,target,true,true,0,false,slope,cap,brake);
    assert(g_motor_pwm >= 0 && g_motor_pwm <= cap);
    assert(!(g_motor_in1_level == HIGH && g_motor_in2_level == HIGH));
  }
}

int main() {
  // Feed-forward replaces the old FF10 plus hard minimum 80.
  DriveController d; d.begin(); primeFeedback(d);
  run(d,800,696,1504595UL);
  assert(g_motor_pwm >= 59 && g_motor_pwm <= 61);
  run(d,400,696,1504595UL,8);
  assert(g_motor_pwm >= 67 && g_motor_pwm <= 69);
  run(d,400,696,1504595UL,-45);
  assert(g_motor_pwm >= 14 && g_motor_pwm <= 16);
  run(d,400,696,1504595UL,-60);
  assert(g_motor_pwm <= 1); // Can reduce all the way to BRAKE without reverse.
  run(d,800,696,1504595UL,30,40);
  assert(g_motor_pwm == 40);
  run(d,1,0,1504595UL,0,100,true);
  assert(g_motor_pwm==0 && g_motor_in1_level==LOW && g_motor_in2_level==LOW);
  run(d,800,696,1504595UL);
  assert(g_motor_pwm >= 59); // No restart latch.
  d.disableImmediately(); assert(g_motor_pwm==0);

  // Output slew, zero target, fault/explicit brake bypass normal ramp.
  DriveController slew; slew.begin(); primeFeedback(slew);
  run(slew,100,696,1504595UL);
  assert(g_motor_pwm <= 10);
  int before=g_motor_pwm;
  HallSample h={2000,1504595UL,0};
  slew.update(5000,h,h,0,true,true,1160,true);
  assert(g_motor_pwm<=before);
  run(slew,1,0,1504595UL); assert(g_motor_pwm==0);

  // A 3.76-second period remains valid through 4.99 seconds of age.
  DriveController slow; slow.begin(); primeFeedback(slow);
  HallSample low={1,3760000UL,0};
  slow.update(5000,low,low,278,true);
  low.age_us=4990000UL;
  slow.update(5000,low,low,278,true);
  assert(slow.speedValid() && slow.leftVelocityMradS()>0);
  low.age_us=5000000UL;
  slow.update(5000,low,low,278,true);
  assert(!slow.speedValid() && slow.feedbackPwm()==0);
  assert(!slow.newPulse());

  // No pulse/first pulse is not a new zero-speed observation for PID.
  DriveController startup; startup.begin(); primeFeedback(startup);
  HallSample absent={0,0,0xFFFFFFFFUL};
  for(int i=0;i<900;++i) startup.update(5000,absent,absent,3000,true);
  assert(startup.hallFaultMask()==0 && startup.feedbackPwm()==0);
  assert(g_motor_pwm<=60);
  for(int i=0;i<300;++i) startup.update(5000,absent,absent,3000,true);
  assert(startup.hallFaultMask()==DriveController::HALL_FAULT_LEFT);
  assert(g_motor_pwm==0);

  // Independent overspeed BRAKE and automatic recovery with valid low speed.
  DriveController fast; fast.begin(); primeFeedback(fast);
  run(fast,800,696,1504595UL);
  run(fast,45,696,500000UL);
  assert(g_motor_pwm==0 && fast.braking());
  assert(fast.hallFaultMask()==0);
  run(fast,900,696,1504595UL);
  assert(g_motor_pwm>=59);

  // Direction changes pass through at least 150 ms of BRAKE.
  DriveController reverse; reverse.begin(); primeFeedback(reverse);
  run(reverse,800,696,1504595UL);
  bool saw_brake=false; int brake_ticks=0;
  for(int i=0;i<600;++i){
    HallSample stopped={static_cast<uint32_t>(3000+i),0,0};
    reverse.update(5000,stopped,stopped,-696,true);
    if(g_motor_pwm==0){saw_brake=true;++brake_ticks;}
    if(g_motor_in2_level==HIGH){assert(saw_brake && brake_ticks>=30);break;}
  }
  assert(g_motor_in2_level==HIGH && g_motor_in1_level==LOW);
  printf("firmware feed-forward, Hall, slew and BRAKE tests: OK\n");
  return 0;
}
