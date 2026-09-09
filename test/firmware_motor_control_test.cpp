#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "motor_control.h"

namespace cfg = safestride_config;

HardwareSerial Serial;

namespace {

int g_motor_pwm = 0;
uint8_t g_motor_in1_level = LOW;
uint8_t g_motor_in2_level = LOW;

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
  // Twelve magnets: one second per pulse is pi/6 rad/s, not pi/3.
  DriveController calibrated; calibrated.begin(); primeFeedback(calibrated);
  run(calibrated,100,696,1000000UL);
  assert(calibrated.leftVelocityMradS() >= 523 &&
         calibrated.leftVelocityMradS() <= 524);

  // Threshold boundary: 27.5 ms is 7.88 km/h, 26.5 ms is 8.18 km/h.
  // A single high observation cannot brake; its successor must brake at once.
  DriveController boundary; boundary.begin(); primeFeedback(boundary);
  run(boundary,100,696,27500UL);
  assert(boundary.appliedTargetMradS()>0 && boundary.hallFaultMask()==0);
  run(boundary,1,696,26500UL);
  assert(boundary.appliedTargetMradS()>0 && boundary.hallFaultMask()==0);
  run(boundary,1,696,26500UL);
  assert(boundary.braking() && boundary.appliedTargetMradS()==0 &&
         boundary.hallFaultMask()==0);
  run(boundary,1,696,30000UL); // 7.23 km/h: stay braked.
  assert(boundary.braking() && boundary.appliedTargetMradS()==0);
  run(boundary,1,696,32000UL); // 6.77 km/h: release and begin PWM slew.
  run(boundary,20,696,32000UL);
  assert(boundary.appliedTargetMradS()>0);

  {
    DriveController terrain; terrain.begin(); primeFeedback(terrain);
    run(terrain,800,696,1504595UL);
    const int initial = g_motor_pwm;
    HallSample h = {1000, 1504595UL, 0};
    int previous = initial;
    for (int i=1; i<=600; ++i) {
      terrain.update(5000,h,h,0,true,true,0,false,0,100,false,true);
      assert(g_motor_pwm <= previous);
      assert(abs(g_motor_pwm - static_cast<int>(initial*(600-i)/600.0F+0.5F)) <= 1);
      previous = g_motor_pwm;
    }
    assert(g_motor_pwm==0 && terrain.braking());
    assert(g_motor_in1_level==LOW && g_motor_in2_level==LOW);
    terrain.update(5000,h,h,0,true,true,0,false,0,100,false,true);
    assert(g_motor_pwm==0);  // repeated stop commands do not restart the fade
    HallSample absent = {1000,0,0xFFFFFFFFUL};
    for (int i=0;i<200;++i)
      terrain.update(5000,absent,absent,696,true,true);
    assert(g_motor_pwm >= 9 && g_motor_pwm <= 10);
    // Recovery uses 10 count/s only until it catches the normal controller
    // output. A later demand change returns to the ordinary 20 count/s slew.
    run(terrain,1020,696,752297UL);
    assert(g_motor_pwm == 60);
    run(terrain,20,696,752297UL,30);
    assert(g_motor_pwm >= 62);
    terrain.update(5000,absent,absent,0,true,true,0,false,0,100,false,true);
    terrain.update(5000,absent,absent,0,false);
    terrain.update(5000,absent,absent,0,true,true,0,false,0,100,false,true);
    assert(g_motor_pwm==0);  // an interlock cannot restore captured PWM
  }
  // Feed-forward replaces the old FF10 plus hard minimum 80.
  DriveController d; d.begin(); primeFeedback(d);
  run(d,800,696,752297UL);
  assert(g_motor_pwm >= 59 && g_motor_pwm <= 61);
  run(d,400,696,752297UL,8);
  assert(g_motor_pwm >= 67 && g_motor_pwm <= 69);
  run(d,400,696,752297UL,-45);
  assert(g_motor_pwm >= 14 && g_motor_pwm <= 16);
  run(d,400,696,752297UL,-60);
  assert(g_motor_pwm <= 1); // Can reduce all the way to BRAKE without reverse.
  run(d,800,696,752297UL,30,40);
  assert(g_motor_pwm == 40);
  run(d,1,0,752297UL,0,100,true);
  assert(g_motor_pwm==0 && g_motor_in1_level==LOW && g_motor_in2_level==LOW);
  run(d,800,696,752297UL);
  assert(g_motor_pwm >= 59); // No restart latch.
  d.disableImmediately(); assert(g_motor_pwm==0);

  // Output slew, zero target, fault/explicit brake bypass normal ramp.
  DriveController slew; slew.begin(); primeFeedback(slew);
  run(slew,100,696,752297UL);
  assert(g_motor_pwm <= 10);
  int before=g_motor_pwm;
  HallSample h={2000,752297UL,0};
  slew.update(5000,h,h,0,true,true,1160,true);
  assert(g_motor_pwm<=before);
  run(slew,1,0,752297UL); assert(g_motor_pwm==0);

  // A 1.88-second period remains valid through 4.99 seconds of age.
  DriveController slow; slow.begin(); primeFeedback(slow);
  HallSample low={1,1880000UL,0};
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

  // A single above-target period is normal closed-loop error, not a hard
  // BRAKE. Replaying that stale measurement at 200 Hz must not turn it into
  // false evidence of sustained overspeed.
  DriveController fast; fast.begin(); primeFeedback(fast);
  run(fast,800,696,752297UL);

  // 0.15 m/s is above the former target+0.05 m/s trip point but below the
  // absolute 8 km/h safety limit. PID correction must continue driving.
  run(fast,100,696,400000UL);
  assert(g_motor_pwm>0 && !fast.braking());

  HallSample one_fast_period={10000,25000UL,0}; // 8.67 km/h, absolute overspeed.
  fast.update(5000,one_fast_period,one_fast_period,696,true);
  for(int i=0;i<100;++i) {
    one_fast_period.age_us+=5000UL;
    fast.update(5000,one_fast_period,one_fast_period,696,true);
  }
  assert(g_motor_pwm>0 && !fast.braking());

  // A second independent absolute-overspeed period confirms the condition.
  one_fast_period.pulse_count++;
  one_fast_period.age_us=0;
  fast.update(5000,one_fast_period,one_fast_period,696,true);
  assert(g_motor_pwm==0 && fast.braking());
  assert(fast.hallFaultMask()==0);

  // A newly observed low-speed period releases the transient BRAKE.
  HallSample recovered={one_fast_period.pulse_count+1,752297UL,0};
  fast.update(5000,recovered,recovered,696,true);
  for(int i=0;i<900;++i) {
    recovered.pulse_count++;
    fast.update(5000,recovered,recovered,696,true);
  }
  assert(g_motor_pwm>=59);

  // One physically impossible Hall period must not become a latched fault
  // merely because the 200 Hz loop sees that cached value repeatedly.
  DriveController noisy; noisy.begin(); primeFeedback(noisy);
  HallSample impossible={20000,20000UL,0};
  noisy.update(5000,impossible,impossible,696,true);
  for(int i=0;i<100;++i) {
    impossible.age_us+=5000UL;
    noisy.update(5000,impossible,impossible,696,true);
  }
  assert(noisy.hallFaultMask()==0);

  // Direction changes pass through at least 150 ms of BRAKE.
  DriveController reverse; reverse.begin(); primeFeedback(reverse);
  run(reverse,800,696,752297UL);
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
