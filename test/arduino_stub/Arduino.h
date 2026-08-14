#pragma once

// Minimal host-only Arduino API used to syntax-check the firmware without an
// installed board core. This is not linked or uploaded to a microcontroller.

#define SAFESTRIDE_HOST_BUILD 1

#include <stddef.h>
#include <stdint.h>

#define HIGH 0x1
#define LOW 0x0
#define INPUT_PULLUP 0x2
#define OUTPUT 0x1
#define CHANGE 0x3
#define FALLING 0x2
#define RISING 0x3
#define NOT_AN_INTERRUPT (-1)
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define PI 3.1415926535897932384626433832795

template <typename T>
T constrain(T value, T lower, T upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

class Stream {
 public:
  virtual ~Stream() {}
  virtual int available() = 0;
  virtual int read() = 0;
  virtual size_t write(uint8_t value) = 0;
  virtual size_t write(const uint8_t* data, size_t length) = 0;
};

class HardwareSerial : public Stream {
 public:
  void begin(uint32_t);
  int available();
  int read();
  size_t write(uint8_t);
  size_t write(const uint8_t*, size_t);
  size_t print(const char* value);
  size_t println(const char* value);
};

extern HardwareSerial Serial;

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
void analogWrite(uint8_t pin, int value);
int analogRead(uint8_t pin);
int digitalPinToInterrupt(uint8_t pin);
void attachInterrupt(int interrupt_number, void (*callback)(), int mode);
void noInterrupts();
void interrupts();
uint32_t millis();
uint32_t micros();
void delayMicroseconds(unsigned int microseconds);
