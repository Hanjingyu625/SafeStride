#pragma once
#include <Arduino.h>
#include <vector>

// Host-only UART double. Real AVR builds use the installed AltSoftSerial.
namespace hmi_uart_test {
inline std::vector<uint8_t>& rx() { static std::vector<uint8_t> v; return v; }
inline std::vector<uint8_t>& tx() { static std::vector<uint8_t> v; return v; }
}
class AltSoftSerial {
 public:
  void begin(uint32_t) {}
  int available() { return hmi_uart_test::rx().size(); }
  int read() {
    auto& bytes = hmi_uart_test::rx();
    if (bytes.empty()) return -1;
    const int value = bytes.front(); bytes.erase(bytes.begin()); return value;
  }
  size_t write(const uint8_t* bytes, size_t n) {
    hmi_uart_test::tx().insert(hmi_uart_test::tx().end(), bytes, bytes+n);
    return n;
  }
};
