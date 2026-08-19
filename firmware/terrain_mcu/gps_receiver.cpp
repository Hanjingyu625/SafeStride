#include "gps_receiver.h"

#include <math.h>

#include "config.h"

#if !defined(SAFESTRIDE_HOST_BUILD)
#include <AltSoftSerial.h>
#include <TinyGPSPlus.h>
#endif

namespace cfg = safestride_terrain_config;

namespace {

#if !defined(SAFESTRIDE_HOST_BUILD)
AltSoftSerial g_gps_serial;
TinyGPSPlus g_gps_parser;
uint32_t g_last_sentence_ms = 0UL;

uint32_t roundedUnsigned32(double value) {
  if (value <= 0.0) {
    return 0UL;
  }
  if (value >= 4294967295.0) {
    return 0xFFFFFFFFUL;
  }
  return static_cast<uint32_t>(value + 0.5);
}

int32_t coordinateE7(double degrees) {
  const double scaled = degrees * 10000000.0;
  if (scaled >= 2147483647.0) {
    return 2147483647L;
  }
  if (scaled <= -2147483648.0) {
    return (-2147483647L - 1L);
  }
  return static_cast<int32_t>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
}
#endif

}  // namespace

void GpsReceiver::begin() {
#if !defined(SAFESTRIDE_HOST_BUILD)
  if (cfg::ENABLE_GPS) {
    g_gps_serial.begin(cfg::GPS_BAUD);
  }
#endif
}

void GpsReceiver::poll() {
#if !defined(SAFESTRIDE_HOST_BUILD)
  if (!cfg::ENABLE_GPS) {
    return;
  }
  uint16_t processed = 0U;
  while (g_gps_serial.available() > 0 && processed < 256U) {
    const int incoming = g_gps_serial.read();
    if (incoming < 0) {
      break;
    }
    if (g_gps_parser.encode(static_cast<char>(incoming))) {
      g_last_sentence_ms = millis();
    }
    ++processed;
  }
#endif
}

GpsSample GpsReceiver::sample(uint32_t now_ms) const {
  GpsSample result = {0L, 0L, 0UL, 0U, 0U};
#if !defined(SAFESTRIDE_HOST_BUILD)
  if (!cfg::ENABLE_GPS || g_last_sentence_ms == 0UL ||
      now_ms - g_last_sentence_ms > cfg::GPS_STALE_TIMEOUT_MS) {
    return result;
  }

  const bool fix_valid =
      g_gps_parser.location.isValid() &&
      g_gps_parser.location.age() <= cfg::GPS_STALE_TIMEOUT_MS;
  const bool speed_valid =
      g_gps_parser.speed.isValid() &&
      g_gps_parser.speed.age() <= cfg::GPS_STALE_TIMEOUT_MS;
  if (fix_valid) {
    result.flags |= FLAG_FIX_VALID;
    result.latitude_e7 = coordinateE7(g_gps_parser.location.lat());
    result.longitude_e7 = coordinateE7(g_gps_parser.location.lng());
  }
  if (speed_valid) {
    result.flags |= FLAG_SPEED_VALID;
    result.speed_mm_s = roundedUnsigned32(g_gps_parser.speed.mps() * 1000.0);
  }
  if (g_gps_parser.satellites.isValid()) {
    const uint32_t satellites = g_gps_parser.satellites.value();
    result.satellites = static_cast<uint8_t>(
        satellites > 255UL ? 255UL : satellites);
  }
#else
  (void)now_ms;
#endif
  return result;
}
