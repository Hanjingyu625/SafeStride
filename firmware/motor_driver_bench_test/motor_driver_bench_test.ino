#include <Arduino.h>
#include <avr/wdt.h>

#if !defined(ARDUINO_ARCH_AVR)
#error "This bench sketch is intended for an AVR Arduino Uno/Nano."
#endif

namespace {

constexpr uint32_t SERIAL_BAUD = 115200UL;

// Same single-driver pin map as the production SafeStride firmware.
constexpr uint8_t MOTOR_PWM_PIN = 5U;
constexpr uint8_t MOTOR_IN1_PIN = 6U;
constexpr uint8_t MOTOR_IN2_PIN = 8U;

// Deliberately limited for a lifted-wheel bench test. Arduino PWM is 0..255.
constexpr int16_t MAX_TEST_PWM = 100;
constexpr uint32_t MIN_RUN_MS = 50UL;
constexpr uint32_t MAX_RUN_MS = 3000UL;
constexpr size_t COMMAND_BUFFER_SIZE = 64U;

char g_command_buffer[COMMAND_BUFFER_SIZE];
size_t g_command_length = 0U;
bool g_running = false;
int16_t g_signed_pwm = 0;
uint32_t g_stop_at_ms = 0UL;

bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

void preloadLow(uint8_t pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void stopOutputs() {
  // Remove PWM before changing direction signals.
  analogWrite(MOTOR_PWM_PIN, 0);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  g_running = false;
  g_signed_pwm = 0;
  g_stop_at_ms = 0UL;
}

void printHelp() {
  Serial.println(F("SafeStride SZH-GNP521 bench test"));
  Serial.println(F("Wheels must be lifted; keep battery fuse/E-stop ready."));
  Serial.println(F("Commands (newline terminated):"));
  Serial.println(F("  RUN <signed_pwm> <duration_ms> CONFIRM"));
  Serial.println(F("  STOP"));
  Serial.println(F("  STATUS"));
  Serial.println(F("  HELP"));
  Serial.println(F("Limits: signed_pwm=-100..100, duration_ms=50..3000"));
  Serial.println(F("Example: RUN 30 1000 CONFIRM"));
}

void printStatus() {
  if (!g_running) {
    Serial.println(F("STATUS STOPPED"));
    return;
  }

  const uint32_t now = millis();
  const uint32_t remaining = deadlineReached(now, g_stop_at_ms)
      ? 0UL
      : g_stop_at_ms - now;
  Serial.print(F("STATUS RUNNING pwm="));
  Serial.print(g_signed_pwm);
  Serial.print(F(" remaining_ms="));
  Serial.println(remaining);
}

bool parseLongStrict(const char* text, long& value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char* end = nullptr;
  value = strtol(text, &end, 10);
  return end != text && *end == '\0';
}

void startRun(int16_t signed_pwm, uint32_t duration_ms) {
  stopOutputs();
  const uint8_t magnitude = static_cast<uint8_t>(
      signed_pwm < 0 ? -signed_pwm : signed_pwm);

  if (signed_pwm > 0) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  } else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  }
  analogWrite(MOTOR_PWM_PIN, magnitude);

  g_running = true;
  g_signed_pwm = signed_pwm;
  g_stop_at_ms = millis() + duration_ms;
  Serial.print(F("RUNNING pwm="));
  Serial.print(signed_pwm);
  Serial.print(F(" duty_percent="));
  Serial.print(static_cast<float>(magnitude) * 100.0F / 255.0F, 1);
  Serial.print(F(" duration_ms="));
  Serial.println(duration_ms);
}

void handleCommand(char* line) {
  char* command = strtok(line, " \t");
  if (command == nullptr) {
    return;
  }

  if (strcmp(command, "STOP") == 0 || strcmp(command, "stop") == 0) {
    stopOutputs();
    Serial.println(F("STOPPED"));
    return;
  }
  if (strcmp(command, "STATUS") == 0 || strcmp(command, "status") == 0) {
    printStatus();
    return;
  }
  if (strcmp(command, "HELP") == 0 || strcmp(command, "help") == 0) {
    printHelp();
    return;
  }
  if (strcmp(command, "RUN") != 0 && strcmp(command, "run") != 0) {
    stopOutputs();
    Serial.println(F("ERROR unknown command; outputs stopped"));
    return;
  }

  const char* pwm_text = strtok(nullptr, " \t");
  const char* duration_text = strtok(nullptr, " \t");
  const char* confirmation = strtok(nullptr, " \t");
  const char* extra = strtok(nullptr, " \t");
  long pwm = 0L;
  long duration = 0L;

  if (!parseLongStrict(pwm_text, pwm) ||
      !parseLongStrict(duration_text, duration) || confirmation == nullptr ||
      strcmp(confirmation, "CONFIRM") != 0 || extra != nullptr) {
    stopOutputs();
    Serial.println(F("ERROR syntax; outputs stopped"));
    return;
  }
  if (pwm == 0L || pwm < -MAX_TEST_PWM || pwm > MAX_TEST_PWM) {
    stopOutputs();
    Serial.println(F("ERROR pwm must be -100..-1 or 1..100; outputs stopped"));
    return;
  }
  if (duration < static_cast<long>(MIN_RUN_MS) ||
      duration > static_cast<long>(MAX_RUN_MS)) {
    stopOutputs();
    Serial.println(F("ERROR duration must be 50..3000 ms; outputs stopped"));
    return;
  }

  startRun(
      static_cast<int16_t>(pwm),
      static_cast<uint32_t>(duration));
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) {
      return;
    }
    const char value = static_cast<char>(incoming);
    if (value == '\r') {
      continue;
    }
    if (value == '\n') {
      g_command_buffer[g_command_length] = '\0';
      handleCommand(g_command_buffer);
      g_command_length = 0U;
      continue;
    }
    if (g_command_length + 1U >= COMMAND_BUFFER_SIZE) {
      g_command_length = 0U;
      stopOutputs();
      Serial.println(F("ERROR command too long; outputs stopped"));
      continue;
    }
    g_command_buffer[g_command_length++] = value;
  }
}

}  // namespace

void setup() {
  MCUSR = 0U;
  wdt_disable();

  // Preload all output latches LOW before switching pins to output mode.
  preloadLow(MOTOR_PWM_PIN);
  preloadLow(MOTOR_IN1_PIN);
  preloadLow(MOTOR_IN2_PIN);
  stopOutputs();

  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(20UL);
  printHelp();
  Serial.println(F("STATUS STOPPED"));
  wdt_enable(WDTO_1S);
}

void loop() {
  wdt_reset();
  readSerialCommands();
  if (g_running && deadlineReached(millis(), g_stop_at_ms)) {
    stopOutputs();
    Serial.println(F("STOPPED duration elapsed"));
  }
}
