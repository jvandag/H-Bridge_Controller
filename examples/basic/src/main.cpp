#include <Arduino.h>
#include <HBridgeController.h>

constexpr uint8_t T1_pin = 18;
constexpr uint8_t T2_pin = 19;
constexpr uint8_t T3_pin = 32;
constexpr uint8_t T4_pin = 14;
constexpr uint32_t freq = 10000; // Starting frequency
constexpr float duty_cycle = 75.0f; // Starting duty cycle
constexpr uint32_t deadtime_ns = 200; // Dead time in nanoseconds between high PWM pulses

HBridgeController hbridge1(
  T1_pin, T2_pin, T3_pin, T4_pin,
  freq, duty_cycle, MCPWM_UNIT_0, // each H-bridge needs its own MCPWM_UNIT
  deadtime_ns, FORWARD
);

HBridgeController hbridge2(
  T1_pin, T2_pin, T3_pin, T4_pin,
  freq, duty_cycle, MCPWM_UNIT_1, // each H-bridge needs its own MCPWM_UNIT
  deadtime_ns, REVERSE
);

void setup() {
  hbridge1.start();
  hbridge2.start();
}

void loop() {
  hbridge1.set_direction(FORWARD);
  hbridge1.set_freq(10000);
  hbridge1.set_duty_cycle(75.0f);
  delay(1);
  hbridge1.set_freq(20000);
  delay(1);
  hbridge1.set_direction(REVERSE);
  delay(1);
  hbridge1.set_duty_cycle(0.0f);
  delay(1);
}
