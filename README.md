# H-Bridge Controller
An H-Bridge controller for **ESP32** microcontrollers. Uses complementary hardware PWM (MCPWM) with built in dead time between states to prevent shorting between V+ and and GND. Allows live configuration of frequency, duty cycle, and direction.

This module is meant to control an H-Bridge that uses <u>ONLY NMOS transistors</u>, You should be able to adapt it to a PMOS only H-bridge by swaping the T1 connections with the T2 connections and the T3 connections with the T4 connections, although this has not been tested.

## Primary H-Bridge Stages
**Disconnected:** all 
**Stage 1:**

## Example Usage:
```C++
#include "HBridgeController.h"

#define T1_pin 18
#define T2_pin 19
#define T3_pin 32
#define T4_pin 14
#define freq 10000 // Starting frequency
#define duty_cycle 75.0f // Starting duty cycle
#define deadtime_ns 200 // Dead time in nanoseconds between high PWM pulses
                        // Prevents shorting V+ to GND

// Initialize H-bridges
HBridgeController hbridge1(
  T1_pin, T2_pin, T3_pin, T4_pin,
  freq, duty_cycle, MCPWM_UNIT_0, // each H-bridge need it's own MCPWM_UNIT
  deadtime_ns, FORWARD
);

HBridgeController hbridge2(
  T1_pin, T2_pin, T3_pin, T4_pin,
  freq, duty_cycle, MCPWM_UNIT_1, // each H-bridge need it's own MCPWM_UNIT
  deadtime_ns, REVERSE
);


void setup() {
  hbridge1.start();
  hbridge2.start();
}

void loop() {
  hbridge.set_direction(FORWARD); // Set direction to forward
  hbridge.set_freq(10000); // Change frequency
  hbridge.set_duty_cycle(75.0f); // Change duty cycle
  delay(1);
  hbridge.set_freq(20000); // Change frequency again
  delay(1);
  hbridge.set_direction(REVERSE); // Swap direction to reverse
  delay(1);
  hbridge.set_duty_cycle(0.0f); // Change duty cycle again
  delay(1);
}
```

### Important Notes
- The ESP32-WROOM-32 chip only has two MCPWM_UNITs and thus only supports two H-bridge controllers at a time.
- Setting the duty cylce to 0 percent will short the motor terminals together by connecting them to the ground node if the H-bridge is connected to the same pins in the diagram. It will stay configuration without modulating until the duty cycle is changed.
- Setting the duty cylce to 100 percent will connect one motor terminal to ground and the other to V+ if the H-bridge is connected to the same pins in the diagram. It will stay configuration without modulating until the duty cycle is changed.

## Output Preview
