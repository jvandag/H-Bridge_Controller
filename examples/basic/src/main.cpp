#include <Arduino.h>
#include <HBridgeController.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

constexpr uint8_t HS1_pin = 18;
constexpr uint8_t LS1_pin = 21;
constexpr uint8_t HS2_pin = 17; // note 17 is used by chip for Serial2, and PSRAM (v2)
constexpr uint8_t LS2_pin = 19; 
constexpr uint32_t freq = 10000; // Starting frequency
constexpr float duty_cycle = 75.0f; // Default duty cycle
constexpr float drive_duty = 85.0f; // Duty cycle when driving the motor
constexpr uint32_t deadtime_ns = 200; // Dead time in nanoseconds between high PWM pulses
constexpr uint8_t button_pin = 27; // Supports internal pull-up
constexpr uint32_t button_debounce_ms = 50;

HBridgeController hbridge1(
  HS1_pin, LS1_pin, HS2_pin, LS2_pin,
  freq, duty_cycle, MCPWM_UNIT_0, // each H-bridge needs its own MCPWM_UNIT
  deadtime_ns, FORWARD
);

SemaphoreHandle_t hb_mutex = nullptr;
bool button_pressed = false;

void button_task(void* /*param*/) {
  bool last_pressed = false;
  int last_level = digitalRead(button_pin);
  int stable_level = last_level;
  TickType_t last_change_tick = xTaskGetTickCount();
  const TickType_t debounce_ticks = pdMS_TO_TICKS(button_debounce_ms);

  for (;;) {
    const int level = digitalRead(button_pin);
    const TickType_t now = xTaskGetTickCount();
    if (level != last_level) {
      last_level = level;
      last_change_tick = now;
    }
    if ((now - last_change_tick) >= debounce_ticks && level != stable_level) {
      stable_level = level;
      button_pressed = (stable_level == LOW);
    }
    const bool pressed = button_pressed;
    if (pressed != last_pressed) {
      if (pressed) {
        Serial.println("Button pressed: forcing DRIVE + REVERSE");
      }
      else {
        Serial.println("Button released: returning to BODY_DIODE + FORWARD");
      }
      if (xSemaphoreTake(hb_mutex, portMAX_DELAY) == pdTRUE) {
        if (pressed) {
          hbridge1.set_state(DRIVE);
          hbridge1.set_direction(REVERSE);
          hbridge1.set_freq(freq);
          hbridge1.set_duty_cycle(drive_duty);
        }
        else {
          hbridge1.set_state(BODY_DIODE);
          hbridge1.set_direction(FORWARD);
          hbridge1.set_freq(freq);
          hbridge1.set_duty_cycle(duty_cycle);
        }
        xSemaphoreGive(hb_mutex);
      }
      last_pressed = pressed;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  delay(250);

  hb_mutex = xSemaphoreCreateMutex();

  pinMode(button_pin, INPUT_PULLUP);
  button_pressed = (digitalRead(button_pin) == LOW);

  xTaskCreatePinnedToCore(
    button_task,
    "button_task",
    2048,
    nullptr,
    1,
    nullptr,
    1
  );

  // Default: BODY_DIODE forward at 75% duty.
  if (xSemaphoreTake(hb_mutex, portMAX_DELAY) == pdTRUE) {
    hbridge1.start(BODY_DIODE);
    hbridge1.set_direction(FORWARD);
    hbridge1.set_duty_cycle(duty_cycle);
    hbridge1.set_freq(freq);
    xSemaphoreGive(hb_mutex);
  }
}

void service_other_tasks() {
  // Placeholder for other work (hall sensor processing, sensor reads, display updates).
  // This continues to run even while the button forces DRIVE/REVERSE.
}

void loop() {
  service_other_tasks();
  delay(5);
}
