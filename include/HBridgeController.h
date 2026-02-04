/*
Controls an H-Bridge motor driver using four PWM pins.

Author: Jeremiah Vandagrift
Date: 03-Feb-2026
*/
#pragma once

#include <Arduino.h>

extern "C" { // Include C headers
  #include "driver/mcpwm.h"
  // #include "esp_clk.h"
  #include "esp32/clk.h"
}

enum direction {
    REVERSE = 0,
    FORWARD = 1
};

enum run_state {
    STOPPED = 0,
    RUNNING = 1
};

class HBridgeController {
    public: 
        HBridgeController(uint8_t T1_pin, uint8_t T2_pin, uint8_t T3_pin, uint8_t T4_pin, uint32_t freq = 1000, float duty_cycle = 0.75, 
        mcpwm_unit_t MCPWM_UNIT = MCPWM_UNIT_0, uint32_t deadtime_ns = 200, direction start_direction = FORWARD);
        bool set_direction(direction dir);
        bool set_freq(uint32_t freq); // must not have a period less than the deadtime
        bool set_duty_cycle(float duty_cycle); // 1 - 100
        direction get_direction();
        uint32_t get_freq();
        float get_duty_cycle();
        run_state get_state();
        bool start();
        bool stop();
    private:
        uint8_t T1_pin, T2_pin, T3_pin, T4_pin;
        uint32_t freq;
        float duty_cycle;
        mcpwm_unit_t MCPWM_UNIT = MCPWM_UNIT_0;
        uint32_t deadtime_ns;
        direction dir;
        run_state state = STOPPED;
        mcpwm_config_t cfg = {};

        static uint32_t deadtime_ticks_from_ns(uint32_t ns);
};
