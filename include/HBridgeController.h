/*
Controls an H-Bridge motor driver using four PWM pins.

Author: Jeremiah Vandagrift
Date: 03-Feb-2026
*/
#ifndef HBRIDGE_CONTROLLER_H
#define HBRIDGE_CONTROLLER_H

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


/*
* `STOPPED`:        Drives all gates low (motor disconnected)

* `BODY_DIODE`:     Only PWMs lowside for regen breaking while one lowside is fixed closed.

* `COMPLIMENTARY`:  Uses complementary PWM with deadtime between to open the high side gate 
*                 while the lowside gate being PWMed is low. Used for regen breaking. Leaves one
*                 lowside gate closed at all times like BODY_DIODE

* `DRIVE`:          Same as complimentary but always keeps a side gate closed instead of a lowside
*                 This allows the motor to be driven from source and controlled by the duty cycle.
*/
enum state {
    STOPPED       = 0,
    BODY_DIODE    = 1,
    COMPLIMENTARY = 2, 
    DRIVE         = 3
};

class HBridgeController {
    public: 
        HBridgeController(uint8_t HS1, uint8_t LS1, uint8_t HS2, uint8_t LS2, uint32_t freq = 1000, float duty_cycle = 75.0f, 
                          mcpwm_unit_t MCPWM_UNIT = MCPWM_UNIT_0, uint32_t deadtime_ns = 200, direction start_direction = FORWARD,
                          state start_state = BODY_DIODE);
        bool set_direction(direction dir);
        bool set_freq(uint32_t freq); // must not have a period less than the deadtime
        bool set_duty_cycle(float duty_cycle); // 1 - 100
        direction get_direction();
        uint32_t get_freq();
        float get_duty_cycle();
        state get_state();
        bool set_state(state new_state);
        bool start(state start_state = DRIVE);
        bool stop();
    private:
        uint8_t HS1, HS2, LS1, LS2;
        uint32_t freq;
        float duty_cycle;
        mcpwm_unit_t MCPWM_UNIT = MCPWM_UNIT_0;
        uint32_t deadtime_ns;
        direction dir;
        state current_state = STOPPED;
        mcpwm_config_t cfg = {};
        static constexpr uint8_t k_ledc_resolution_bits = 10;
        static constexpr uint32_t k_ledc_max_duty = (1u << k_ledc_resolution_bits) - 1u;
        static constexpr uint32_t k_stop_brake_ms = 5;
        uint8_t ledc_ch_hs1 = 0;
        uint8_t ledc_ch_ls1 = 0;
        uint8_t ledc_ch_hs2 = 0;
        uint8_t ledc_ch_ls2 = 0;
        bool hw_pwm_attached = false;

        static uint32_t deadtime_ticks_from_ns(uint32_t ns);
        bool config_mcpwm(state config_state);
        void init_hw_pwm_channels();
        void attach_hw_pwm();
        void detach_hw_pwm();
        void disable_mcpwm_outputs();
        void apply_hw_pwm_duty();
        uint32_t duty_to_ledc(float duty) const;
};

#endif // HBRIDGE_CONTROLLER_H
