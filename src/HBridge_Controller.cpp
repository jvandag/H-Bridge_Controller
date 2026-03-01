#include "HBridgeController.h"
#include <stdexcept>
#include <Esp.h>

// assign default values and call init function to initialize MCPWM
HBridgeController::HBridgeController(uint8_t HS1, uint8_t LS1, uint8_t HS2, uint8_t LS2,
                                     uint32_t freq, float duty_cycle, mcpwm_unit_t MCPWM_UNIT,
                                     uint32_t deadtime_ns, direction start_direction, state start_state)
: HS1(HS1), LS1(LS1), HS2(HS2), LS2(LS2), freq(freq), duty_cycle(duty_cycle), MCPWM_UNIT(MCPWM_UNIT),
  deadtime_ns(deadtime_ns), dir(start_direction) {
    this->init_hw_pwm_channels();
}

void HBridgeController::init_hw_pwm_channels() {
    const uint8_t base = (this->MCPWM_UNIT == MCPWM_UNIT_0) ? 0 : 4;
    this->ledc_ch_hs1 = base + 0;
    this->ledc_ch_ls1 = base + 1;
    this->ledc_ch_hs2 = base + 2;
    this->ledc_ch_ls2 = base + 3;
}

/*
* @brief changes the direction of the H-Bridge, changing the direction of current through the load
* 
* @param `direction` dir: the desired direction
* @return `bool` true if successful, false otherwise
*
*/
bool HBridgeController::set_direction(direction dir) {
    this->dir = dir;
    return this->set_duty_cycle(this->duty_cycle); // Re-apply duty cycle to update direction
}


/*
* @brief Changes the frequency of the H-Bridge
* 
* @param `uint32_t` freq: new frequency in Hz
* @return `bool`: true if successful, false otherwise
*
*/
bool HBridgeController::set_freq(uint32_t freq) {
    String err_msg = "";

    if (this->current_state == DRIVE || this->current_state == COMPLIMENTARY) {
        if (this->deadtime_ns <= 0) {
            err_msg = "Deadtime cannot be less than or equal to zero";
            throw std::invalid_argument(err_msg.c_str());
        }
        const uint64_t max_freq = 1000000000ULL / (static_cast<uint64_t>(deadtime_ns) * 2ULL);
        if (freq >= max_freq) {
            err_msg = String("Frequency too high for the given deadtime. Max: ") + String(max_freq) + String(" Hz");
            throw std::invalid_argument(err_msg.c_str());
        }
        this->freq = freq;
        this->cfg.frequency = this->freq;
        mcpwm_set_frequency(this->MCPWM_UNIT, MCPWM_TIMER_0, freq);
        mcpwm_set_frequency(this->MCPWM_UNIT, MCPWM_TIMER_1, freq);
    }
    else {
        this->freq = freq;
        this->attach_hw_pwm();
        this->apply_hw_pwm_duty();
    }

    return true;
}



/*
* @brief Changes the duty cycle of the H-Bridge
* 
* @param `float` duty_cycle: new duty cycle percentage (0-100)
* @return `bool`: true if successful, false otherwise
*
*/
bool HBridgeController::set_duty_cycle(float duty_cycle) {
    if (duty_cycle < 0 || duty_cycle > 100) {
        throw std::invalid_argument("Duty cycle must be between 0 and 100");
        return false;
    }
    this->duty_cycle = duty_cycle;
    
    if (this->current_state == DRIVE || this->current_state == COMPLIMENTARY) {
        if (this->deadtime_ns <= 0) {
            throw std::invalid_argument("Deadtime cannot be less than or equal to zero");
        }
        if (this->current_state == DRIVE) {
            if (this->dir == FORWARD) {
                // HS1 high, LS2 PWM. All other gates low.
                mcpwm_set_signal_high(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_GEN_A); // HS1
                mcpwm_set_signal_low(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_GEN_B);  // LS1
                mcpwm_set_signal_low(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_GEN_A);  // HS2

                mcpwm_set_duty(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_GEN_B, this->duty_cycle); // LS2
                mcpwm_set_duty_type(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_GEN_B, MCPWM_DUTY_MODE_0);
            }
            else {
                // HS2 high, LS1 PWM. All other gates low.
                mcpwm_set_signal_high(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_GEN_A); // HS2
                mcpwm_set_signal_low(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_GEN_B);  // LS2
                mcpwm_set_signal_low(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_GEN_A);  // HS1

                mcpwm_set_duty(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_GEN_B, this->duty_cycle); // LS1
                mcpwm_set_duty_type(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM_DUTY_MODE_0);
            }
        }
        else {
            // COMPLIMENTARY
            mcpwm_set_duty(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, this->dir == REVERSE ? this->duty_cycle : 0);
            mcpwm_set_duty_type(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            mcpwm_set_duty_type(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

            mcpwm_set_duty(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, this->dir == FORWARD ? this->duty_cycle : 0);
            mcpwm_set_duty_type(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
            mcpwm_set_duty_type(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
        }
    }
    else {
        this->attach_hw_pwm();
        this->apply_hw_pwm_duty();
    }

    return true;
}


/*
* @brief Determines the number of MCPWM deadtime ticks corresponding to the given
* nanoseconds.
* 
* @return `float`: current duty cycle percentage (0-100)
*
*/
float HBridgeController::get_duty_cycle() {
    return this->duty_cycle;
}

direction HBridgeController::get_direction() {
    return this->dir;
}

uint32_t HBridgeController::get_freq() {
    return this->freq;
}

state HBridgeController::get_state() {
    return this->current_state;
}

uint32_t HBridgeController::duty_to_ledc(float duty) const {
    if (duty <= 0.0f) {
        return 0;
    }
    if (duty >= 100.0f) {
        return k_ledc_max_duty;
    }
    return static_cast<uint32_t>((duty * static_cast<float>(k_ledc_max_duty)) / 100.0f);
}

void HBridgeController::attach_hw_pwm() {
    ledcSetup(this->ledc_ch_hs1, this->freq, k_ledc_resolution_bits);
    ledcSetup(this->ledc_ch_ls1, this->freq, k_ledc_resolution_bits);
    ledcSetup(this->ledc_ch_hs2, this->freq, k_ledc_resolution_bits);
    ledcSetup(this->ledc_ch_ls2, this->freq, k_ledc_resolution_bits);

    if (!this->hw_pwm_attached) {
        ledcAttachPin(this->HS1, this->ledc_ch_hs1);
        ledcAttachPin(this->LS1, this->ledc_ch_ls1);
        ledcAttachPin(this->HS2, this->ledc_ch_hs2);
        ledcAttachPin(this->LS2, this->ledc_ch_ls2);
        this->hw_pwm_attached = true;
    }
}

void HBridgeController::detach_hw_pwm() {
    if (!this->hw_pwm_attached) {
        return;
    }
    ledcDetachPin(this->HS1);
    ledcDetachPin(this->LS1);
    ledcDetachPin(this->HS2);
    ledcDetachPin(this->LS2);
    this->hw_pwm_attached = false;
}

void HBridgeController::disable_mcpwm_outputs() {
    mcpwm_stop(this->MCPWM_UNIT, MCPWM_TIMER_0);
    mcpwm_stop(this->MCPWM_UNIT, MCPWM_TIMER_1);

    mcpwm_set_signal_low(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_GEN_A);
    mcpwm_set_signal_low(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_GEN_B);
    mcpwm_set_signal_low(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_GEN_A);
    mcpwm_set_signal_low(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_GEN_B);

    gpio_reset_pin(static_cast<gpio_num_t>(this->HS1));
    gpio_reset_pin(static_cast<gpio_num_t>(this->LS1));
    gpio_reset_pin(static_cast<gpio_num_t>(this->HS2));
    gpio_reset_pin(static_cast<gpio_num_t>(this->LS2));
    this->hw_pwm_attached = false;
}

void HBridgeController::apply_hw_pwm_duty() {
    const uint32_t duty = this->duty_to_ledc(this->duty_cycle);
    const uint32_t on = k_ledc_max_duty;
    const uint32_t off = 0;

    switch (this->current_state) {
        case BODY_DIODE:
            if (this->dir == FORWARD) {
                ledcWrite(this->ledc_ch_hs1, off);
                ledcWrite(this->ledc_ch_hs2, off);
                ledcWrite(this->ledc_ch_ls1, on);
                ledcWrite(this->ledc_ch_ls2, duty);
            }
            else {
                ledcWrite(this->ledc_ch_hs1, off);
                ledcWrite(this->ledc_ch_hs2, off);
                ledcWrite(this->ledc_ch_ls1, duty);
                ledcWrite(this->ledc_ch_ls2, on);
            }
            break;
        case STOPPED:
        default:
            ledcWrite(this->ledc_ch_hs1, off);
            ledcWrite(this->ledc_ch_ls1, off);
            ledcWrite(this->ledc_ch_hs2, off);
            ledcWrite(this->ledc_ch_ls2, off);
            break;
    }
}


/*
* @brief initializes and configures the H-Bridge pins.
*
* @return `bool`: true if successful, false otherwise
*
*/
bool HBridgeController::start(state start_state) {
    // Route MCPWM signals to the GPIO matrix
    return this->set_state(start_state);
}

bool HBridgeController::config_mcpwm(state config_state) {
    if (config_state != COMPLIMENTARY && config_state != DRIVE) {
        return false;
    }
    mcpwm_gpio_init(this->MCPWM_UNIT, MCPWM0A, this->HS1);
    mcpwm_gpio_init(this->MCPWM_UNIT, MCPWM0B, this->LS1);
    mcpwm_gpio_init(this->MCPWM_UNIT, MCPWM1A, this->HS2);
    mcpwm_gpio_init(this->MCPWM_UNIT, MCPWM1B, this->LS2);

    this->set_freq(freq);
    this->cfg.counter_mode = MCPWM_UP_COUNTER;
    this->cfg.duty_mode = MCPWM_DUTY_MODE_0;

    // Pair 1 (HS1/LS1) on TIMER0
    this->cfg.cmpr_a = this->dir == REVERSE ? this->duty_cycle : 0;
    this->cfg.cmpr_b = this->dir == REVERSE ? this->duty_cycle : 0;
    mcpwm_init(this->MCPWM_UNIT, MCPWM_TIMER_0, &this->cfg);

    // Pair 2 (HS2/LS2) on TIMER1
    cfg.cmpr_a = this->dir == FORWARD ? this->duty_cycle : 0;
    cfg.cmpr_b = this->dir == FORWARD ? this->duty_cycle : 0;
    mcpwm_init(this->MCPWM_UNIT, MCPWM_TIMER_1, &cfg);

    if (config_state == COMPLIMENTARY) {
        // Deadtime for both timers
        const uint32_t dt_ticks = deadtime_ticks_from_ns(deadtime_ns);
        mcpwm_deadtime_enable(this->MCPWM_UNIT, MCPWM_TIMER_0,
                            MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                            dt_ticks, dt_ticks);
        mcpwm_deadtime_enable(this->MCPWM_UNIT, MCPWM_TIMER_1,
                            MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                            dt_ticks, dt_ticks);
    }
    else {
        mcpwm_deadtime_disable(this->MCPWM_UNIT, MCPWM_TIMER_0);
        mcpwm_deadtime_disable(this->MCPWM_UNIT, MCPWM_TIMER_1);
    }
    this->set_duty_cycle(duty_cycle);
    return true;
}

bool HBridgeController::set_state(state new_state) {
    this->current_state = new_state;
    switch (new_state) {
        case STOPPED:
            this->disable_mcpwm_outputs();
            this->attach_hw_pwm();
            // Briefly short motor terminals to dissipate inductive kickback,
            // then disconnect by driving all gates low.
            ledcWrite(this->ledc_ch_hs1, 0);
            ledcWrite(this->ledc_ch_hs2, 0);
            ledcWrite(this->ledc_ch_ls1, k_ledc_max_duty);
            ledcWrite(this->ledc_ch_ls2, k_ledc_max_duty);
            delay(k_stop_brake_ms);
            ledcWrite(this->ledc_ch_hs1, 0);
            ledcWrite(this->ledc_ch_hs2, 0);
            ledcWrite(this->ledc_ch_ls1, 0);
            ledcWrite(this->ledc_ch_ls2, 0);
            return true;
        case BODY_DIODE:
            this->disable_mcpwm_outputs();
            this->attach_hw_pwm();
            this->apply_hw_pwm_duty();
            return true;
        case COMPLIMENTARY:
            this->detach_hw_pwm();
            return this->config_mcpwm(new_state);
        case DRIVE:
            this->detach_hw_pwm();
            return this->config_mcpwm(new_state);
        default:
            return false;
    }
}


/*
* @brief Stops the H-Bridge by stopping the MCPWM timers.
* 
* @return `bool`: true if successful, false otherwise
*
*/
bool HBridgeController::stop() {
    return this->set_state(STOPPED);
}



/*
* @brief Determines the number of MCPWM deadtime ticks corresponding to the given
* nanoseconds.
* 
* @param `uint32_t` ns: nanoseconds of deadtime
* @return `uint32_t`: number of ticks per given number of nanoseconds
*
*/
uint32_t HBridgeController::deadtime_ticks_from_ns(uint32_t ns) {
  // Deadtime ticks are based on APB clock (usually 80 MHz on ESP32).
  const uint32_t apb_hz = esp_clk_apb_freq();
  // ticks = ns * apb_hz / 1e9 (rounded)
  uint64_t ticks = (uint64_t)ns * (uint64_t)apb_hz;
  ticks = (ticks + 500000000ULL) / 1000000000ULL;
  if (ticks > 1023) ticks = 1023; // MCPWM deadtime register width
  return (uint32_t)ticks;
}
