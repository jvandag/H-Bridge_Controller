#include "HBridgeController.h"
#include <stdexcept>
#include <Esp.h>

// assign default values and call start function to initialize MCPWM
HBridgeController::HBridgeController(uint8_t T1_pin, uint8_t T2_pin, uint8_t T3_pin, uint8_t T4_pin,
                                     uint32_t freq, float duty_cycle, mcpwm_unit_t MCPWM_UNIT,
                                     uint32_t deadtime_ns, direction start_direction)
: T1_pin(T1_pin), T2_pin(T2_pin), T3_pin(T3_pin), T4_pin(T4_pin), freq(freq), duty_cycle(duty_cycle), MCPWM_UNIT(MCPWM_UNIT), deadtime_ns(deadtime_ns), dir(start_direction) {
    this->start();
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
    if (this->deadtime_ns <= 0) {
        err_msg = "Deadtime cannot be less than or equal to zero";
        throw std::invalid_argument(err_msg.c_str());
    }
    const uint64_t max_freq = 1000000000ULL / (static_cast<uint64_t>(deadtime_ns) * 2ULL);
    if (freq >= max_freq) {
        err_msg = String("Frequency too high for the given deadtime. Max: ") + String(max_freq) + String(" Hz");
        throw std::invalid_argument(err_msg.c_str());
    }
    else if (freq < 0) {
        err_msg = "Frequency cannot be negative";
        throw std::invalid_argument(err_msg.c_str());
    }
    this->freq = freq;
    this->cfg.frequency = this->freq;
    mcpwm_set_frequency(this->MCPWM_UNIT, MCPWM_TIMER_0, freq);
    mcpwm_set_frequency(this->MCPWM_UNIT, MCPWM_TIMER_1, freq);

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
    if (this->deadtime_ns <= 0) {
        throw std::invalid_argument("Deadtime cannot be less than or equal to zero");
    }
    if (duty_cycle < 0 || duty_cycle > 100) {
        throw std::invalid_argument("Duty cycle must be between 0 and 100");
        return false;
    }
    this->duty_cycle = duty_cycle;
    
    // Update duty cycle on hardware PWM here

    // For forward direction we want to always apply a voltage to T2 while PWMing T3 and T4
    // For reverse direction we want to always apply a voltage to T4 while PWMing T1 and T2
    mcpwm_set_duty(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, this->dir == REVERSE ? this->duty_cycle : 0);
    mcpwm_set_duty_type(this->MCPWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

    mcpwm_set_duty(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, this->dir == FORWARD ? this->duty_cycle : 0);
    mcpwm_set_duty_type(this->MCPWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);

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


/*
* @brief initializes the MCPWM peripheral and configures the H-Bridge pins.
*
* @return `bool`: true if successful, false otherwise
*
*/
bool HBridgeController::start() {
    // Route MCPWM signals to the GPIO matrix
    mcpwm_gpio_init(this->MCPWM_UNIT, MCPWM0A, this->T1_pin);
    mcpwm_gpio_init(this->MCPWM_UNIT, MCPWM0B, this->T2_pin);
    mcpwm_gpio_init(this->MCPWM_UNIT, MCPWM1A, this->T3_pin);
    mcpwm_gpio_init(this->MCPWM_UNIT, MCPWM1B, this->T4_pin);

    
    this->set_freq(freq);
    this->cfg.counter_mode = MCPWM_UP_COUNTER;
    this->cfg.duty_mode = MCPWM_DUTY_MODE_0;

    // Pair 1 (T1/T2) on TIMER0
    this->cfg.cmpr_a = this->dir == REVERSE ? this->duty_cycle : 0;
    this->cfg.cmpr_b = this->dir == REVERSE ? this->duty_cycle : 0;
    mcpwm_init(this->MCPWM_UNIT, MCPWM_TIMER_0, &this->cfg);

    // Pair 2 (T3/T4) on TIMER1
    cfg.cmpr_a = this->dir == FORWARD ? this->duty_cycle : 0;
    cfg.cmpr_b = this->dir == FORWARD ? this->duty_cycle : 0;
    mcpwm_init(this->MCPWM_UNIT, MCPWM_TIMER_1, &cfg);

    // Deadtime for both timers
    const uint32_t dt_ticks = deadtime_ticks_from_ns(deadtime_ns);
    mcpwm_deadtime_enable(this->MCPWM_UNIT, MCPWM_TIMER_0,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                        dt_ticks, dt_ticks);
    mcpwm_deadtime_enable(this->MCPWM_UNIT, MCPWM_TIMER_1,
                        MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                        dt_ticks, dt_ticks);

    // Apply duty on A for each pair
    this->set_duty_cycle(duty_cycle);
    return true;
}


/*
* @brief Stops the H-Bridge by stopping the MCPWM timers.
* 
* @return `bool`: true if successful, false otherwise
*
*/
bool HBridgeController::stop() {
    mcpwm_stop(this->MCPWM_UNIT, MCPWM_TIMER_0);
    mcpwm_stop(this->MCPWM_UNIT, MCPWM_TIMER_1);
    return true;
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