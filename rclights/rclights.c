// ********************************************************************************
// Documented in blog post
// http://ruslanledesma.com/2025/01/26/raspberry-pico-as-rc-lights-controller.html
// ********************************************************************************

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// ********************************************************************************
// Measuring of input PWM

const uint INPUT_PIN = 27;
const uint INPUT_SLICE = 5;

const float SYS_CLK_FREQ = 125000000; // 125 MHz
const float INPUT_PWM_SYS_CLK_DIV = 125; // this value divides the system clock frequency to slow down the PWM measuring slice/component, a value of 125 slows down the PWM slice to 1 MHz or 1 PWM measure cycle per microsecond.

const float INPUT_PWM_FREQ = 62; // for example, a value of 50 corresponds to 50 Hz or 1 period/frame every 20 miliseconds
const float INPUT_PWM_PERIOD_MS = 1000 / INPUT_PWM_FREQ;
const float INPUT_PWM_COUNTER_UNITS_PER_SEC = 1000000; // each counter unit corresponds to 1 microsecond
const uint16_t INPUT_PWM_COUNTER_MAX = INPUT_PWM_COUNTER_UNITS_PER_SEC / INPUT_PWM_FREQ; // aka counter TOP or WRAP in Pico's docs, the amount of microseconds in one period/frame of the input signal

#define INPUT_PWM_AVG_SAMPLES 5

#define INPUT_PWM_SMOOTH_SAMPLES 4

uint init_pwm_measuring() {
  // Sanity checks
  assert(clock_get_hz(clk_sys)            == SYS_CLK_FREQ);
  assert(pwm_gpio_to_channel(INPUT_PIN)   == PWM_CHAN_B);
  assert(pwm_gpio_to_slice_num(INPUT_PIN) == INPUT_SLICE);

  gpio_set_function(INPUT_PIN, GPIO_FUNC_PWM);
  pwm_set_clkdiv_mode(INPUT_SLICE, PWM_DIV_B_HIGH);
  pwm_set_clkdiv(INPUT_SLICE, INPUT_PWM_SYS_CLK_DIV);
  pwm_set_wrap(INPUT_SLICE, INPUT_PWM_COUNTER_MAX); 
}

float measure_input_pwm_hi_us() {
  pwm_set_counter(INPUT_SLICE, 0);
  pwm_set_enabled(INPUT_SLICE, true);
  sleep_ms(INPUT_PWM_PERIOD_MS);
  pwm_set_enabled(INPUT_SLICE, false);

  float hi_us = pwm_get_counter(INPUT_SLICE);

  /* printf("hi_us = %f\n", hi_us); */

  return hi_us;
}

/* The following function returns the average of the last
   INPUT_PWM_AVG_SAMPLES input values.  This is one way to workaround
   noise in the input signal.  It trades off read cycles for a
   sequence of values that is less bumpy but might be away from most
   of the input values if peaks are too big. */
float average_input_pwm_hi_us() {
  static uint8_t curr_sample = 0;
  static float samples[INPUT_PWM_AVG_SAMPLES + 1];
  static float avg_hi_us = 0;

  uint8_t oldest_sample = (curr_sample + 1) % (INPUT_PWM_AVG_SAMPLES + 1);

  samples[curr_sample] = measure_input_pwm_hi_us() / INPUT_PWM_AVG_SAMPLES;
  avg_hi_us += samples[curr_sample] - samples[oldest_sample];
  curr_sample = oldest_sample;

  return avg_hi_us;
}

/* The following function returns the last smooth value until enough
   input PWM values are the same.  INPUT_PWM_SMOOT_SAMPLES defines
   many is enough.  This is one way to workaround noise in the input
   signal.  It trades off read cycles for a squence of values that
   ignores peaks but might not follow the average of the input signal
   if there is too much noise.  */
float smooth_input_pwm_hi_us() {
  static float smooth_hi_us = 0;
  static uint8_t curr_sample = 0;
  static float samples[INPUT_PWM_SMOOTH_SAMPLES];

  /* You might want to experiment reading values directly or reading
     an averaged value on the next line. */
  float avg_hi_us = measure_input_pwm_hi_us(); // average_input_pwm_hi_us();
  samples[curr_sample] = avg_hi_us;

  /* printf("avg_hi_us = %f\n", avg_hi_us); */

  if (avg_hi_us != smooth_hi_us) {
    bool all_equal = true;

    for (int i = 0; i < INPUT_PWM_SMOOTH_SAMPLES; i++) {
      all_equal = all_equal && samples[i] == avg_hi_us;
    }

    if (all_equal) {
      smooth_hi_us = avg_hi_us;
    }
  }

  curr_sample = (curr_sample + 1) % INPUT_PWM_SMOOTH_SAMPLES;

  return smooth_hi_us;
}  


// ********************************************************************************
// Data structure Led and corresponding operations that control leds
// connected to GPIO pins.

const uint16_t OUTPUT_PWM_MAX_LEVEL = 100;
const uint16_t OUTPUT_PWM_ON_LEVEL = 20;
const uint16_t OUTPUT_PWM_OFF_LEVEL = 0;
const uint16_t OUTPUT_PWM_HI_LEVEL = 100;

const uint32_t BLINK_INTERVAL_US = 400000;

enum LedState {
  OFF,
  ON,
  HI,
};

struct Led {
  const uint id;
  uint pwm_slice;
  enum LedState state;
};

struct Led FRONT_WHITE = {
  .id = 17,
  .pwm_slice = -1,
  .state = OFF,
};
struct Led FRONT_BLUE = {
  .id = 18,
  .pwm_slice = -1,
  .state = OFF,
};
struct Led LEFT_BLINKERS = {
  .id = 20,
  .pwm_slice = -1,
  .state = OFF,
};
struct Led RIGHT_BLINKERS = {
  .id = 21,
  .pwm_slice = -1,
  .state = OFF,
};
struct Led STOP = {
  .id = 22,
  .pwm_slice = -1,
  .state = OFF,
};
struct Led REVERSE = {
  .id = 28,
  .pwm_slice = -1,
  .state = OFF,
};

void init_led(struct Led* led) {
  led->pwm_slice = pwm_gpio_to_slice_num(led->id);

  gpio_set_function(led->id, GPIO_FUNC_PWM);
  pwm_set_wrap(led->pwm_slice, OUTPUT_PWM_MAX_LEVEL);
  pwm_set_enabled(led->pwm_slice, true);
}

void init_leds() {
  init_led(&FRONT_WHITE);
  init_led(&FRONT_BLUE);
  init_led(&LEFT_BLINKERS);
  init_led(&RIGHT_BLINKERS);
  init_led(&STOP);
  init_led(&REVERSE);
}

void turn_led_off(struct Led* led) {
  if (led->state == OFF) {
    return;
  }

  pwm_set_gpio_level(led->id, OUTPUT_PWM_OFF_LEVEL);
  led->state = OFF;
}

void turn_led_on(struct Led* led) {
  if (led->state == ON) {
    return;
  }

  pwm_set_gpio_level(led->id, OUTPUT_PWM_ON_LEVEL);
  led->state = ON;
}

void turn_led_hi(struct Led* led) {
  if (led->state == HI) {
    return;
  }

  pwm_set_gpio_level(led->id, OUTPUT_PWM_HI_LEVEL);
  led->state = HI;
}

void blink_led(struct Led* led) {
  static uint32_t next_us = 0;
  static bool blink_on = false;

  const uint32_t curr_us = time_us_32();

  if (next_us < curr_us) {
    next_us = curr_us + BLINK_INTERVAL_US;
    blink_on = !blink_on;
  }

  if (blink_on) {
    turn_led_on(led);
  } else if (led->state == ON && !blink_on) {
    turn_led_off(led);
  }
}

// ********************************************************************************
// Conversion of input PWM to master light states

const float INPUT_PWM_US_RANGE_MIN = 1019; // You might need to adjust these to match the MIN microseconds duty cycle for your transmitter/receiver combination
const float INPUT_PWM_US_RANGE_MAX = 1981; // Similar warning as that of INPUT_PWM_US_RANGE_MIN
const float INPUT_PWM_US_RANGE_SIZE = INPUT_PWM_US_RANGE_MAX - INPUT_PWM_US_RANGE_MIN + 1;
const float MASTER_LIGHT_STATE_COUNT = 48;
const float INPUT_PWM_US_BUCKET_SIZE = INPUT_PWM_US_RANGE_SIZE / (MASTER_LIGHT_STATE_COUNT - 1);

uint8_t input_pwm_hi_us_to_master_state_id(float hi_us) {
  /* ******************************************************************************** */
  /* Iterative way for debugging purposes.*/
  /* uint8_t state_id = 0; */
  /* float current_threshold = INPUT_PWM_US_RANGE_MIN + INPUT_PWM_US_BUCKET_SIZE / 2; */

  /* while(current_threshold < hi_us) { */
  /*   state_id++; */
  /*   current_threshold += INPUT_PWM_US_BUCKET_SIZE; */
  /* } */

  /* printf("current_threshold = %f\n", current_threshold); */

  /* return state_id; */
  /* ******************************************************************************** */

  return (hi_us - INPUT_PWM_US_RANGE_MIN + INPUT_PWM_US_BUCKET_SIZE / 2) / INPUT_PWM_US_BUCKET_SIZE;
}

uint8_t input_pwm_hi_us_to_master_lights_state(float hi_us) {
  uint8_t state_id = input_pwm_hi_us_to_master_state_id(hi_us);

  /* printf("state_id = %d\n", state_id); */

  uint8_t state = (state_id % 3) + ((state_id / 3) << 2);

  return state;
}

// ********************************************************************************
// Application of master light states to light sets.
//
// Light sets introduced in the blog post do not correspond to a
// concrete data structure, rather they are modelled by the
// application of their corresponding rules in this section.

void apply_front_white_light_rules(uint8_t day_night_state, uint8_t hi_beams_state) {
  if (hi_beams_state) {
    turn_led_hi(&FRONT_WHITE);
    return;
  }

  if (day_night_state) {
    turn_led_on(&FRONT_WHITE);
  } else {
    turn_led_off(&FRONT_WHITE);
  }
}

void apply_blink_light_state(uint8_t state) {
  switch(state) {
  case 0: // off
    turn_led_off(&LEFT_BLINKERS);
    turn_led_off(&RIGHT_BLINKERS);
    break;
  case 1: // left
    blink_led(&LEFT_BLINKERS);
    turn_led_off(&RIGHT_BLINKERS);
    break;
  case 2: // right
    turn_led_off(&LEFT_BLINKERS);
    blink_led(&RIGHT_BLINKERS);
    break;
  default: // hazard
    blink_led(&LEFT_BLINKERS);
    blink_led(&RIGHT_BLINKERS);
  }
}

void apply_reverse_light_state(uint8_t state) {
  if (state) {
    turn_led_on(&REVERSE);
  } else {
    turn_led_off(&REVERSE);
  }
}

void apply_stop_light_rules(uint8_t day_night_state, uint8_t brake_light_state) {
  if (brake_light_state) {
    turn_led_hi(&STOP);
    return;
  }

  if (day_night_state) {
    turn_led_on(&STOP);
  } else {
    turn_led_off(&STOP);
  }
}

void apply_master_lights_state(uint8_t state) {
  uint8_t brake_light_state = state & 1;
  uint8_t reverse_light_state = (state >> 1) & 1;
  uint8_t blink_light_state = (state >> 2) & 3;
  uint8_t hi_beams_state = (state >> 4) & 1;
  uint8_t day_night_state = (state >> 5) & 1;

  apply_stop_light_rules(day_night_state, brake_light_state);
  apply_reverse_light_state(reverse_light_state);
  apply_blink_light_state(blink_light_state);
  apply_front_white_light_rules(day_night_state, hi_beams_state);
}

// ********************************************************************************
// Program entry point

int main() {
  /* stdio_init_all(); */

  init_pwm_measuring();
  init_leds();

  turn_led_on(&FRONT_BLUE);

  float input_pwm_hi_us = 0;
  uint8_t master_lights_state = 0;

  while(true) {
    input_pwm_hi_us = smooth_input_pwm_hi_us();

    /* printf("input_pwm_hi_us = %f\n", input_pwm_hi_us); */

    master_lights_state = input_pwm_hi_us_to_master_lights_state(input_pwm_hi_us);

    /* printf("master_lights_state = %b\n", master_lights_state); */

    apply_master_lights_state(master_lights_state);
  }
}
