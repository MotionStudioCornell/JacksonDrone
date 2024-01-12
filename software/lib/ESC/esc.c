#include "esc.h"

void esc_setup(ESC *myesc, uint32_t PIN_PWM1, uint32_t PWM_FREQ, uint32_t PWM_WRAP, double MIN_DUTY_CYC, double MAX_DUTY_CYC)
{
  myesc->PIN_PWM1=PIN_PWM1;
  myesc->level_range[0] = (uint16_t)(MIN_DUTY_CYC * PWM_WRAP);
  myesc->level_range[1] = (uint16_t)(MAX_DUTY_CYC * PWM_WRAP);
  //Make it a PWM Pin
  gpio_set_function(PIN_PWM1, GPIO_FUNC_PWM);
  // Find out which PWM slice is connected to the PWM pin
  uint slice_num = pwm_gpio_to_slice_num(PIN_PWM1);
  //freq should be 50 Hz for most of the ESCs, and it's true for our case
  pwm_set_clkdiv(slice_num, clock_get_hz(clk_sys) / (PWM_FREQ * PWM_WRAP));
  pwm_set_wrap(slice_num, PWM_WRAP);
  //give it an init value, does not matter(kinda)
  pwm_set_gpio_level(PIN_PWM1, myesc->level_range[0]);
  //start the pwm
  pwm_set_enabled(slice_num, true);
}

void arm_motor(ESC *myesc){
  //MAX -> wait for beep -> MIN -> wait for beep
  pwm_set_gpio_level(myesc->PIN_PWM1, myesc->level_range[1]);
  sleep_ms(500);
  pwm_set_gpio_level(myesc->PIN_PWM1, myesc->level_range[0]);
  sleep_ms(500);
}

void motor_control(ESC *myesc, double percent_throttle)
{
  //set to be Min+(Max-Min)*percent, so for min= 5% * wrap, max= 10% * wrap, an input of 50% throttle will be 5% + (10%-5%) * 50% = 7.5% * wrap
  pwm_set_gpio_level(myesc->PIN_PWM1, myesc->level_range[0] + (myesc->level_range[1] - myesc->level_range[0])*percent_throttle);
}