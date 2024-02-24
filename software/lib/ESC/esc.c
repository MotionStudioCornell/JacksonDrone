#include "esc.h"

void esc_setup(ESC *myesc, uint32_t PIN_PWM[], uint32_t PWM_FREQ, uint32_t PWM_WRAP, double MIN_DUTY_CYC, double MAX_DUTY_CYC)
{

  myesc->level_range[0] = (uint16_t)(MIN_DUTY_CYC * PWM_WRAP);
  myesc->level_range[1] = (uint16_t)(MAX_DUTY_CYC * PWM_WRAP);

  for (int i = 0; i < 4; i++)
  {
    myesc->PIN_PWM[i] = PIN_PWM[i];
    // Make it a PWM Pin
    gpio_set_function(PIN_PWM[i], GPIO_FUNC_PWM);
    // Find out which PWM slice is connected to the PWM pin
    uint slice_num = pwm_gpio_to_slice_num(PIN_PWM[i]);

    // printf("GPIO %d is on slice %d \n", PIN_PWM[i], slice_num);
    // freq should be 50 Hz for most of the ESCs, and it's true for our case
    pwm_set_clkdiv(slice_num, clock_get_hz(clk_sys) / (PWM_FREQ * PWM_WRAP));
    pwm_set_wrap(slice_num, PWM_WRAP);
    // give it an init value, does not matter(kinda)
    pwm_set_gpio_level(PIN_PWM[i], myesc->level_range[0]);
    // start the pwm
    pwm_set_enabled(slice_num, true);
  }
}

void cali_motor(ESC *myesc)
{
  //  MIN -> wait for 2 beeps

  for (int i = 0; i < 4; i++)
  {
    pwm_set_gpio_level(myesc->PIN_PWM[i], myesc->level_range[0]);
  }

  for (int i = 0; i < 4; i++)
  {
    pwm_set_gpio_level(myesc->PIN_PWM[i], myesc->level_range[1]);
  }

  sleep_ms(3000);

  for (int i = 0; i < 4; i++)
  {
    pwm_set_gpio_level(myesc->PIN_PWM[i], myesc->level_range[0]);
  }
  sleep_ms(3000);
}

void arm_motor(ESC *myesc)
{
  //  MIN -> wait for 2 beeps

  for (int i = 0; i < 4; i++)
  {
    pwm_set_gpio_level(myesc->PIN_PWM[i], myesc->level_range[0]);
  }

  sleep_ms(500);
}

void motor_control(ESC *myesc, double percent_throttle, uint MOTOR_NUM)
{

  // Ensure the throttle percentage is within bounds
  if (percent_throttle < 0.0)
    percent_throttle = 0.0;
  if (percent_throttle > 1.0)
    percent_throttle = 1.0;

  // Ensure MOTOR_NUM is within bounds
  if (MOTOR_NUM >= 4)
    return; // Assuming 4 motors

  // Calculate the duty cycle based on the throttle percentage
  uint16_t duty_cycle_level = (uint16_t)(myesc->level_range[0] + (myesc->level_range[1] - myesc->level_range[0]) * percent_throttle);

  // set to be Min+(Max-Min)*percent, so for min= 5% * wrap, max= 10% * wrap, an input of 50% throttle will be 5% + (10%-5%) * 50% = 7.5% * wrap
  pwm_set_gpio_level(myesc->PIN_PWM[MOTOR_NUM], duty_cycle_level);
}