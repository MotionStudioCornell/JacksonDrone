#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

typedef struct ESC
{
  //[min,max] : should be [0.05*PWM_WARP, 0.1*PWM_WARP] with freq 50Hz
  uint16_t level_range[2];
  uint32_t PIN_PWM[4];

} ESC;

void esc_setup(ESC *myesc, uint32_t *PIN_PWM, uint32_t PWM_FREQ, uint32_t PWM_WRAP, double MIN_DUTY_CYC, double MAX_DUTY_CYC);

//calibrate the ESC so it know the min and max throttle
void cali_motor(ESC *myesc);
// arm the ESC so it can starts
void arm_motor(ESC *myesc);

// percent_throttle: [0,1], 0 for no throttle, 1 for max throttle
void motor_control(ESC *myesc, double percent_throttle, uint MOTOR_NUM);