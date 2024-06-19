#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "../lib/MPU9250/mpu9250.h"
#include "../lib/ESC/esc.h"
#include "../lib/Filter/leaky_LP.h"
#include "../lib/Control/controller.h"
#include "../lib/Radio/radio.h"
#include "pico/cyw43_arch.h"
#include <math.h>
#include <time.h>

// Connection
// GPIO 4(pin 6)MISO / spi0_rx→ ADO on MPU9250 board
// GPIO 5(pin 7)Chip select → NCS on MPU9250 board
// GPIO 6(pin 9)SCK / spi0_sclk → SCL on MPU9250 board
// GPIO 7(pin 10)MOSI / spi0_tx → SDA on MPU9250 board
// IMU const
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7

// ESC const
#define PWM_WRAP 100000 // counts
#define PWM_FREQ 50     // hz
#define MAX_DUTY 0.1
#define MIN_DUTY 0.05

#define PIN_PWM1 21
#define PIN_PWM2 20
#define PIN_PWM3 19
#define PIN_PWM4 18

// uart0
#define UART_ID uart0
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

// RADIO
#define RADIO_TX 8
#define RADIO_RX 9

// controller
 #define Default_Kp 0.2
 #define Default_Ki 10
 #define Default_Kd 0.015
 


#define Default_yaw_Kp 0
#define Default_yaw_Ki 0
#define Default_yaw_Kd 0


// init throttle when armed, so it doesnt start weird
#define INIT_THROTTLE 15

static mpu9250 imu;
static ESC esc;
// Low pass
static leaky_lp w_filter;
static leaky_lp a_filter;
// controller
static controller fc;
// radio
static radio rdo;

// control cycle timing
#define dT_ms 20
#define dT_s dT_ms / 1000.0

volatile int buffer_index = 0;
volatile bool new_input = true;

void uart0_setup()
{
  // Set up our UART with a basic baud rate.
  uart_init(UART_ID, 115200);

  gpio_set_function(0, GPIO_FUNC_UART);
  gpio_set_function(1, GPIO_FUNC_UART);

  uart_set_baudrate(UART_ID, 115200);
  // Set UART flow control CTS/RTS, we don't want these, so turn them off
  uart_set_hw_flow(UART_ID, false, false);
  // Set our data format
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  // Turn off FIFO's - we want to do this character by character
  uart_set_fifo_enabled(UART_ID, false);
}

// kp=x, kd=y, in serial to set values
void PD_handler()
{
  static char buffer[32];
  static int buffer_index = 0;
  char ch;
  new_input = true;

  while (uart_is_readable(uart0))
  {
    // Read a single character
    ch = uart_getc(uart0);

    // Check for buffer overflow
    if (buffer_index < (sizeof(buffer) - 1))
    {
      // If the character is not a newline or carriage return, add it to the buffer
      if (ch != '\n' && ch != '\r')
      {
        buffer[buffer_index++] = ch;
      }
      else
      {
        buffer[buffer_index] = '\0'; // Null-terminate the string

        // Check if the input is for kp
        if (strncmp(buffer, "kp=", 3) == 0)
        {
          fc.Kp = atof(buffer + 3); // Convert the string to a number and assign to kp
        }

        // Check if the input is for ki
        if (strncmp(buffer, "ki=", 3) == 0)
        {
          fc.Ki = atof(buffer + 3); // Convert the string to a number and assign to kp
        }
        // Check if the input is for kd
        else if (strncmp(buffer, "kd=", 3) == 0)
        {
          fc.Kd = atof(buffer + 3); // Convert the string to a number and assign to kd
        }

        buffer_index = 0; // Reset the buffer index
      }
    }
    else
    {
      // Handle buffer overflow, for example, by clearing the buffer
      buffer_index = 0;
    }
  }
}
int main()

{
  // Serial
  stdio_init_all();

  // UART
  uart0_setup();
  // And set up and enable the interrupt handlers
  irq_set_exclusive_handler(UART0_IRQ, PD_handler);
  irq_set_enabled(UART0_IRQ, true);
  // Now enable the UART to send interrupts - RX only
  uart_set_irq_enables(UART_ID, true, false);

  // Need Wifi For the LED GPIO
  if (cyw43_arch_init())
  {
    printf("Wi-Fi init failed");
    return -1;
  }
  // Turn on the LED to show we are powered on
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  // RADIO
  radio_setup(&rdo, RADIO_RX, RADIO_TX);

  // IMU stuff
  mpu9250_setup(&imu, PIN_CS, PIN_MISO, PIN_SCK, PIN_MOSI);
  printf("Calibrating Gyro....Keep it Still...\n");
  gyro_cal(&imu, 50);
  acc_cal(&imu, 50);
  printf("Done. Offsets: w_x: %.5f, w_y:%.5f, w_z:%.5f \n", imu.w_offsets[0], imu.w_offsets[1], imu.w_offsets[2]);

  // ESC stuff
  uint32_t PWM_PIN[4] = {PIN_PWM1, PIN_PWM2, PIN_PWM3, PIN_PWM4};
  esc_setup(&esc, PWM_PIN, PWM_FREQ, PWM_WRAP, MIN_DUTY, MAX_DUTY);

  printf("Calibrating ESC...\n");
  cali_motor(&esc);
  printf("Done.\n");

  printf("Arming ESC...\n");
  arm_motor(&esc);
  printf("Done.\n");

  leaky_init(&w_filter, 0.9f);
  leaky_init(&a_filter, 0.9f);

  init_controller(&fc, 0.98f, dT_s, Default_Kp, Default_Ki, Default_Kd, Default_yaw_Kp, Default_yaw_Ki, Default_yaw_Kd);

  // blink to show we are ready
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  sleep_ms(100);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  sleep_ms(100);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

  while (1)
  {
    absolute_time_t startTime = get_absolute_time();
    // fetch new imu data
    mpu9250_update(&imu);
    float wf[3];
    float af[3];
    // low pass
    leaky_update(&w_filter, imu.w, wf);
    leaky_update(&a_filter, imu.a, af);

    // good for arduino serial plotter:
    //  printf("wx:%f, wy:%f, wz:%f, wxf:%f, wyf:%f, wzf:%f, ax:%f, ay:%f, az:%f, axf:%f, ayf:%f, azf:%f\n",
    //         imu.w[0], imu.w[1], imu.w[2],
    //         wf[0], wf[1], wf[2],
    //         imu.a[0], imu.a[1], imu.a[2],
    //         af[0], af[1], af[2]);

    update_controller(&fc, wf, af);

    
    // displays the current throttle
    if (new_input)
    {
      new_input = false;
      printf("roll:%.3f, pitch:%.3f, yaw:%.3f \n", fc.x.roll, fc.x.pitch, fc.x.yaw);
      printf("Kp = %.3f, Ki = %.3f, Kd = %.3f, throttle = %.3f , %s\n", fc.Kp, fc.Ki, fc.Kd, rdo.throttle, rdo.controller_armed ? "ARMed" : "NOT-ARMed");
    }

    if (rdo.controller_armed)
    {
      motor_control(&esc, INIT_THROTTLE + rdo.throttle + fc.u.t1, 1);
      motor_control(&esc, INIT_THROTTLE + rdo.throttle + fc.u.t2, 2);
      motor_control(&esc, INIT_THROTTLE + rdo.throttle + fc.u.t3, 3);
      motor_control(&esc, INIT_THROTTLE + rdo.throttle + fc.u.t4, 4);
      
    }
    else
    {
      motor_control(&esc, 0, 1);
      motor_control(&esc, 0, 2);
      motor_control(&esc, 0, 3);
      motor_control(&esc, 0, 4);
    }

    absolute_time_t endTime = get_absolute_time();

    int64_t time_diff_ms = absolute_time_diff_us(startTime, endTime) / 1000;
    // to achieve a fixed control loop time, if possible
    u_int32_t sleep_time_ms = dT_ms - time_diff_ms > 0 ? dT_ms - time_diff_ms : 0;
    sleep_ms(sleep_time_ms);
    // printf("cycle time: %lld ms, ", time_diff_ms <= dT_ms ? dT_ms : time_diff_ms);
    // print_control(fc.u);
  }
}