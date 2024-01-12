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
#include "pico/cyw43_arch.h"

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

// radio
#define RADIO_UART_ID uart1
#define BAUD_RATE 420000
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define CRSF_ADDRESS_CRSF_TRANSMITTER 0xEE
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC

static mpu9250 imu;
static ESC esc;



void uart0_setup()
{
  // Set up our UART with the required speed.
  uart_init(uart0, 115200);

  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(0, GPIO_FUNC_UART);
  gpio_set_function(1, GPIO_FUNC_UART);

  // Enable UART interrupt when RX data is available
  uart_set_irq_enables(uart0, true, false);
}

int buffer_index = 0;
double throttle = 0;

void uart0_irq_handler()
{
  char buffer[10];

  while (uart_is_readable(uart0))
  {
    // Read a single character
    char ch = uart_getc(uart0);

    // If the character is a digit, add it to the buffer
    if (ch >= '0' && ch <= '9')
    {
      buffer[buffer_index++] = ch;
    }
    // If the character is a newline or carriage return, convert the buffer to a number
    else if (ch == '\n' || ch == '\r')
    {
      buffer[buffer_index] = '\0';  // Null-terminate the string
      throttle = atoi(buffer) / 100.0; // Convert the string to a number
      buffer_index = 0;             // Reset the buffer index
    }
  }
}

void radio_setup()
{
  // Set up UART
  uart_init(RADIO_UART_ID, BAUD_RATE);
  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(8, GPIO_FUNC_UART);
  gpio_set_function(9, GPIO_FUNC_UART);

  uart_set_irq_enables(RADIO_UART_ID, true, false);

  uart_set_hw_flow(RADIO_UART_ID, false, false);
  uart_set_format(RADIO_UART_ID, DATA_BITS, STOP_BITS, PARITY);
  uart_set_fifo_enabled(RADIO_UART_ID, false);
}



// void decode_crsf(uint8_t *data, uint length)
// {
//   // CRSF frame has a minimum length of 4 bytes (Address, Length, Type, CRC)
//   if (length < 4)
//   {
//     printf("Frame too short\n");
//     return;
//   }

// }
bool valid_type = false;
uint8_t radio_buffer[32];
uint radio_buffer_index = 0;

void radio_irq_handler()
{

  while (uart_is_readable(RADIO_UART_ID))
  {
    int character = uart_getc(RADIO_UART_ID);

    if (character == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
      valid_type = true;
    }

    if (valid_type)
    {
      if(radio_buffer_index<sizeof(radio_buffer)){
          radio_buffer[radio_buffer_index] = character;
          radio_buffer_index++;
      }else{
          // for (int i = 5; i < 5+5; i++)
          // {
          //   printf("[%02X, %d], ", radio_buffer[i], radio_buffer[i]); // Print each byte as a 2-digit hexadecimal number
          // }
          // printf("\n"); // Print a newline at the end

          if (radio_buffer[7]==0xC1){
            printf("%f \n",radio_buffer[6]/196.0);
            throttle = radio_buffer[6] / 196.0;
          }


          valid_type = false;
          radio_buffer_index = 0;
      }
      
    }
  }
}

int main()
{
  // Serial
  stdio_init_all();

  // UART
  uart0_setup();
  // Set the IRQ handler
  irq_set_exclusive_handler(UART0_IRQ, uart0_irq_handler);
  irq_set_enabled(UART0_IRQ, true);

  //RADIO
  radio_setup();
  // Set the IRQ handler
  irq_set_exclusive_handler(UART1_IRQ, radio_irq_handler);
  irq_set_enabled(UART1_IRQ, true);

  // Need Wifi For the LED GPIO
  if (cyw43_arch_init())
  {
    printf("Wi-Fi init failed");
    return -1;
  }
  // Turn on the LED to show we are powered on
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

  // IMU stuff
  mpu9250_setup(&imu, PIN_CS, PIN_MISO, PIN_SCK, PIN_MOSI);
  printf("Calibrating Gyro....Keep it Still...\n");
  gyro_cal(&imu, 50);
  printf("Done. Offsets: w_x: %.5f, w_y:%.5f, w_z:%.5f \n", imu.w_offsets[0], imu.w_offsets[1], imu.w_offsets[2]);

  // ESC stuff
  esc_setup(&esc, PIN_PWM1, PWM_FREQ, PWM_WRAP, MIN_DUTY, MAX_DUTY);
  printf("Arming ESC...\n");
  arm_motor(&esc);
  printf("Done.");

  // blink to show we are ready
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  sleep_ms(100);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  sleep_ms(100);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
  sleep_ms(100);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
  sleep_ms(100);
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

  while (1)
  {
    mpu9250_update(&imu);
    // printf("Gyro: X = %10.5f, Y = %10.5f, Z = %10.5f (dps) | Acc: X = %7.5f, Y = %7.5f, Z = %7.5f (g) | Temp = %4.2f degC \n", imu.w[0], imu.w[1], imu.w[2], imu.a[0], imu.a[1], imu.a[2], imu.temperature);
    motor_control(&esc, throttle);

    //   int i =0;
    //   while(i< 50 ){
    //     motor_control(&esc, 0.01 * i);
    //     i++;
    //     sleep_ms(100);
    //   }

    //   while (i > 0)
    //   {
    //     motor_control(&esc, 0.01 * i);
    //     i--;
    //     sleep_ms(100);
    //   }
  }
}