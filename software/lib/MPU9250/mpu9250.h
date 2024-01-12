#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"

#define SPI_PORT spi0
#define READ_BIT 0x80

typedef struct mpu9250
{
  int16_t w_raw[3];
  int16_t a_raw[3];
  int16_t temperature_raw;

  double w_offsets[3];
  double a_offsets[3];

  double w[3];
  double a[3];
  double temperature;

  int PIN_CS;

} mpu9250;

void mpu9250_setup(mpu9250 *imu, uint32_t PIN_CS, uint32_t PIN_MISO, uint32_t PIN_SCK, uint32_t PIN_MOSI);
// buffer_size: size of the data buffer for data collection during cal
void gyro_cal(mpu9250 *imu, int buffer_size);
void mpu9250_update(mpu9250 *imu);