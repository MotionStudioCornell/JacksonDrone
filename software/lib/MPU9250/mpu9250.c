#include "mpu9250.h"
#define SPI_PORT spi0
#define READ_BIT 0x80

static inline void cs_select(mpu9250 *imu)
{
  asm volatile("nop \n nop \n nop");
  gpio_put(imu->PIN_CS, 0); // Active low
  asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(mpu9250 *imu)
{
  asm volatile("nop \n nop \n nop");
  gpio_put(imu->PIN_CS, 1);
  asm volatile("nop \n nop \n nop");
}

void gyro_cal(mpu9250 *imu, int buffer_size){

  // Buffer
  double gyro_buffer[buffer_size][3];

  int buffer_i = 0;
  while (buffer_i < buffer_size)
  {
    mpu9250_update(imu);
    gyro_buffer[buffer_i][0] = imu->w[0];
    gyro_buffer[buffer_i][1] = imu->w[1];
    gyro_buffer[buffer_i][2] = imu->w[2];
    buffer_i++;
  }

  // Calculate gyro offsets
  for (int i = 0; i < 3; i++)
  {
    double gyro_sum = 0.0;

    for (int j = 0; j < buffer_size; j++)
    {
      gyro_sum += gyro_buffer[j][i];
    }
    imu->w_offsets[i] = gyro_sum / buffer_size;
  }

}

void mpu9250_setup(mpu9250 *imu, uint32_t PIN_CS, uint32_t PIN_MISO, uint32_t PIN_SCK, uint32_t PIN_MOSI)
{
  // SPI STUFF
  spi_init(SPI_PORT, 100 * 1000);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  // CS is active-low, turn it on by setting it high
  gpio_init(PIN_CS);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_put(PIN_CS, 1);
  
  // Two byte reset. First byte register, second byte data
  // There are a load more options to set up the device in different ways that could be added here
  //assign CS Pin
  imu->PIN_CS = PIN_CS;
  //reset write to reg addr 0x6B with data 0x01
  uint8_t buf[] = {0x6B, 0x01};
  cs_select(imu);
  spi_write_blocking(SPI_PORT, buf, 2);
  cs_deselect(imu);
  }

static void read_registers(mpu9250 *imu, uint8_t reg, uint8_t *buf, uint16_t len)
{
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto incrementing
  // so we don't need to keep sending the register we want, just the first.

  reg |= READ_BIT;
  cs_select(imu);
  spi_write_blocking(SPI_PORT, &reg, 1);
  sleep_us(10);

  spi_read_blocking(SPI_PORT, 0, buf, len);
  cs_deselect(imu);
  sleep_us(10);
}

void convert(mpu9250 *imu){
  //from datasheet:
  // gyro 65.5, 32.8, 16.4 for +-500,1000,2000 dps
  // acc 8,192, 4,096, 2,048 for +-4,8,16 g
  // assume gyro +-250dps, acc+-2g
  double gyro_sen = 131;
  double acc_sen = 16384;

  for (int i = 0; i < 3; i++)
  {
    imu->w[i] = imu->w_raw[i]/ gyro_sen;
    imu->a[i] = imu->a_raw[i] / acc_sen;
  }
  // TEMP_degC  = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
  // magic number for temp sen, cant find it in datasheet, got it from: https://github.com/wollewald/MPU9250_WE/blob/main/src/MPU6500_WE.h
  imu->temperature = imu->temperature_raw / 333.87 + 21;
}

void apply_offset(mpu9250 *imu)
{
  for (int i = 0; i < 3; i++)
  {
    imu->w[i] -=imu->w_offsets[i];
  }
}

void mpu9250_update(mpu9250 *imu)
{
  uint8_t buffer[6];
  // Start reading acceleration registers from register 0x3B for 6 bytes
  read_registers(imu, 0x3B, buffer, 6);
  for (int i = 0; i < 3; i++)
  {
    imu->a_raw[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
  }

  // Now gyro data from reg 0x43 for 6 bytes
  read_registers(imu, 0x43, buffer, 6);
  for (int i = 0; i < 3; i++)
  {
    imu->w_raw[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
  }

  // Now temperature from reg 0x41 for 2 bytes
  read_registers(imu, 0x41, buffer, 2);
  imu->temperature_raw = buffer[0] << 8 | buffer[1];

  //convert from raw
  convert(imu);
  apply_offset(imu);
}