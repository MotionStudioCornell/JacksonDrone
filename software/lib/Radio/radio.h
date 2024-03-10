#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"

#define RADIO_UART_ID uart1
#define BAUD_RATE 420000
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define CRSF_ADDRESS_CRSF_TRANSMITTER 0xEE
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEA
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC

typedef struct radio
{
  uint32_t PIN_RX;
  uint32_t PIN_TX;
  bool valid_type;
  volatile uint8_t radio_buffer[32];
  uint radio_buffer_index;

  float throttle;
  bool controller_armed;

} radio;


void radio_setup(radio *myradio, uint32_t pin_rx, uint32_t pin_tx);
void radio_irq_handler(); // ISR prototype