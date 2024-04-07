#include "radio.h"

// Global instance of `radio` if needed by the ISR
static radio *global_radio = NULL;

void radio_setup(radio *myradio, uint32_t pin_rx, uint32_t pin_tx)
{
  myradio->valid_type = false;
  myradio->radio_buffer_index = 0;
  myradio->throttle = 0;
  myradio->controller_armed = false;
  myradio->PIN_TX = pin_tx;
  myradio->PIN_RX = pin_rx;
  // Set up UART
  uart_init(RADIO_UART_ID, BAUD_RATE);
  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(myradio->PIN_TX, GPIO_FUNC_UART);
  gpio_set_function(myradio->PIN_RX, GPIO_FUNC_UART);
  uart_set_irq_enables(RADIO_UART_ID, true, false);

  uart_set_hw_flow(RADIO_UART_ID, false, false);
  uart_set_format(RADIO_UART_ID, DATA_BITS, STOP_BITS, PARITY);
  uart_set_fifo_enabled(RADIO_UART_ID, false);

  // Set the IRQ handler
  irq_set_exclusive_handler(UART1_IRQ, radio_irq_handler);
  irq_set_enabled(UART1_IRQ, true);

  global_radio = myradio; // Set the global instance for ISR access
}

void radio_irq_handler()
{

  while (uart_is_readable(RADIO_UART_ID))
  {
    int character = uart_getc(RADIO_UART_ID);

    if (character == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
      global_radio->valid_type = true;
    }

    if (global_radio->valid_type)
    {
      if (global_radio->radio_buffer_index < sizeof(global_radio->radio_buffer))
      {
        global_radio->radio_buffer[global_radio->radio_buffer_index] = character;
        global_radio->radio_buffer_index++;
      }
      else
      {
        for (int i = 0; i < 16; i++)
        {
          printf("[%02X, %d], ", global_radio->radio_buffer[i], global_radio->radio_buffer[i]); // Print each byte as a 2-digit hexadecimal number
        }
        printf("\n"); // Print a newline at the end
        //45-255 C0, 0-196 C1 14_arm

        // lower half of the left joystick
        if (global_radio->radio_buffer[7] == 0xC0)
        {
          global_radio->throttle = ((global_radio->radio_buffer[6] - 45.0) / (255.0 - 45.0) * 0.5)*100.0;
        }
        // higher half of the left joystick
        if (global_radio->radio_buffer[7] == 0xC1)
        {
          global_radio->throttle = (global_radio->radio_buffer[6] / 196.0 * 0.5 + 0.5)*100.0;
        }
        // if already armed
        // the SE button 0x00 if pressed

        if (global_radio->radio_buffer[14] == 0xBF)
        {
          global_radio->controller_armed = false;
        }
        else if (global_radio->throttle == 0.0 && global_radio->radio_buffer[14] == 0x00)
        {
          global_radio->controller_armed = true;
        }

        global_radio->valid_type = false;
        global_radio->radio_buffer_index = 0;
      }
    }
  }
}