#ifndef POWERLIB_UART
#define POWERLIB_UART

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

#include "utils.h"
/* --------------------- Types ---------------------- */

typedef struct
{
  uint8_t invalid;
  uint8_t header;
  uint8_t top_left_wheel;
  uint8_t back_left_wheel;
  uint8_t top_right_wheel;
  uint8_t back_right_wheel;
  uint8_t drum;
  uint8_t actuator;
} SerialPacket;

/* ----------------------Variables -------------------*/

/* --------------------- Functions ------------------ */

void UART_setup();
SerialPacket UART_read();
void UART_write();
void UART_callback(uint8_t reg, void (*callback)(SerialPacket, void *, void *), void *userdata1, void *userdata2);
void UART_rx_task();
void UART_can_task();

#endif