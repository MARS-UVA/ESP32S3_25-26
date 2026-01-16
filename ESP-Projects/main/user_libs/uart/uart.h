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

typedef struct __attribute__((packed))
{
  uint8_t invalid;
  uint8_t header;

  float top_left_wheel;
  float back_left_wheel;
  float top_right_wheel;
  float back_right_wheel;
  float bucket_left;
  float bucket_right;
  float left_actuator;
  float right_actuator;
} OutPacket;

/* ----------------------Variables -------------------*/

/* --------------------- Functions ------------------ */

void UART_setup();
void UART_read(SerialPacket *packet);
void UART_write(OutPacket *packet);
void UART_receive_wifi(SerialPacket *pkt);
void UART_callback(uint8_t reg, void (*callback)(SerialPacket, void *, void *), void *userdata1, void *userdata2);
void UART_rx_task();
void UART_tx_task();
void UART_can_task();

#endif