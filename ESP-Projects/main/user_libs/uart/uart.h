#ifndef POWERLIB_UART
#define POWERLIB_UART

#include "driver/uart.h"
#include "driver/gpio.h"
#include "../can/can.h"


/* --------------------- Types ---------------------- */

typedef struct serialPacket {
  
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
int uart_priority;

/* --------------------- Functions ------------------ */

void UART_setup();
SerialPacket UART_read();
void UART_write();
void UART_callback(uint8_t reg, void (*callback)(SerialPacket, void*, void*), void* userdata1, void* userdata2);
void uart_event_task();

#endif