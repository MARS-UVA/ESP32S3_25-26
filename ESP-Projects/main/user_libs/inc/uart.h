#pragma once

#include "driver/uart.h"
#include "utils.h"
#include "packets.h"

#define S3_TX_PIN 43
#define S3_RX_PIN 44

/* --------------------- Functions ------------------ */

/**
 * @brief Initializes the UART peripheral and creates queues for inter-task communication.
 */
void UART_setup();

/**
 * @brief Reads a control packet from UART and sends it to the control queue if valid.
 * 
 * @param packet    Pointer to a ControlPacket_OneRobot struct where the received data will be stored.
 */
void UART_read(ControlPacket_OneRobot *packet);

/**
 * @brief Writes a packet of any type to the Jetson on UART.
 * 
 * @param packet    Pointer to the packet to be sent.
 * @param size      Size of the packet in bytes.
 */
void UART_write(void *packet, size_t size);
