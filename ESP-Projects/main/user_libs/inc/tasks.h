/*
 * @file marsRTOS.h
 *
 * @author Carlos Giron
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "packets.h"

#include "can.h"
#include "uart.h"
#include "wifi.h"
#include "talonFX.h"
#include "talonSRX.h"
#include "pdh.h"
#include "OneRobot.h"



extern QueueHandle_t control_queue; // queue stores the control packet values
extern QueueHandle_t temperature_queue; // queue stores the temperature packet values
extern TalonFX *fxMotors[];
extern TalonSRX *srxMotors[];
extern Actuator frontActuator;
extern Actuator backActuator;

void current_update_task(PDH *pdh);
void UART_rx_task();
void UART_tx_task(PDH *pdh);
void one_robot_control_can_task();
void enable_task();
void motor_task();

/**
 * @brief Task that updates the temperature queue with the latest temperature readings from the robot.
 */
void temperature_update_task();