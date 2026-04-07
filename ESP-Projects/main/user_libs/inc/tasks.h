/*
 * @file tasks.h
 *
 * @brief Task definitions for the OneRobot control system.
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
extern QueueHandle_t current_voltage_queue; // queue stores the current/voltage packet values
extern QueueHandle_t position_queue; // queue stores the position packet values
extern TalonFX *fxMotors[];
extern TalonSRX *srxMotors[];
extern Actuator frontActuator;
extern Actuator backActuator;

void UART_rx_task();
void UART_tx_task();
void one_robot_control_can_task();
void motor_task();

/**
 * @brief Task that updates the temperature queue with the latest temperature readings from the robot.
 */
void temperature_update_task();

/**
 * @brief Task that updates the current/voltage queue with the latest readings from the robot.
 */
void current_voltage_update_task(PDH *pdh);

/**
 * @brief Task that updates the position queue with the latest position readings from the robot.
 */
void position_update_task();

void CAN_enable_task();