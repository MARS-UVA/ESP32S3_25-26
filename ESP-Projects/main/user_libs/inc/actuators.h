/**
 * @file actuators.h
 * @brief Header file for actuator control functions.
 * 
 * This module provides functions to control and synchronize two actuators using
 * PID control, as well as functions to read their positions using hall effec
 * sensors. It includes initialization functions for the actuators and the hall
 * effect sensors, as well as functions to move the actuators to a target
 * position or velocity while keeping them synchronized.
 * 
 * @author Jingyi Li
 * @date 2024-06-01
 * @version 1.0
 */

#pragma once

#include <math.h>

#include "pid.h"
#include "can.h"
#include "adc.h"
#include "pid.h"
#include "packets.h"
#include "talonSRX.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "utils.h"

/**
 * @struct Actuator
 * @brief Structure for an actuator with its associated components and state.
 * 
 * This structure contains a pointer to a TalonSRX motor controller, a
 * potentiometer structure for position feedback, a PID controller for control,
 * and variables to keep track of the previous position, velocity, and time for
 * control calculations. It also includes a pointer to an integer representing
 * the direction of the actuator, which can be used to invert the control output 
 * if necessary.
 */
typedef struct
{
    TalonSRX *controller;
    Pot *pot;
    PIDController *pid;
    double prevPosition;
    double prevVelocity;
    double lastTime;
    double velocity;
    int *direction;
} Actuator;

/**
 * @brief Initializes an Actuator structure with the given components.
 * 
 * This function takes pointers to a TalonSRX motor controller, a potentiometer,
 * a PID controller and an integer for direction, and initializes an Actuator
 * structure with these components. It also sets the initial previous position
 * to 1, previous velocity to 0, last time to -1 (indicating that it has not
 * been updated yet), and velocity to 0.
 * 
 * @param[in] talonSrx Pointer to the TalonSRX motor controller associated with the actuator.
 * @param[in] pot Pointer to the potentiometer structure for position feedback.
 * @param[in] pid Pointer to the PID controller for controlling the actuator.
 * @param[in] direction Pointer to an integer representing the direction of the actuator (1 or -1).
 * 
 * @return An initialized Actuator structure with the provided components and default state values.
 */
Actuator initActuator(TalonSRX *talonSrx, Pot *pot, PIDController *pid, int *direction);

/**
 * @brief Initializes the hall effect sensors for the front and back actuators.
 * 
 * This function configures the specified GPIO pins for the front and back hall
 * effect sensors as inputs with pull-up resistors, and sets up interrupt
 * handlers to count the pulses from the sensors. The pulse counts are used to
 * track the position of the actuators. The interrupt handlers will decrement
 * the pulse counts based on the direction of movement, which is determined by
 * the `frontDirection` and `backDirection` variables.
 * 
 * @param[in] pinFront GPIO pin number for the front hall effect sensor.
 * @param[in] pinBack GPIO pin number for the back hall effect sensor.
 */
void hallEffectInit(int pinFront, int pinBack);

/**
 * @brief Moves actuators to a target position while keeping them synchronized.
 * 
 * This function calculates the position error between the front and back
 * actuators and uses PID control to adjust their velocities to move them
 * towards the target position while minimizing the position error. The function
 * also updates the previous position, velocity, and time for both actuators to
 * be used in
 * the next control cycle.
 * 
 * @param[in] frontActuator Pointer to the Actuator structure for the front actuator.
 * @param[in] backActuator Pointer to the Actuator structure for the back actuator.
 * @param[in] targetPosition The desired target position for both actuators to move towards.
 * 
 * @see moveSyncActuatorsToVelocity() for a similar function that moves the actuators to a target velocity instead of position.
 */
void moveSyncActuatorsToPosition(Actuator *frontActuator, Actuator *backActuator, double targetPosition);

/**
 * @brief Moves actuators to a target velocity while keeping them synchronized.
 * 
 * This function calculates the velocity error between the front and back actuators and uses PID control to
 * adjust their velocities to move them towards the target velocity while minimizing the velocity error.
 * The function also updates the previous position, velocity, and time for both actuators to be used in
 * the next control cycle.
 * 
 * @param[in] frontActuator Pointer to the Actuator structure for the front actuator.
 * @param[in] backActuator Pointer to the Actuator structure for the back actuator.
 * @param[in] targetVelocity The desired target velocity for both actuators to move towards.
 * 
 * @see moveSyncActuatorsToPosition() for a similar function that moves the actuators to a target position instead of velocity.
 */
void moveSyncActuatorsToVelocity(Actuator *frontActuator, Actuator *backActuator, double targetVelocity);

/**
 * @brief Calculates the pulse counts from the hall effect sensors and updates the actuator positions accordingly.
 * 
 * This function reads the pulse counts from the hall effect sensors for both the front and back actuators,
 * clamps them to a maximum value based on the `pulse_mm` variable, and updates the previous position of each
 * actuator based on the pulse counts. It then creates a `PositionPacket_OneRobot` structure, populates it with the
 * updated positions of both actuators, and returns it.
 * 
 * @param[in] frontActuator Pointer to the Actuator structure for the front actuator.
 * @param[in] backActuator Pointer to the Actuator structure for the back actuator.
 * 
 * @return A `PositionPacket_OneRobot` structure containing the updated positions of both actuators.
 */
PositionPacket_OneRobot calculatePulse(Actuator *frontActuator, Actuator *backActuator);

/**
 * @brief Evaluates the potentiometer values for the front and back actuators.
 * 
 * This function reads the analog values from the potentiometers connected to the front and back actuators,
 * and updates their respective position values based on the readings.
 * 
 * @param[in] frontActuator Pointer to the Actuator structure for the front actuator.
 * @param[in] backActuator Pointer to the Actuator structure for the back actuator.
 */
void evaluatePot(Actuator *frontActuator, Actuator *backActuator);
