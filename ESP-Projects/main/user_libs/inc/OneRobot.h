#pragma once

#include "can.h"
#include "uart.h"
#include "packets.h"
#include "actuators.h"

#include "talonFX.h"
#include "talonSRX.h"
#include "pdh.h"
#include "i2c.h"

// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 38 // 38
#define BACK_LEFT_WHEEL_ID 13 // 13
#define FRONT_RIGHT_WHEEL_ID 36 // 36
#define BACK_RIGHT_WHEEL_ID 37
#define BACK_BUCKET_DRUM_ID 25  
#define FRONT_BUCKET_DRUM_ID 60
#define FRONT_ACTUATOR_ID 16
#define BACK_ACTUATOR_ID 55 

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 16 // 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 15 // 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 13 // 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 14
#define BACK_BUCKET_DRUM_CHANNEL_ID 18
#define FRONT_BUCKET_DRUM_CHANNEL_ID 12
#define FRONT_ACTUATOR_CHANNEL_ID 10
#define BACK_ACTUATOR_CHANNEL_ID 11
// Define INA219
#define INA219_SENSOR_ADDR      0x40
#define INA219_REG_BUSVOLTAGE   0x02

#define HALL_PIN_FRONT   7
#define HALL_PIN_BACK  15

#define ADC_PIN_FRONT ADC_CHANNEL_3
#define ADC_PIN_BACK ADC_CHANNEL_4

extern int frontDirection;
extern int backDirection;
extern QueueHandle_t uart_queue;

void initializeTalons(void);
void directControl(ControlPacket_OneRobot pkt);
void initAuxVoltageSensor(void);
float updateAuxVoltage(void);


/**
 * @brief Representation of hardware across robot needed for callback func setup
 */
typedef struct {
    PDH* pdh;
    TalonFX** motors;
    size_t count;
}RobotRegistry;

// TODO: Remove this function after testing
void test_run_motor();

TempPacket_OneRobot getTemperatureOneRobot();

void canSetupTalons();

/**
 * @brief Sets up CAN comms and all callback funcs used on competition robot
 *
 * @param pdh     Pointer to PDH instance.
 * @param motors  Pointer to TalonFX array for temp sensing
 * @param count   Number of TalonFX motors on robot
 */
void canSetupRobot(PDH *pdh, TalonFX **motors, size_t count);

/**
 * @brief Gets current/voltage data from the robot and stores it in a packet struct.
 * 
 * @param pdh   Pointer to PDH instance.
 * @return CurrVoltPacket_OneRobot struct containing current/voltage data.
 */
CurrVoltPacket_OneRobot getCurrentVoltageOneRobot(PDH *pdh);
