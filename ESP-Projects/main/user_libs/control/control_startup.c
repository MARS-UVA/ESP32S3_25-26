#include "control_startup.h"
#include "../pid/actuators.h"
#include "../ADC/adc2.h"

// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 36
#define BACK_LEFT_WHEEL_ID 37
#define FRONT_RIGHT_WHEEL_ID 38
#define BACK_RIGHT_WHEEL_ID 13
#define BUCKET_DRUM_RIGHT_ID 25
#define BUCKET_DRUM_LEFT_ID 60
#define LEFT_ACTUATOR_ID 3
#define RIGHT_ACTUATOR_ID 4

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 14
#define BUCKET_DRUM_RIGHT_CHANNEL_ID 4
#define BUCKET_DRUM_LEFT_CHANNEL_ID 2
#define LEFT_ACTUATOR_CHANNEL_ID 10
#define RIGHT_ACTUATOR_CHANNEL_ID 9

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX bucketDrumRight;
TalonFX bucketDrumLeft;

Actuator leftActuator;
TalonSRX leftActuatorSRX;
Pot leftActuatorPot;
PIDController leftActuatorPID;

Actuator rightActuator;
TalonSRX rightActuatorSRX;
Pot rightActuatorPot;
PIDController rightActuatorPID;

TalonFX *fxMotors[] = {&frontLeft, &backLeft, &frontRight, &backRight, &bucketDrumLeft, &bucketDrumRight};
// TalonSRX *srxMotors[] = {&leftActuator, &rightActuator};

// Initialize Talon "objects"
void initializeTalons()
{
    // frontLeft = talonFXInit(FRONT_LEFT_WHEEL_ID, FRONT_LEFT_WHEEL_CHANNEL_ID);
    // backLeft = talonFXInit(BACK_LEFT_WHEEL_ID, BACK_LEFT_WHEEL_CHANNEL_ID);
    // frontRight = talonFXInit(FRONT_RIGHT_WHEEL_ID, FRONT_RIGHT_WHEEL_CHANNEL_ID);
    // backRight = talonFXInit(BACK_RIGHT_WHEEL_ID, BACK_RIGHT_WHEEL_CHANNEL_ID);

    // bucketDrumRight = talonFXInit(BUCKET_DRUM_RIGHT_ID, BUCKET_DRUM_RIGHT_CHANNEL_ID);
    // bucketDrumLeft = talonFXInit(BUCKET_DRUM_LEFT_ID, BUCKET_DRUM_LEFT_CHANNEL_ID);

    leftActuatorSRX = talonSRXInit(LEFT_ACTUATOR_ID, LEFT_ACTUATOR_CHANNEL_ID, true);
    rightActuatorSRX = talonSRXInit(RIGHT_ACTUATOR_ID, RIGHT_ACTUATOR_CHANNEL_ID, true);

    potSetup((adc_channel_t[]){ADC_CHANNEL_3, ADC_CHANNEL_4}, 2);
    leftActuatorPot = potInit(90, 1260, ADC_CHANNEL_3);
    rightActuatorPot = potInit(100, 1280, ADC_CHANNEL_4);

    leftActuatorPID = initPID(0.9, 0, 0);
    rightActuatorPID = initPID(0.9, 0, 0);

    leftActuator = initActuator(&leftActuatorSRX, &leftActuatorPot, &leftActuatorPID);
    rightActuator = initActuator(&rightActuatorSRX, &rightActuatorPot, &rightActuatorPID);
}
void directControl(SerialPacket pkt)
{
    // int8_t leftSpeed = pkt.top_left_wheel;
    // setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    // setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1);

    // int8_t rightSpeed = pkt.top_right_wheel;
    // setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)) * -1);
    // setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);

    double targetPosition = ((int8_t)pkt.actuator) / 255.0;
    // moveSyncActuatorsToPosition(&leftActuator, &rightActuator, targetPosition);
    moveSyncActuatorsToVelocity(&leftActuator, &rightActuator, -1);
    vTaskDelay(pdMS_TO_TICKS(1));
}

// printf for actuator positions
void printActuatorPositions()
{
    while (1)
    {
        double leftPos = leftActuator.pot->pos;
        double rightPos = rightActuator.pot->pos;
        printf("%ld\t%f\t%f\n", xTaskGetTickCount(), leftPos, rightPos);
        // printf("Left Actuator Target: %f, Right Actuator Target: %f\n", leftActuator.prevVelocity, rightActuator.prevVelocity);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void UART_can_task()
{
    SerialPacket motor_state = {0, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    SerialPacket new_data;

    while (1)
    {
        if (xQueueReceive(uart_queue, &new_data, 0) == pdTRUE)
        {
            motor_state = new_data;
        }
        directControl(motor_state);

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void UART_tx_task() //
{
    OutPacket packet = {0, 1, 0, 1, 0, 0, 1, 0, 0, 0};

    while (1)
    {
        /*packet.invalid = 0;
        packet.header = 0x0;

        packet.top_left_wheel = fxMotors[0]->current;
        packet.back_left_wheel = fxMotors[1]->current;
        packet.top_right_wheel = fxMotors[2]->current;
        packet.back_right_wheel = fxMotors[3]->current;
        packet.bucket_left = fxMotors[4]->current;
        packet.bucket_right = fxMotors[5]->current;

        packet.left_actuator = srxMotors[0]->current;
        packet.right_actuator = srxMotors[1]->current;
        */

        UART_write(&packet);
        vTaskDelay(100);
    }
}
