#include "control_startup.h"

// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 36
#define BACK_LEFT_WHEEL_ID 37
#define FRONT_RIGHT_WHEEL_ID 38
#define BACK_RIGHT_WHEEL_ID 13
#define BUCKET_DRUM_RIGHT_ID 25
#define BUCKET_DRUM_LEFT_ID 60
#define LEFT_ACTUATOR_ID 0
#define RIGHT_ACTUATOR_ID 1

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 14
#define BUCKET_DRUM_RIGHT_CHANNEL_ID 4
#define BUCKET_DRUM_LEFT_CHANNEL_ID 2
#define LEFT_ACTUATOR_CHANNEL_ID 0
#define RIGHT_ACTUATOR_CHANNEL_ID 1

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX bucketDrumRight;
TalonFX bucketDrumLeft;

TalonSRX leftActuator;
TalonSRX rightActuator;

TalonFX *fxMotors[] = {&frontLeft, &backLeft, &frontRight, &backRight, &bucketDrumLeft, &bucketDrumRight};
TalonSRX *srxMotors[] = {&leftActuator, &rightActuator};

// Initialize Talon "objects"
void initializeTalons()
{
    frontLeft = talonFXInit(FRONT_LEFT_WHEEL_ID, FRONT_LEFT_WHEEL_CHANNEL_ID);
    backLeft = talonFXInit(BACK_LEFT_WHEEL_ID, BACK_LEFT_WHEEL_CHANNEL_ID);
    frontRight = talonFXInit(FRONT_RIGHT_WHEEL_ID, FRONT_RIGHT_WHEEL_CHANNEL_ID);
    backRight = talonFXInit(BACK_RIGHT_WHEEL_ID, BACK_RIGHT_WHEEL_CHANNEL_ID);

    // bucketDrumRight = talonFXInit(BUCKET_DRUM_RIGHT_ID, BUCKET_DRUM_RIGHT_CHANNEL_ID);
    // bucketDrumLeft = talonFXInit(BUCKET_DRUM_LEFT_ID, BUCKET_DRUM_LEFT_CHANNEL_ID);

    // leftActuator = talonSRXInit(LEFT_ACTUATOR_ID, LEFT_ACTUATOR_CHANNEL_ID, false);
    // rightActuator = talonSRXInit(RIGHT_ACTUATOR_ID, RIGHT_ACTUATOR_CHANNEL_ID, true);
}

void directControl(SerialPacket pkt)
{
    int8_t leftSpeed = pkt.top_left_wheel;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1);

    int8_t rightSpeed = pkt.top_right_wheel;
    setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)) * -1);
    setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);
}

void UART_can_task()
{
    SerialPacket motor_state = {0, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    SerialPacket new_data;
    // ESP_LOGI("HELLO", "HELLO AGAIN");

    while (1)
    {
        if (xQueueReceive(uart_queue, &new_data, 0) == pdTRUE)
        {
            motor_state = new_data;
        }
        directControl(motor_state);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void UART_tx_task() //
{
    OutPacket packet = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    while (1)
    {
        packet.invalid = 0;
        packet.header = 0x0;

        packet.top_left_wheel = fxMotors[0]->current;
        packet.back_left_wheel = fxMotors[1]->current;
        packet.top_right_wheel = fxMotors[2]->current;
        packet.back_right_wheel = fxMotors[3]->current;
        packet.bucket_left = fxMotors[4]->current;
        packet.bucket_right = fxMotors[5]->current;

        packet.left_actuator = srxMotors[0]->current;
        packet.right_actuator = srxMotors[1]->current;

        UART_write(&packet);
        vTaskDelay(1);
    }
}
