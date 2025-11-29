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

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX bucketDrumRight;
TalonFX bucketDrumLeft;
TalonSRX leftActuator;
TalonSRX rightActuator;

// Initialize Talon "objects"
void initializeTalons()
{
    frontLeft = talonFXInit(FRONT_LEFT_WHEEL_ID);
    backLeft = talonFXInit(BACK_LEFT_WHEEL_ID);
    frontRight = talonFXInit(FRONT_RIGHT_WHEEL_ID);
    backRight = talonFXInit(BACK_RIGHT_WHEEL_ID);
    bucketDrumRight = talonFXInit(BUCKET_DRUM_RIGHT_ID);
    bucketDrumLeft = talonFXInit(BUCKET_DRUM_LEFT_ID);

    leftActuator = talonSRXInit(LEFT_ACTUATOR_ID, false);
    rightActuator = talonSRXInit(RIGHT_ACTUATOR_ID, true);
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
    SerialPacket motor_state = {1, 0x0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    SerialPacket new_data;

    int led_B = 12;
    bool state = false;

    while (1)
    {
        if (xQueueReceive(uart_queue, &new_data, 0) == pdTRUE)
        {
            motor_state = new_data;
            ledToggle(led_B, &state);
        }
        // directControl lol
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
