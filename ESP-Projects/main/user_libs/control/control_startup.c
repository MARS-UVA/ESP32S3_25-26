#include "control_startup.h"
#include "uart.h"

// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_DRIVE_MOTOR_ID 38
#define FRONT_RIGHT_DRIVE_MOTOR_ID 36
#define BACK_LEFT_DRIVE_MOTOR_ID 13
#define BACK_RIGHT_DRIVE_MOTOR_ID 37

// spinning
#define SPIN_FRONT_MOTOR_ID 60   // assuming left = front
#define SPIN_BACK_MOTOR_ID 25    // needs updating
#define ARM_FRONT_ACTUATOR_ID 55 ///
#define ARM_BACK_ACTUATOR_ID 16  // needs updating

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_DRIVE_MOTOR_CHANNEL_ID 12
#define BACK_LEFT_DRIVE_MOTOR_CHANNEL_ID 13
#define FRONT_RIGHT_DRIVE_MOTOR_CHANNEL_ID 3
#define BACK_RIGHT_DRIVE_MOTOR_CHANNEL_ID 1

#define SPIN_FRONT_CHANNEL_ID 2
#define SPIN_BACK_CHANNEL_ID 0
#define ARM_BACK_ACTUATOR_CHANNEL_ID 14
#define ARM_FRONT_TALON_CHANNEL_ID 15

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX spinFront;
TalonFX spinBack;

TalonSRX frontArmActuator;
TalonSRX backArmActuator;

// TalonFX *fxMotors[] = {&frontLeft, &backLeft, &frontRight, &backRight, &spinFront, &spinBack};
// TalonSRX *srxMotors[] = {&frontArmActuator, &backArmActuator};
//  Initialize Talon "objects"
void initializeTalons()
{
    frontLeft = talonFXInit(FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_CHANNEL_ID);
    backLeft = talonFXInit(BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_CHANNEL_ID);
    frontRight = talonFXInit(FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_CHANNEL_ID);
    backRight = talonFXInit(BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_CHANNEL_ID);

    frontArmActuator = talonSRXInit(ARM_FRONT_ACTUATOR_ID, ARM_FRONT_TALON_CHANNEL_ID, false);
    spinFront = talonFXInit(SPIN_FRONT_MOTOR_ID, SPIN_FRONT_CHANNEL_ID);

    backArmActuator = talonSRXInit(ARM_BACK_ACTUATOR_ID, ARM_BACK_ACTUATOR_CHANNEL_ID, false);
    spinBack = talonFXInit(SPIN_BACK_MOTOR_ID, SPIN_BACK_CHANNEL_ID);
}

void directControl(SerialPacket pkt)
{
    int8_t leftSpeed = pkt.front_left_drive_motor;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)));

    int8_t rightSpeed = pkt.front_right_drive_motor;
    setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)));
    setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);

    setTargetFX(&spinFront, ((int8_t)(pkt.spin_front_motor - 127)) * -1);
    setTargetFX(&spinBack, ((int8_t)(pkt.spin_front_motor - 127)));

    float actuatorOutput = 0;
    if (pkt.arm_back_actuator > 127)
    {
        actuatorOutput = 0.8;
    }
    else if (pkt.arm_back_actuator < 127)
    { // if actuator value is negative, set actuator output to 80% of full output downwards
        actuatorOutput = -0.8;
    }
    setSRX(&backArmActuator, actuatorOutput);

    actuatorOutput = 0;
    if (pkt.arm_front_actuator > 127)
    {
        actuatorOutput = 0.8;
    }
    else if (pkt.arm_front_actuator < 127)
    { // if actuator value is negative, set actuator output to 80% of full output downwards
        actuatorOutput = -0.8;
    }
    setSRX(&frontArmActuator, actuatorOutput);

}

void UART_can_task()
{
    SerialPacket motor_state = {0, 0x00, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0X7F, 0x7F};
    SerialPacket new_data;

    while (1)
    {
        if (xQueueReceive(uart_queue, &new_data, 0) == pdTRUE)
        {
            motor_state = new_data;
        }
        directControl(motor_state);
        vTaskDelay(pdMS_TO_TICKS(1));
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
