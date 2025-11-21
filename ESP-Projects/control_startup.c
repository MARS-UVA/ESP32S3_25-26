#include "can2.h"
#include "uart.h"

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

    leftActuator = TalonSRXInit(LEFT_ACTUATOR_ID, bool 0);
    rightActuator = TalonSRXInit(RIGHT_ACTUATOR_ID, bool 1);
}

void directControl(SerialPacket pkt)
{
    int8_t leftSpeed = pkt.top_left_wheel;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1, 0);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1, 0);

    int8_t rightSpeed = packet.top_right_wheel;
    frontRight.setControl(&frontRight, ((int8_t)(rightSpeed - 127)), 0);
    backRight.setControl(&backRight, ((int8_t)(rightSpeed - 127)), 0);
}
