#include "control_startup.h"

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

void directControl(ControlPacket pkt)
{
    int8_t leftSpeed = pkt.top_left_wheel;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1);

    int8_t rightSpeed = pkt.top_right_wheel;
    setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)) * -1);
    setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);
}
