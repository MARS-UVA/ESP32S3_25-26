#include "control_startup.h"
#include "OneRobot.h"

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX backBucketDrum;
TalonFX frontBucketDrum;

TalonSRX frontActuator;
TalonSRX backActuator;

TalonFX *fxMotors[] = {&frontLeft, &backLeft, &frontRight, &backRight, &frontBucketDrum, &backBucketDrum};
TalonSRX *srxMotors[] = {&frontActuator, &backActuator};

// Initialize Talon "objects"
void initializeTalons(PDH *pdh)
{
    frontLeft = talonFXInit(FRONT_LEFT_WHEEL_ID, FRONT_LEFT_WHEEL_CHANNEL_ID, pdh);
    backLeft = talonFXInit(BACK_LEFT_WHEEL_ID, BACK_LEFT_WHEEL_CHANNEL_ID, pdh);
    frontRight = talonFXInit(FRONT_RIGHT_WHEEL_ID, FRONT_RIGHT_WHEEL_CHANNEL_ID, pdh);
    backRight = talonFXInit(BACK_RIGHT_WHEEL_ID, BACK_RIGHT_WHEEL_CHANNEL_ID, pdh);

    backBucketDrum = talonFXInit(BACK_BUCKET_DRUM_ID, BACK_BUCKET_DRUM_CHANNEL_ID, pdh);
    frontBucketDrum = talonFXInit(FRONT_BUCKET_DRUM_ID, FRONT_BUCKET_DRUM_CHANNEL_ID, pdh);

    frontActuator = talonSRXInit(FRONT_ACTUATOR_ID, FRONT_ACTUATOR_CHANNEL_ID, false);
    backActuator = talonSRXInit(BACK_ACTUATOR_ID, BACK_ACTUATOR_CHANNEL_ID, true);
}

void directControl(ControlPacket_OneRobot pkt)
{
    int8_t leftSpeed = pkt.front_left_wheel;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1);

    int8_t rightSpeed = pkt.front_right_wheel;
    setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)) * -1);
    setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);
}
