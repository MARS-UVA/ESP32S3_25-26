#include "control_startup.h"
#include "ExcavationRobot.h"

//Excavation robot
TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX bucketLadder;
TalonFX conveyorBelt;

TalonSRX leftTrackActuator;
TalonSRX rightTrackActuator;

TalonFX *fxMotors[] = {&frontLeft, &backLeft, &frontRight, &backRight, &bucketLadder, &conveyorBelt};
TalonSRX *srxMotors[] = {&leftTrackActuator, &rightTrackActuator};

// Initialize Excavation Talons "objects"
void initializeTalons(PDH *pdh)
{
    frontLeft = talonFXInit(FRONT_LEFT_WHEEL_ID, FRONT_LEFT_WHEEL_CHANNEL_ID, pdh);
    backLeft = talonFXInit(BACK_LEFT_WHEEL_ID, BACK_LEFT_WHEEL_CHANNEL_ID, pdh);
    frontRight = talonFXInit(FRONT_RIGHT_WHEEL_ID, FRONT_RIGHT_WHEEL_CHANNEL_ID, pdh);
    backRight = talonFXInit(BACK_RIGHT_WHEEL_ID, BACK_RIGHT_WHEEL_CHANNEL_ID, pdh);

    bucketLadder = talonFXInit(BUCKET_LADDER_ID, BUCKET_LADDER_CHANNEL_ID, pdh);
    conveyorBelt = talonFXInit(CONVEYOR_BELT_ID, CONVEYOR_BELT_CHANNEL_ID, pdh);

    leftTrackActuator = talonSRXInit(LEFT_TRACK_ACTUATOR_ID, LEFT_TRACK_ACTUATOR_CHANNEL_ID, false);
    rightTrackActuator = talonSRXInit(RIGHT_TRACK_ACTUATOR_ID, RIGHT_TRACK_ACTUATOR_CHANNEL_ID, true);
}

void directControl(ControlPacket_ExcavationRobot pkt)
{
    int8_t leftSpeed = pkt.front_left_wheel;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1);

    int8_t rightSpeed = pkt.front_right_wheel;
    setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)) * -1);
    setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);
}