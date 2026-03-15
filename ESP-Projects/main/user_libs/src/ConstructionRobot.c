#include "control_startup.h"
#include "ConstructionRobot.h"

//Construction robot
TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;

TalonSRX actuator;

TalonFX *fxMotors[] = {&frontLeft, &backLeft, &frontRight, &backRight};
TalonSRX *srxMotors[] = {&actuator};

// Initialize Construction Talons "objects"
void initializeTalons(PDH *pdh)
{
    frontLeft = talonFXInit(FRONT_LEFT_WHEEL_ID, FRONT_LEFT_WHEEL_CHANNEL_ID, pdh);
    backLeft = talonFXInit(BACK_LEFT_WHEEL_ID, BACK_LEFT_WHEEL_CHANNEL_ID, pdh);
    frontRight = talonFXInit(FRONT_RIGHT_WHEEL_ID, FRONT_RIGHT_WHEEL_CHANNEL_ID, pdh);
    backRight = talonFXInit(BACK_RIGHT_WHEEL_ID, BACK_RIGHT_WHEEL_CHANNEL_ID, pdh);

    actuator = talonSRXInit(ACTUATOR_ID, ACTUATOR_CHANNEL_ID, false);
}

void directControl(ControlPacket_ConstructionRobot pkt)
{
    int8_t leftSpeed = pkt.front_left_wheel;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1);

    int8_t rightSpeed = pkt.front_right_wheel;
    setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)) * -1);
    setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);
}