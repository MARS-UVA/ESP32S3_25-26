#include "OneRobot.h"

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
TalonSRX *srxMotors[] = {&leftActuatorSRX, &rightActuatorSRX};

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

    // potSetup((adc_channel_t[]){ADC_PIN_LEFT, ADC_PIN_RIGHT}, 2);
    // leftActuatorPot = potInit(90, 1260, ADC_PIN_LEFT);
    // rightActuatorPot = potInit(90, 1260, ADC_PIN_RIGHT);
    
    // position
    // leftActuatorPID = initPID(2, 0.15, 0.001);
    // rightActuatorPID = initPID(2, 0.15, 0.001);

    // velocity
    // leftActuatorPID = initPID(0.9, 0.5, 0); 
    // rightActuatorPID = initPID(0.9, 0.5, 0);

    hallEffectInit(HALL_PIN_LEFT, HALL_PIN_RIGHT);

    leftActuator = initActuator(&leftActuatorSRX, &leftActuatorPot, &leftActuatorPID, &leftDirection);
    rightActuator = initActuator(&rightActuatorSRX, &rightActuatorPot, &rightActuatorPID, &rightDirection);
}

void directControl(ControlPacket pkt)
{
    // int8_t leftSpeed = pkt.top_left_wheel;
    // setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    // setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1);

    // int8_t rightSpeed = pkt.top_right_wheel;
    // setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)) * -1);
    // setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);

    double actuatorSpeed = (pkt.actuator - 127) / 255.0;
    if (actuatorSpeed > 0) {
        *leftActuator.direction = -1;
        *rightActuator.direction = -1;
    } if (actuatorSpeed < 0) {
        *leftActuator.direction = 1;
        *rightActuator.direction = 1;
    }

    setSRX(leftActuator.controller, actuatorSpeed);
    setSRX(rightActuator.controller, actuatorSpeed);
    // moveSyncActuatorsToPosition(&leftActuator, &rightActuator, actuatorPosition);
    // moveSyncActuatorsToVelocity(&leftActuator, &rightActuator, -1);
}
