#include "actuators.h"
#include "pid.h"
#include "esp_timer.h"
#include <math.h>

Actuator initActuator(TalonSRX *talonSrx, Pot *pot, PIDController *pid)
{
    return (Actuator){
        .controller = talonSrx,
        .pot = pot,
        .pid = pid,
        .prevPosition = 0,
        .prevVelocity = 0,
        .lastTime = -1,
        .velocity = 0};
};

void moveSyncActuatorsToPosition(Actuator *leftActuator, Actuator *rightActuator, double targetPosition)
{
    double leftPos = leftActuator->pot->pos;
    double rightPos = rightActuator->pot->pos;
    double syncPositionError = (leftPos - rightPos) * 0.5;
    //double syncPositionError = 0;

    if (leftActuator->lastTime < 0 || rightActuator->lastTime < 0)
    {
        leftActuator->lastTime = esp_timer_get_time() / 1000000.0;
        rightActuator->lastTime = esp_timer_get_time() / 1000000.0;

        return;
    }

    double currentTime = esp_timer_get_time() / 1000000.0;

    double leftPositionOutput = computePID(leftActuator->pid, targetPosition, leftPos - syncPositionError, currentTime - leftActuator->lastTime);
    double rightPositionOutput = computePID(rightActuator->pid, targetPosition, rightPos + syncPositionError, currentTime - rightActuator->lastTime);

    //double leftDistance = leftPositionOutput - ((leftPos + rightPos) / 2.0);
    //double rightDistance = rightPositionOutput - ((leftPos + rightPos) / 2.0);
    
    double leftVelocity = leftPositionOutput;
    double rightVelocity = rightPositionOutput;

    printf("Left Output: %f, Left Pos: %f, Right Output: %f, Right Pos: %f, Target Pos: %f\n", leftVelocity, leftPos, rightVelocity, rightPos, targetPosition);

    leftVelocity = fmax(fmin(leftVelocity, 1), -1);
    rightVelocity = fmax(fmin(rightVelocity, 1), -1);

    leftActuator->velocity = leftVelocity;
    rightActuator->velocity = rightVelocity;
    //setSRX(leftActuator->controller, leftVelocity);
    //setSRX(rightActuator->controller, rightVelocity);

    //printf("Left Output: %f, Right Output: %f\n", leftVelocity, rightVelocity);

    leftActuator->lastTime = currentTime;
    rightActuator->lastTime = currentTime;
}

void moveSyncActuatorsToVelocity(Actuator *leftActuator, Actuator *rightActuator, double targetVelocity)
{
    double leftPos = leftActuator->pot->pos;
    double rightPos = rightActuator->pot->pos;
    double syncPositionError = (leftPos - rightPos);

    if (leftActuator->lastTime < 0 || rightActuator->lastTime < 0)
    {
        leftActuator->lastTime = esp_timer_get_time() / 1000000.0;
        rightActuator->lastTime = esp_timer_get_time() / 1000000.0;

        leftActuator->prevPosition = leftPos;
        rightActuator->prevPosition = rightPos;
        leftActuator->prevVelocity = 0;
        rightActuator->prevVelocity = 0;
        return;
    }

    double leftVelocity = leftActuator->prevVelocity;
    double rightVelocity = rightActuator->prevVelocity;

    double currentTime = esp_timer_get_time() / 1000000.0;

    double leftVelocityOutput = computePID(leftActuator->pid, targetVelocity - syncPositionError, leftVelocity, currentTime - leftActuator->lastTime);
    double rightVelocityOutput = computePID(rightActuator->pid, targetVelocity + syncPositionError, rightVelocity, currentTime - rightActuator->lastTime);

    //printf("Left Velocity: %f, Position: %f, Right Velocity: %f, Position: %f, Target Velocity: %f (before clamping)\n", leftVelocityOutput, leftPos, rightVelocityOutput, rightPos, targetVelocity);

    leftVelocityOutput = fmax(fmin(leftVelocityOutput, 1), -1);
    rightVelocityOutput = fmax(fmin(rightVelocityOutput, 1), -1);

    leftActuator->velocity = leftVelocityOutput;
    rightActuator->velocity = rightVelocityOutput;
    //setSRX(leftActuator->controller, leftVelocityOutput);
    //setSRX(rightActuator->controller, rightVelocityOutput);

    // printf("Left Velocity: %f, Right Velocity: %f, Target Velocity: %f\n", leftVelocityOutput, rightVelocityOutput, targetVelocity);
    // printf("targetVelocity: %f, time delta: %f\n", targetVelocity, currentTime - leftActuator->lastTime);

    leftActuator->prevPosition = leftPos;
    rightActuator->prevPosition = rightPos;
    leftActuator->prevVelocity = leftVelocityOutput;
    rightActuator->prevVelocity = rightVelocityOutput;
    leftActuator->lastTime = currentTime;
    rightActuator->lastTime = currentTime;
}
