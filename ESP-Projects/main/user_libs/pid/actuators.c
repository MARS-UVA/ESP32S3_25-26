#include "actuators.h"
#include "pid.h"
#include "esp_timer.h"
#include <math.h>
#include "driver/gpio.h"
#include "utils.h"

static double pulse_mm = 22.5;

volatile int pulseCountLeft = 0;
volatile int pulseCountRight = 0;

int leftDirection;
int rightDirection;

Actuator initActuator(TalonSRX *talonSrx, Pot *pot, PIDController *pid, int *direction)
{
    return (Actuator){
        .controller = talonSrx,
        .pot = pot,
        .pid = pid,
        .prevPosition = 0,
        .prevVelocity = 0,
        .lastTime = -1,
        .velocity = 0,
        .direction = direction};
};

static void IRAM_ATTR gpio_isr_handler_left(void* arg) {
    pulseCountLeft+=leftDirection;
}

static void IRAM_ATTR gpio_isr_handler_right(void* arg) {
    pulseCountRight+=rightDirection;
}

void hallEffectInit(int pinLeft, int pinRight) 
{
    gpio_config_t io_conf_a = {
        .intr_type = GPIO_INTR_POSEDGE,    // <--- Trigger on Rising Edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pinLeft),    
        .pull_up_en = 1
    };
    gpio_config(&io_conf_a);

    gpio_config_t io_conf_b = {
        .intr_type = GPIO_INTR_POSEDGE, 
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pinRight),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf_b);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(pinLeft, gpio_isr_handler_left, (void*) pinLeft);
    gpio_isr_handler_add(pinRight, gpio_isr_handler_right, (void*) pinRight);
}

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

void calculatePulse(Actuator *leftActuator, Actuator *rightActuator) {
    pulseCountLeft = fmax(fmin(pulseCountLeft, (int)(pulse_mm*254)), 0);
    leftActuator->prevPosition = map((int)(pulse_mm*254), 0, pulseCountLeft);
        
    pulseCountRight = fmax(fmin(pulseCountRight, (int)(pulse_mm*254)), 0);
    rightActuator->prevPosition = map((int)(pulse_mm*254), 0, pulseCountRight);
}