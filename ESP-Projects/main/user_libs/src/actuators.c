#include "actuators.h"

static double pulse_mm = 44; //printing gave a multiplier of 22.5

volatile int pulseCountFront = 0;
volatile int pulseCountBack = 0;

int frontDirection;
int backDirection;

Actuator initActuator(TalonSRX *talonSrx, Pot *pot, PIDController *pid, int *direction)
{
    return (Actuator){
        .controller = talonSrx,
        .pot = pot,
        .pid = pid,
        .prevPosition = 1,
        .prevVelocity = 0,
        .lastTime = -1,
        .velocity = 0,
        .direction = direction};
};

static void IRAM_ATTR gpio_isr_handler_front(void* arg) {
    pulseCountFront-=frontDirection;
}

static void IRAM_ATTR gpio_isr_handler_back(void* arg) {
    pulseCountBack-=backDirection;
}

void hallEffectInit(int pinFront, int pinBack) 
{
    gpio_config_t io_conf_a = {
        .intr_type = GPIO_INTR_POSEDGE,    // <--- Trigger on Rising Edge
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pinFront),    
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf_a);

    gpio_config_t io_conf_b = {
        .intr_type = GPIO_INTR_POSEDGE, 
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << pinBack),
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf_b);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(pinFront, gpio_isr_handler_front, (void*) pinFront);
    gpio_isr_handler_add(pinBack, gpio_isr_handler_back, (void*) pinBack);
}

void moveSyncActuatorsToPosition(Actuator *frontActuator, Actuator *backActuator, double targetPosition)
{
    double frontPos = frontActuator->pot->pos;
    double backPos = backActuator->pot->pos;
    double syncPositionError = (frontPos - backPos) * 0.5;
    //double syncPositionError = 0;

    if (frontActuator->lastTime < 0 || backActuator->lastTime < 0)
    {
        frontActuator->lastTime = esp_timer_get_time() / 1000000.0;
        backActuator->lastTime = esp_timer_get_time() / 1000000.0;

        return;
    }

    double currentTime = esp_timer_get_time() / 1000000.0;

    double frontPositionOutput = computePID(frontActuator->pid, targetPosition, frontPos - syncPositionError, currentTime - frontActuator->lastTime);
    double backPositionOutput = computePID(backActuator->pid, targetPosition, backPos + syncPositionError, currentTime - backActuator->lastTime);

    //double frontDistance = frontPositionOutput - ((frontPos + backPos) / 2.0);
    //double backDistance = backPositionOutput - ((frontPos + backPos) / 2.0);

    double frontVelocity = frontPositionOutput;
    double backVelocity = backPositionOutput;

    printf("Front Output: %f, Front Pos: %f, Back Output: %f, Back Pos: %f, Target Pos: %f\n", frontVelocity, frontPos, backVelocity, backPos, targetPosition);

    frontVelocity = fmax(fmin(frontVelocity, 1), -1);
    backVelocity = fmax(fmin(backVelocity, 1), -1);

    frontActuator->velocity = frontVelocity;
    backActuator->velocity = backVelocity;
    //setSRX(frontActuator->controller, frontVelocity);
    //setSRX(backActuator->controller, backVelocity);

    //printf("Front Output: %f, Back Output: %f\n", frontVelocity, backVelocity);

    frontActuator->lastTime = currentTime;
    backActuator->lastTime = currentTime;
}

void moveSyncActuatorsToVelocity(Actuator *frontActuator, Actuator *backActuator, double targetVelocity)
{
    double frontPos = frontActuator->pot->pos;
    double backPos = backActuator->pot->pos;
    double syncPositionError = (frontPos - backPos);

    if (frontActuator->lastTime < 0 || backActuator->lastTime < 0)
    {
        frontActuator->lastTime = esp_timer_get_time() / 1000000.0;
        backActuator->lastTime = esp_timer_get_time() / 1000000.0;

        frontActuator->prevPosition = frontPos;
        backActuator->prevPosition = backPos;
        frontActuator->prevVelocity = 0;
        backActuator->prevVelocity = 0;
        return;
    }

    double frontVelocity = frontActuator->prevVelocity;
    double backVelocity = backActuator->prevVelocity;

    double currentTime = esp_timer_get_time() / 1000000.0;

    double frontVelocityOutput = computePID(frontActuator->pid, targetVelocity - syncPositionError, frontVelocity, currentTime - frontActuator->lastTime);
    double backVelocityOutput = computePID(backActuator->pid, targetVelocity + syncPositionError, backVelocity, currentTime - backActuator->lastTime);

    //printf("Front Velocity: %f, Position: %f, Back Velocity: %f, Position: %f, Target Velocity: %f (before clamping)\n", frontVelocityOutput, frontPos, backVelocityOutput, backPos, targetVelocity);

    frontVelocityOutput = fmax(fmin(frontVelocityOutput, 1), -1);
    backVelocityOutput = fmax(fmin(backVelocityOutput, 1), -1);

    frontActuator->velocity = frontVelocityOutput;
    backActuator->velocity = backVelocityOutput;
    //setSRX(frontActuator->controller, frontVelocityOutput);
    //setSRX(backActuator->controller, backVelocityOutput);

    // printf("Front Velocity: %f, Back Velocity: %f, Target Velocity: %f\n", frontVelocityOutput, backVelocityOutput, targetVelocity);
    // printf("targetVelocity: %f, time delta: %f\n", targetVelocity, currentTime - frontActuator->lastTime);

    frontActuator->prevPosition = frontPos;
    backActuator->prevPosition = backPos;
    frontActuator->prevVelocity = frontVelocityOutput;
    backActuator->prevVelocity = backVelocityOutput;
    frontActuator->lastTime = currentTime;
    backActuator->lastTime = currentTime;
}

PositionPacket_OneRobot calculatePulse(Actuator *frontActuator, Actuator *backActuator) 
{
    pulseCountFront = fmax(fmin(pulseCountFront, (int)(pulse_mm*254)), 0);
    frontActuator->prevPosition = map((int)(pulse_mm*254), 0, pulseCountFront);

    pulseCountBack = fmax(fmin(pulseCountBack, (int)(pulse_mm*254)), 0);
    backActuator->prevPosition = map((int)(pulse_mm*254), 0, pulseCountBack);

    PositionPacket_OneRobot pkt = Init_Position_Packet();
    pkt.front_actuator_position = frontActuator->prevPosition;
    pkt.back_actuator_position = backActuator->prevPosition;

    return pkt;
} 

void evaluatePot(Actuator *frontActuator, Actuator *backActuator) 
{
    readPot(frontActuator->pot);
    readPot(backActuator->pot);
}