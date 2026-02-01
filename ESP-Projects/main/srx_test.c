#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <uart.h>
#include "can2.h"
#include "adc2.h"

TalonSRX motor;
TalonSRX motor2;

Pot pot1;
Pot pot2;

#define LEFT_CHANNEL ADC_CHANNEL_9
#define RIGHT_CHANNEL ADC_CHANNEL_7

static adc_channel_t channel[2] = {LEFT_CHANNEL, RIGHT_CHANNEL};
static uint8_t channel_num = 2;

void runSRX()
{
    while (1)
    {
        setSRX(&motor, 1);
        setSRX(&motor2, 1);
    }
}

void readPots()
{
    while (1)
    {
        readPot(&pot1);
        readPot(&pot2);
    }
}

void app_main()
{
    canSetup();
    potSetup(channel, channel_num);

    //motor = talonSRXInit(4, 9, true);
    //motor2 = talonSRXInit(4, 9, false);

    pot1 = potInit(1190, 3153, LEFT_CHANNEL);
    pot2 = potInit(1190, 3153, RIGHT_CHANNEL);

    //xTaskCreate(runSRX, "runSRX", 4096, NULL, 8, NULL);
    xTaskCreate(readPots, "readPot", 4096, NULL, 9, NULL);
}