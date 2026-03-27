#pragma once

#include "esp_adc/adc_continuous.h"
#include "utils.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "sdkconfig.h"

typedef struct
{
    int minPos; // minimum ADC reading
	int maxPos; // maximum ADC reading
    adc_channel_t channel;
	double pos;
} Pot;

void potSetup(adc_channel_t *channel, uint8_t channel_num);

Pot potInit(int minPos, int maxPos, adc_channel_t channel);

void readPot(Pot *pot);