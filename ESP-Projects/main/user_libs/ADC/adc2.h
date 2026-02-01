#ifndef ADC2_H
#define ADC2_H

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"

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

#endif