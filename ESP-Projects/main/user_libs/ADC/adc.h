#ifndef ADC_H
#define ADC_H

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"

typedef struct pot
{
	int minPos; // minimum ADC reading
	int maxPos; // maximum ADC reading
	float pos;
} Pot;

Pot potInit(adc_unit_t unit, adc_channel_t channel, int minPos, int maxPos);

void initalizePots();

void readPot();

void PotDeInit(Pot *pot);

#endif
