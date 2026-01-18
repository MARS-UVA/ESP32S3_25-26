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


typedef struct pot {
	adc_continuous_handle_t handle;
	//float(*read)(struct pot*);
	//float(*readCm)(struct pot*);
	uint32_t actuatorOffset;
	int minPos;				// minimum ADC reading
	int maxPos;				// maximum ADC reading
	float pos;
} Pot;

extern Pot pot;

void PotInit(adc_channel_t channel);

void readPot();

//void calibrateYourMom(Pot *leftPot, Pot *rightPot);

void PotDeInit(Pot* pot);

#endif
