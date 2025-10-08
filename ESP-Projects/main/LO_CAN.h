#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/twai.h"

#ifndef LO_CAN_H
#define LO_CAN_H

void talonPercentOut(int16_t speed);
void canSetUp(void);

#endif
