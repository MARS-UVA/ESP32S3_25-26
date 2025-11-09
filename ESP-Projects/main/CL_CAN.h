#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"

#ifndef CL_CAN_H
#define CL_CAN_H

//FUNCTIONS
void talonPercentOut(int16_t speed);
void setPIDValues(const float *configs);
void setTargetVelocity(int16_t velocity);
void canSetup();

#endif // CL_CAN_H