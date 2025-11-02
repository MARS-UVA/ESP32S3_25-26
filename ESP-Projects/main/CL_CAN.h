#ifndef CL_CAN_H
#define CL_CAN_H

#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/twai.h"

//FUNCTIONS
void talonPercentOut(int16_t speed);
void setPIDValues();
void setTargetVelocity(int velocity);
void canSetup();

#endif // CL_CAN_H