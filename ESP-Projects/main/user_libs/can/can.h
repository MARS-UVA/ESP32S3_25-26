#ifndef CAN_H
#define CAN_H

#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/twai.h"


//LIBRARY CONSTANTS

#define CAN_LOG "CAN_LOG"

// setting rx and tx pins into can transciever
#define RX_GPIO_NUM GPIO_NUM_1
#define TX_GPIO_NUM GPIO_NUM_2



//FUNCTIONS
void talonPercentOut(int16_t speed);
void setPIDValues();
void setTargetVelocity(int velocity);
void canSetup();
void canStop();


#endif // CAN_H