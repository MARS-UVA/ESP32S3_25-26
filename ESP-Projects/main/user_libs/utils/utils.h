#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <memory.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include <stdbool.h>
#include <esp_err.h>

void floatToByteArray(float f, char *arr);
float map(int min, int max, int pos);
void swapEndian(uint8_t *arr, uint8_t len);
void writeToBuffInd(uint8_t *dst, uint8_t *src, uint8_t ind, uint8_t len);
void showData(uint8_t *arr, uint8_t len);
void ledSetup(uint8_t gpio);
void ledToggle(uint8_t gpio, bool *state);

#endif