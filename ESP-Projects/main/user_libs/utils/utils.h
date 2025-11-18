#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>
#include <memory.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

void floatToByteArray(float f, char *arr);
float map(int min, int max, int pos);
void swapEndian(uint8_t *arr, uint8_t len);
void writeToBuffInd(uint8_t *dst, uint8_t *src, uint8_t ind, uint8_t len);
void showData(uint8_t *arr, uint8_t len);

#endif