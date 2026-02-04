/*
 * util.c
 *
 *  Created on: Nov 2025
 *      Author: diana, carlos
 */
#include "utils.h"

// convert a float to an array of 4 bytes
void floatToByteArray(float f, char *arr)
{
    unsigned int asInt = *((int *)&f);

    for (int i = 0; i < 4; i++)
        arr[i] = (asInt >> 8 * i) & 0xFF;
}
// map a value "pos" from a range of "min" to "max" to a float between 0 and 1
float map(int min, int max, int pos)
{
    return ((float)(pos - min)) / (max - min);
}
// swaps endianness of incomming byte array
void swapEndian(uint8_t *arr, uint8_t len)
{
    uint8_t tmp;
    for (uint8_t i = 0; i < len / 2; i++)
    {
        tmp = arr[i];
        arr[i] = arr[len - 1 - i];
        arr[len - 1 - i] = tmp;
    }
}
void writeToBuffInd(uint8_t *dst, uint8_t *src, uint8_t ind, uint8_t len)
{
    memcpy(dst + ind, src, len);
}
void showData(uint8_t *arr, uint8_t len)
{
    printf("\nData = {");
    for (int i = 0; i < len - 1; i++)
    {
        printf("0x%02d, ", *(arr + i));
    }
    printf("0x%02x}\n", *(arr + len - 1));
}

void ledSetup(uint8_t gpio)
{
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
}

void ledToggle(uint8_t gpio, bool *state)
{
    gpio_set_level(gpio, *state);
    *state = !(*state);
}