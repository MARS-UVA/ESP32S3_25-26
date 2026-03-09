/*
 * @file    util.c
 * @brief   Utility functions for the ESP32
 * 
 * @author Diana Lin <xrc9wg@virginia.edu>
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Anthony Vu <anthonyvu@email.virginia.edu>
 * @version 1.0
 * @date    2025-11-01
 * @copyright Copyright (c) 2025 Mechatronics and Robotics Society
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

uint32_t extractBits(uint64_t data, uint8_t startBit, uint8_t bitLength)
{
    if (bitLength == 0 || bitLength > 32 || startBit >= 64)
        return 0;

    uint64_t mask = (1ULL << bitLength) - 1ULL;
    return (uint32_t)((data >> startBit) & mask);
}