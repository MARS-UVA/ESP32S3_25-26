/**
 * @file utils.h
 * @brief Utility functions for the ESP32
 * 
 * @author Diana Lin <xrc9wg@virginia.edu>
 * @author Carlos Giron <rdb7fq@virginia.edu>
 * @author Anthony Vu <anthonyvu@email.virginia.edu>
 * @version 1.0
 * @date    2025-11-01
 * @copyright Copyright (c) 2025 Mechatronics and Robotics Society
 */

#pragma once

#include <stdio.h>
#include <memory.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include <stdbool.h>
#include <esp_err.h>
#include "esp_event.h"
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

//  TODO: Write documentation for these functions

/**
 * 
 */
void floatToByteArray(float f, char *arr);

/**
 * 
 */
float map(int min, int max, int pos);

/**
 * 
 */
void swapEndian(uint8_t *arr, uint8_t len);

/**
 * 
 */
void writeToBuffInd(uint8_t *dst, uint8_t *src, uint8_t ind, uint8_t len);

/**
 * 
 */
void showData(uint8_t *arr, uint8_t len);

/**
 * 
 */
void ledSetup(uint8_t gpio);

/**
 * 
 */
void ledToggle(uint8_t gpio, bool *state);

/**
 * @brief Extract a contiguous bit-field from a 64-bit value.
 *
 * Bits are numbered starting from bit 0 (least-significant bit).
 *
 * @param data      Source 64-bit value.
 * @param startBit  Index of first bit to extract.
 * @param bitLength Number of bits to extract.
 * @return Extracted value, right-aligned.
 */
uint32_t extractBits(uint64_t data, uint8_t startBit, uint8_t bitLength);