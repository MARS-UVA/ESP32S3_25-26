#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//LIBRARY CONSTANTS

#define I2C_LOG "I2C_LOG"
// basic constants for setting up i2c drivers
#define I2C_MASTER_SCL_IO       GPIO_NUM_41		/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO       GPIO_NUM_42		/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM          I2C_NUM_0		/*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ      100000			/*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS   1000			/*!< I2C master timeout (ms) */

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//STRUCTS

typedef struct {
	const char* name;				// A human readable name for the device (used for debugging)
    const uint16_t address;			// 7-bit I2C Address not including the read/write bit.
	const char** register_names;	// Array of names for the registers (use for debugging)
    const uint8_t* registers;		// Array of register addresses for the device to read
    const size_t register_count;	// The number of registers to read from
} i2c_sensor_config_t;

typedef struct {
	const i2c_sensor_config_t* config; // Pointer to the static config
	i2c_master_dev_handle_t handle;	// The handle used to represent the device
} i2c_sensor_t;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//FUNCTIONS

esp_err_t I2C_Create_Bus(i2c_master_bus_handle_t *bus_handle);
esp_err_t I2C_Delete_Bus(i2c_master_bus_handle_t *bus_handle);
esp_err_t I2C_Add_Sensor(i2c_master_bus_handle_t* bus_handle, i2c_sensor_t* sensor);
esp_err_t I2C_Remove_Sensor(i2c_sensor_t* sensor);

esp_err_t I2C_Read_Register(i2c_sensor_t* sensor, size_t index, uint8_t* result);
esp_err_t I2C_Read_Registers(i2c_sensor_t* sensor, uint8_t result[]);
esp_err_t I2C_Burst_Read_Register(i2c_sensor_t* sensor, size_t index, uint8_t result[], size_t len);

esp_err_t _I2C_Read_Register_(i2c_master_dev_handle_t* handle, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t _I2C_Write_Register_(i2c_master_dev_handle_t* handle, uint8_t reg_addr, const uint8_t *data, size_t len);
esp_err_t _I2C_Write_Read_Register_(i2c_master_dev_handle_t* handle, const uint8_t* wdata, size_t wlen, uint8_t* rdata, size_t rlen);
esp_err_t _I2C_Write_Byte_(i2c_master_dev_handle_t* handle, const uint8_t *data);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#endif // I2C_H