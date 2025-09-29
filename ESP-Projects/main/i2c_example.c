/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a IMU3000 inertial measurement unit.
*/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO GPIO_NUM_41 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_42 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0      /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define IMU3000_SENSOR_ADDR 0x68 /*!< Address of the MPU9250 sensor */
#define gyro_x_H 0x1d
#define gyro_x_L 0x1e
#define gyro_y_H 0x1f
#define gyro_y_L 0x20
#define gyro_z_H 0x21
#define gyro_z_L 0x22
#define IMU3000_WHO_AM_I_REG_ADDR 0x0 /*!< Register addresses of the "who am I" register */

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t imu3000_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        // configs for the i2c master bus
        .i2c_port = I2C_MASTER_NUM,      // sets the i2c port (either 0 or 1) to use for communication. only 2 for i2c comms on the esp32s3
        .sda_io_num = I2C_MASTER_SDA_IO, // configuring pins for sda and scl
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU3000_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void app_main(void)
{
    uint8_t data[2]; // high then low
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;

    uint8_t gyros[] = {gyro_x_H, gyro_x_L, gyro_y_H, gyro_y_L, gyro_z_H, gyro_z_L};
    const char *dimensions[] = {"X-axis", "Y-axis", "Z-axis"};

    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the IMU3000 WHO_AM_I register, on power up the register should have the value 0x71 */
    ESP_ERROR_CHECK(imu3000_register_read(dev_handle, IMU3000_WHO_AM_I_REG_ADDR, data, 1));
    printf("WHO_AM_I = %x\n", data[0]);
    printf("It should be 69 or 68.\n");
    for (uint8_t i = 3; i > 0; i--)
    {
        printf("Will start reading IMU gyro data in:\t%d seconds\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("\n");

    while (1)
    {
        for (uint8_t dims = 0; dims < 1; dims++)
        {
            ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &gyros[dims], 1, &data[0], 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
            ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &gyros[dims + 1], 1, &data[1], 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

            printf("%s\t-\t%d\n", dimensions[dims], ((int16_t)(data[0] << 8) | data[1]) / 0xFF);
            vTaskDelay(5);
        }
        printf("\n");
    }

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}