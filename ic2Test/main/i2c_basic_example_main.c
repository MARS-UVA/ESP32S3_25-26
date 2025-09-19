/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <time.h>

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO           2    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           1       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency (100kHz) */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000   

#define MPU9250_SENSOR_ADDR         0x68        /*!< Address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR   0x00       /*!< Register addresses of the "who am I" register */
#define MPU9250_X_GYRO_H            0x1D       /*!< Register address of X_GYRO_H */
#define MPU9250_X_GYRO_L            0x1E       /*!< Register address of X_GYRO_L */
#define MPU9250_Y_GYRO_H            0x1F       /*!< Register address of Y_GYRO_H */
#define MPU9250_Y_GYRO_L            0x20       /*!< Register address of Y_GYRO_L */
#define MPU9250_Z_GYRO_H            0x21       /*!< Register address of Z_GYRO_H */
#define MPU9250_Z_GYRO_L            0x22       /*!< Register address of Z_GYRO_L */
#define CLK_SEL                     0x3E       /*!< Register address of CLK_SEL */
#define SMPLRT_DIV                  0x19       /*!< Register address of SMPLRT_DIV */
int timer = 100;

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void app_startup(void){
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    mpu9250_register_write_byte(dev_handle, CLK_SEL, 0x04); // Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
    mpu9250_register_write_byte(dev_handle, SMPLRT_DIV, 0x01); // Set Gyro sample rate to 1kHz/(1+7) = 125Hz
}

void app_main(void)
{
    uint8_t data[2];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
    ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Read GYRO_H and GYRO_L registers */
    while(1) {
        ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_X_GYRO_H, &data[0], 1));
        ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_X_GYRO_L, &data[1], 1));
        uint16_t speedX = ((int16_t)data[0] << 8) | data[1];

        ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_Y_GYRO_H, &data[0], 1));
        ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_Y_GYRO_L, &data[1], 1));
        uint16_t speedY = ((int16_t)data[0] << 8) | data[1];

        ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_Z_GYRO_H, &data[0], 1));
        ESP_ERROR_CHECK(mpu9250_register_read(dev_handle, MPU9250_Z_GYRO_L, &data[1], 1));
        uint16_t speedZ = ((int16_t)data[0] << 8) | data[1];

        ESP_LOGI(TAG, "X_GYRO = %d", (speedX/timer) - 650);
        ESP_LOGI(TAG, "Y_GYRO = %d", (speedY/timer) - 2);
        ESP_LOGI(TAG, "Z_GYRO = %d", (speedZ/timer) - 654);

        vTaskDelay(pdMS_TO_TICKS(timer));
    }

    /* Demonstrate writing by resetting the MPU9250 */
    
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}