#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"


#include "uart.h"
#include "control_startup.h"
#include "can.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "talonFX.h"
#include "OneRobot.h"
#include "tasks.h"
#include "pdh.h"


#define I2C_MASTER_SCL_IO      GPIO_NUM_9
#define I2C_MASTER_SDA_IO      GPIO_NUM_10
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     100000
#define I2C_TIMEOUT_MS         100


#define INA219_ADDR             0x40
#define INA219_REG_BUS_VOLTAGE  0x02

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_ina219_dev = NULL;

/* ---------------- bus + device setup ---------------- */
static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, /* ~45k, weak - add 4.7k-10k external if bus is flaky */
    };
    return i2c_new_master_bus(&bus_cfg, &s_bus);
}

static i2c_master_dev_handle_t s_ina219_dev;

// Attach INA219 to the already-initialized I2C bus
static esp_err_t ina219_attach(void)
{
    esp_err_t err = i2c_master_probe(
        s_bus, INA219_ADDR, I2C_TIMEOUT_MS
    );

    if (err != ESP_OK) {
        ESP_LOGE("INA219", "No response at 0x40: %s",
                 esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA219_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    return i2c_master_bus_add_device(
        s_bus, &dev_cfg, &s_ina219_dev
    );
}

// Read bus voltage in volts
static esp_err_t ina219_read_voltage(float *voltage)
{
    uint8_t reg = INA219_REG_BUS_VOLTAGE;
    uint8_t data[2];

    esp_err_t err = i2c_master_transmit_receive(
        s_ina219_dev,
        &reg, 1,
        data, 2,
        I2C_TIMEOUT_MS
    );

    if (err != ESP_OK) {
        return err;
    }

    // INA219 sends most-significant byte first
    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];

    // Bits 15:3 = bus voltage, 4mV per count
    *voltage = (raw >> 3) * 0.004f;

    return ESP_OK;
}


void app_main(void)
{
    

    // Initialize I2C_Bus_init
    ESP_ERROR_CHECK(i2c_bus_init());
    
    ESP_ERROR_CHECK(ina219_attach());
    

    vTaskDelay(pdMS_TO_TICKS(10));

    while (1) {

        float voltage = 0.0f;

        esp_err_t err = ina219_read_voltage(&voltage);

        if (err == ESP_OK) {
            printf("INA219 Bus Voltage: %.3f V\n", voltage);
        } else {
            ESP_LOGW("INA219", "Voltage read failed: %s",
                    esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(100));

    }
}