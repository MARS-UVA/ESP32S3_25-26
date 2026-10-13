
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

// ---------- I2C Configuration ----------
#define I2C_MASTER_SCL_IO      GPIO_NUM_9
#define I2C_MASTER_SDA_IO      GPIO_NUM_10
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     100000
#define I2C_TIMEOUT_MS         100

// ---------- INA219 Configuration ----------
#define INA219_ADDR             0x40
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE   0x02

// ---------- UART Configuration ----------
#define UART_PORT               UART_NUM_1
#define UART_TX_PIN             GPIO_NUM_17
#define UART_RX_PIN             GPIO_NUM_18
#define UART_BAUD_RATE          9600

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_ina219_dev = NULL;

// ---------- UART Initialization ----------
static esp_err_t uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(
        UART_PORT, 2048, 2048, 0, NULL, 0
    );
    if (err != ESP_OK) return err;

    err = uart_param_config(UART_PORT, &uart_config);
    if (err != ESP_OK) return err;

    return uart_set_pin(
        UART_PORT,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
}

// ---------- UART Transmit ----------
static void uart_send(const char *message)
{
    uart_write_bytes(UART_PORT, message, strlen(message));
}

// ---------- I2C Initialization ----------
static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(&bus_cfg, &s_bus);
}

// ---------- INA219 Initialization ----------
static esp_err_t ina219_attach(void)
{
    esp_err_t err = i2c_master_probe(
        s_bus, INA219_ADDR, I2C_TIMEOUT_MS
    );

    if (err != ESP_OK) {
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

// ---------- Read 16-bit INA219 Register ----------
static esp_err_t ina219_read_register(
    uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    esp_err_t err = i2c_master_transmit_receive(
        s_ina219_dev,
        &reg, 1,
        data, 2,
        I2C_TIMEOUT_MS
    );

    if (err != ESP_OK) return err;

    *value = ((uint16_t)data[0] << 8) | data[1];

    return ESP_OK;
}

// ---------- Main ----------
void app_main(void)
{
    char message[200];

    ESP_ERROR_CHECK(uart_init());

    uart_send("INA219 Voltage Sensor Starting...\r\n");

    ESP_ERROR_CHECK(i2c_bus_init());

    esp_err_t err = ina219_attach();

    if (err != ESP_OK) {
        snprintf(message, sizeof(message),
                 "INA219 not detected: %s\r\n",
                 esp_err_to_name(err));
        uart_send(message);
        return;
    }

    uart_send("INA219 detected at 0x40\r\n");

    while (1) {

        uint16_t raw_bus = 0;
        uint16_t raw_shunt = 0;

        esp_err_t bus_err = ina219_read_register(
            INA219_REG_BUS_VOLTAGE, &raw_bus
        );

        esp_err_t shunt_err = ina219_read_register(
            INA219_REG_SHUNT_VOLTAGE, &raw_shunt
        );

        if (bus_err == ESP_OK && shunt_err == ESP_OK) {

            // Bus voltage: 4mV per count
            float bus_voltage =
                (raw_bus >> 3) * 0.004f;

            // Shunt voltage: signed, 10uV per count
            int16_t signed_shunt = (int16_t)raw_shunt;

            float shunt_voltage =
                signed_shunt * 0.00001f;

            // Approximate voltage at VIN+
            float input_voltage =
                bus_voltage + shunt_voltage;

            // Status bits
            int conversion_ready = (raw_bus >> 1) & 1;
            int overflow = raw_bus & 1;

            snprintf(message, sizeof(message),
                "Bus: %.3f V | VIN+: %.3f V | "
                "Shunt: %.3f mV | Raw: 0x%04X | "
                "CNVR: %d | OVF: %d\r\n",
                bus_voltage,
                input_voltage,
                shunt_voltage * 1000.0f,
                raw_bus,
                conversion_ready,
                overflow
            );

            uart_send(message);

        } else {
            snprintf(message, sizeof(message),
                "INA219 read error: bus=%s shunt=%s\r\n",
                esp_err_to_name(bus_err),
                esp_err_to_name(shunt_err)
            );
            uart_send(message);
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
