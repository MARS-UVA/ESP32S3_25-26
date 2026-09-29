// Includes
#include "freeRTOS/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "driver/gpio.h"

// Defines
#define IMU3000_SENSOR_ADDR 0x68
#define IMU3000_GYROX_LSB 0x1E
#define IMU3000_GYROX_MSB 0x1D

#define BNO055_SENSOR_ADDR 0x28
#define BNO055_GYROX_LSB 0x14
#define BNO055_GRYOX_MSB 0x15
#define BNO055_OP_MODE 0x3D

#define GPIO_BUTTON GPIO_NUM_7
#define LED_GPIO GPIO_NUM_4

// Function Prototypes
void vIMUPrintTask(void *pvParameters);
void vButtonTask(void *pvParameters);
void vLEDBlinkTask(void *pvParameters);

// Task Handles
TaskHandle_t imuTaskHandle;
TaskHandle_t buttonTaskHandle;
TaskHandle_t ledTaskHandle;

// I2C Bus and Sensor Structs
i2c_master_bus_handle_t i2c_bus;

const uint8_t BNO055_OP_MODE_VALUE = 0x03;
const char* bno055_register_names[] = {
    "gyrox_lsb",
    "gyrox_msb",
    "opr_mode"
};
const uint8_t bno055_registers[] = {
    BNO055_GYROX_LSB,
    BNO055_GRYOX_MSB,
};
i2c_sensor_config_t bno055_config = {
    .name = "bno055",
    .address = BNO055_SENSOR_ADDR,
    .register_names = bno055_register_names,
    .registers = bno055_registers,
    .register_count = 2,
};
i2c_sensor_t bno055 = {
    .config = &bno055_config,
};

const char* imu3000_register_names[] = {
    "gyrox_lsb",
    "gyrox_msb"
};
const uint8_t imu3000_registers[] = {
    IMU3000_GYROX_LSB,
    IMU3000_GYROX_MSB
};
i2c_sensor_config_t imu3000_config = {
    .name = "imu3000",
    .address = IMU3000_SENSOR_ADDR,
    .register_names = imu3000_register_names,
    .registers = imu3000_registers,
    .register_count = 2,
};
i2c_sensor_t imu3000 = {
    .config = &imu3000_config,
};



void app_main(void)
{
    I2C_Create_Bus(&i2c_bus);
    I2C_Add_Sensor(&i2c_bus, &imu3000);
    //I2C_Add_Sensor(&i2c_bus, &bno055);
    //I2C_Write_Register(&bno055, BNO055_OP_MODE, &BNO055_OP_MODE_VALUE, 1);


    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);
    gpio_pulldown_dis(GPIO_BUTTON);
    gpio_pullup_en(GPIO_BUTTON);
    

    gpio_set_level(LED_GPIO, 0);

    xTaskCreate(vIMUPrintTask, "vIMUPrintTask", 2048, NULL, 5, &imuTaskHandle);
    xTaskCreate(vButtonTask, "vButtonTask", 2048, NULL, 1, &buttonTaskHandle);
    xTaskCreate(vLEDBlinkTask, "vLEDBlinkTask", 2048, NULL, 5, &ledTaskHandle);
}

void vIMUPrintTask(void *pvParameters)
{
    while (1) {
        // Read IMU data and print to console
        uint8_t imu3000_data[2];
        uint8_t bno055_data[2];
        int16_t result = 0;
        esp_err_t err = I2C_Read_Registers(&imu3000, imu3000_data);
        if (err != ESP_OK) {
            printf("I2C read failed: %s\n", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        result = (imu3000_data[1] << 8) | imu3000_data[0];
        printf("IMU3000 Gyro X: %d\n\n", result);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vButtonTask(void *pvParameters)
{
    while(1){
        while (gpio_get_level(GPIO_BUTTON) == 0) {
            printf("Button pressed\n");
            gpio_set_level(LED_GPIO, 0);
            vTaskSuspend(imuTaskHandle);
            vTaskSuspend(ledTaskHandle);
        }

        vTaskResume(imuTaskHandle);
        vTaskResume(ledTaskHandle);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void vLEDBlinkTask(void *pvParameters)
{
    while (1) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
