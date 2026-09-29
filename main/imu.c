// Goal: Write code to 
//       1) Read values from the gyroscope x register for your IMU and print to the serial console
//       2) Turn on an LED
//       3) Allow a pushbutton to stop printing to the console and the LED to turn off when pressed
// Note: You only need the code for ONE IMU (what you are given). You will implement the code for the EITHER BNO055 OR the IMU3000.

#include "freeRTOS/FreeRTOS.h"
#include "i2c.h"
#include "driver/gpio.h"

// Step 1) Use the reference manuals for your assigned IMU to find the following device and register addresses.
// Additionally, use the peripheral pin mode table to select a gpio pin for the pushbutton and LED.
#define IMU3000_SENSOR_ADDR //define
#define IMU3000_GYROX_LSB //define
#define IMU3000_GYROX_MSB //define

#define BNO055_SENSOR_ADDR //define
#define BNO055_GYROX_LSB //define
#define BNO055_GRYOX_MSB //define
#define BNO055_OP_MODE //define

#define GPIO_BUTTON //define
#define LED_GPIO //define

i2c_master_bus_handle_t i2c_bus;

const uint8_t BNO055_OP_MODE_VALUE = 0x03;
const char* bno055_register_names[] = {
    "gyrox_lsb",
    "gyrox_msb",
    "opr_mode"
};
const uint8_t bno055_registers[] = {
    BNO055_GYROX_LSB,
    BNO055_GRYOX_MSB
};

// Step 2) Fill in the i2c_sensor_config_t and i2c_sensor_t structs for your assigned IMU
// Reference inc/i2c_sensor.h for help
i2c_sensor_config_t bno055_config = {

};
i2c_sensor_t bno055 = {

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
   
};
i2c_sensor_t imu3000 = {
   
};

void app_main(void)
{
    // Step 3) Write code to initialize the I2C bus and add your IMU as a sensor to the bus.
    // Refer to i2c.c and look at the I2C_Create_Bus and I2C_Add_Sensor functions for guidance.


    uint8_t imu3000_data[2];
    uint8_t bno055_data[2];
    int16_t result = 0;

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);
    gpio_pulldown_dis(GPIO_BUTTON);
    gpio_pullup_en(GPIO_BUTTON);
    gpio_set_level(LED_GPIO, 0);
    
    for (;;) {
       
       // Step 4) Write code to read the IMU data from the I2C bus.
       // Refer to the implementation of I2C_Read_Registers() in inc/i2c.h for guidance.
      
       if (err != ESP_OK) {
            printf("I2C read failed: %s\n", esp_err_to_name(err));
            continue;
       }
       
       // Step 5) Write code to translate the results produced by I2C_Read_Registers() into actual gyro values.
       // Refer to the register map for your IMU to understand the conversion.
       // For BNO055, look at page 57 of the reference manual in the activity doc.
       // For IMU3000, look at page 7 of the reference manual in the activity doc.
       result = //conversion from raw IMU data to actual gyro value;
       printf("Gyro X Register Value: %d\n\n", result);

       // Note: Pushbutton reads logic 0 when pressed.
       while(gpio_get_level(GPIO_BUTTON) == 0){
            printf("Stopped\n");
       }

       gpio_set_level(LED_GPIO, 1);
       vTaskDelay(pdMS_TO_TICKS(100));
       gpio_set_level(LED_GPIO, 0);
    }
}
