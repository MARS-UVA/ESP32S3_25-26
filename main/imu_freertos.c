// Goal: Write code to 
//       1) Read values from the gyroscope x register for your IMU and print to the serial console
//       2) Turn on an LED
//       3) Allow a pushbutton to stop printing to the console and the LED to turn off when pressed
//       4) Essentially, you will be doing the same thing as the previous part, only using FreeRTOS and tasks this time.
// Note: You only need the code for ONE IMU (what you are given). You will implement the code for the EITHER BNO055 OR the IMU3000.

// Includes
#include "freeRTOS/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "driver/gpio.h"


// Step 1)  Fill in the correct device addresses for the IMU3000 and BNO055 sensors from the previous part.
// Defines
#define IMU3000_SENSOR_ADDR //define
#define IMU3000_GYROX_LSB //define
#define IMU3000_GYROX_MSB //define

#define BNO055_SENSOR_ADDR //define
#define BNO055_GYROX_LSB //define
#define BNO055_GRYOX_MSB //define
#define BNO055_OP_MODE //define

#define GPIO_BUTTON //define
#define LED_GPIO //define

// Function Prototypes for each functionality's associated task (implement later on in file)
void vIMUPrintTask(void *pvParameters);
void vButtonTask(void *pvParameters);
void vLEDBlinkTask(void *pvParameters);

// Task Handles (will be used for suspending tasks)
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
// Step 2) Paste in your sensor config and sensor structs from the previous part.
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
    // Step 3) Paste in your code from the previous section to initialize the I2C bus and add your IMU as a sensor to the bus.
    // Define your imu3000_data or bno055_data arrays and 16 bit result integer in IMUPrintTask


    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_BUTTON, GPIO_MODE_INPUT);
    gpio_pulldown_dis(GPIO_BUTTON);
    gpio_pullup_en(GPIO_BUTTON);
    gpio_set_level(LED_GPIO, 0);

    // Step 4) Call xTaskCreate to create the tasks for IMU printing, button handling, and LED blinking.
    // Refer to the FreeRTOS documentation for guidance on creating tasks here: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html#tasks
    // Note: You do NOT need to put your calls to xTaskCreate inside a loop.
    // When creating tasks...
    //    1) Put in the name of the task function in the second parameter of xTaskCreate as a string
    //    2) Assign priority levels of 5 to the IMU print and LED blink tasks, and 1 to the button task.
    //    3) Provide a stack size of 2048 for each task.
    //    4) Pass NULL as the fourth parameter (pvParameters) since we are not passing any parameters to the tasks.

}

void vIMUPrintTask(void *pvParameters)
{
    while (1) {
        // Step 5) Implement the IMU print task logic here.
        // In this task, you should print the IMU gyro X register value to the serial console.
        // Adapt your code for the same functionality in the previous section here.
        // At the end of the while(1) loop, add a task delay of pdMS_TO_TICKS(100).

    }
}

void vButtonTask(void *pvParameters)
{
    while(1){
        // Step 6) Implement the pushbutton halt logic here.
        // In this task, you should check the state of the pushbutton (0 = pushed) and halt both the IMU print and LED blink tasks if it is pressed.
        // In your pushbutton state check loop...
        //    1) Print "Button pressed\n" to the monitor menu
        //    2) Turn the LED off.
        //    3) Call vTaskSuspend() on the IMU print and LED blink task handles.
        // After this while loop check, call vTaskResume() on the IMU print and LED blink task handles to resume their execution.
        // At the end of the while(1) loop, add a task delay of pdMS_TO_TICKS(20).
        
    }
}

void vLEDBlinkTask(void *pvParameters)
{
    while (1) {
        // Step 7) Implement the LED blink task logic here.
        // In this task, you should toggle the LED state with a delay of 500 ms between each toggle.

    }
}
