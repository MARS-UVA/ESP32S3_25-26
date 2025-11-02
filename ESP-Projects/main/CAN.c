#include "CL_CAN.h"

/* --------------------- Definitions and static variables ------------------ */

// setting rx and tx pins into can transciever
#define RX_GPIO_NUM GPIO_NUM_1
#define TX_GPIO_NUM GPIO_NUM_2
#define EXAMPLE_TAG "TWAI Set Output"

// sets conifigs for our twai driver to send messages
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();     // CAN bus timing on our talon fx
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Accept all incoming messages for the driver
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);


void setPIDValues()
{
    set_PID_msg.data[1] = 0x53; // kP
    //kP = 3dcccccd = 0.1
    set_PID_msg.data[3] = 0x3d;
    set_PID_msg.data[4] = 0xcc;
    set_PID_msg.data[5] = 0xcc;
    set_PID_msg.data[6] = 0xcd;
    twai_transmit(&set_PID_msg, portMAX_DELAY);
    twai_transmit(&apply_PID_msg, portMAX_DELAY);

    set_PID_msg.data[1] = 0x54; // kI
    //kI = 3dcccccd = 0.1
    set_PID_msg.data[3] = 0x3d;
    set_PID_msg.data[4] = 0xcc;
    set_PID_msg.data[5] = 0xcc;
    set_PID_msg.data[6] = 0xcd;
    twai_transmit(&set_PID_msg, portMAX_DELAY);
    twai_transmit(&apply_PID_msg, portMAX_DELAY);


    set_PID_msg.data[1] = 0x55; // kD
    //kD = 0 
    twai_transmit(&set_PID_msg, portMAX_DELAY);
    twai_transmit(&apply_PID_msg, portMAX_DELAY);

        ESP_LOGI(EXAMPLE_TAG, "PID set");


}

void setTargetVelocity(int velocity)
{
    ESP_ERROR_CHECK(twai_transmit(&enable_msg, portMAX_DELAY));

    if (velocity >= 0) 
    {
    velocity *= 2;
    velocity *= 16;
    }
    else
    {
    velocity *= 2;
    velocity = 0x40000 - (-16 * velocity);
    }
    set_Target_Velocity_msg.data[2] = velocity & 0xFF; // high byte
    set_Target_Velocity_msg.data[3] = (velocity >> 8) & 0xFF; // low byte
    set_Target_Velocity_msg.data[4] = (velocity >> 16) & 0xFF;
    set_Target_Velocity_msg.data[6] = (velocity >> 16) & 0x0a;
    set_Target_Velocity_msg.data[7] = (velocity >> 16) & 0x00;


    twai_transmit(&set_Target_Velocity_msg, portMAX_DELAY);   
    twai_clear_receive_queue();
}

// function to drive a motor at speed/1024 percent output
void talonPercentOut(int16_t speed)
{
    ESP_ERROR_CHECK(twai_transmit(&enable_msg, portMAX_DELAY));

    uint8_t spBytes[2];
    spBytes[0] = (speed >> 8) & 0x0f; // high byte
    spBytes[1] = (speed & 0xff);      // low byte

    drive_msg.data[7] = spBytes[0]; // flipping endianness
    drive_msg.data[6] = spBytes[1];

    ESP_ERROR_CHECK(twai_transmit(&drive_msg, portMAX_DELAY));
}

void canSetup(void){
    // Install and start TWAI driver for CAN
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    ESP_ERROR_CHECK(twai_transmit(&enable_msg, portMAX_DELAY));
}