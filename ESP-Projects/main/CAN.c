#include "CAN.h"

/* --------------------- Definitions and static variables ------------------ */

twai_message_t enable_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame if 0
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN enable frame
    .identifier = 0x401bf,
    .data_length_code = 2,
    .data = {0x01, 0x00},
};

twai_message_t drive_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN set Output frame
    .identifier = 0x204b540 | 0x1b,
    .data_length_code = 8,
    .data = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

twai_message_t neutral_mode_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN set Output frame
    .identifier = 0x2047c00 | 0x1b,
    .data_length_code = 8,
    .data = {0x21, 0x6E, 0x08, 0x00, 0x00, 0x00, 0x00, 0xAA},

};

twai_message_t set_PID_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN set Output frame
    .identifier = 0x2047c00,
    .data_length_code = 8,
    .data = {0x21, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xAA},

};

twai_message_t apply_PID_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN set Output frame
    .identifier = 0x2047c00 | 0x1b,
    .data_length_code = 8,
    .data = {0x10, 0x0c, 0xc5, 0x06, 0x0d, 0x00, 0x00, 0x00},

};
twai_message_t set_Target_Velocity_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN set Output frame
    .identifier = 0x2043700 | 0x1b,
    .data_length_code = 8,
    .data = { 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },

};

twai_message_t supplyCurrentLimit_msg = {
    // Message type and format settings
    .extd = 1,         // Standard Format message (29-bit ID)
    .rtr = 0,          // Send a data frame
    .ss = 0,           // Not single shot
    .self = 0,         // Message is not a self reception request (loopback)
    .dlc_non_comp = 0, // DLC is less than 8

    // Message ID and payload for CAN set Output frame
    .identifier = 0x2047c00 | 0x1b,
    .data_length_code = 8,
    .data = {0x21, 0x72, 0x08, 0x00, 0x00, 0x00, 0x00, 0xAA},
};

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

        ESP_LOGI(CAN_LOG, "PID set");


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
    vTaskDelay(1);
}

void canSetup(void){
    // Install and start TWAI driver for CAN
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CAN_LOG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CAN_LOG, "Driver started");

    ESP_ERROR_CHECK(twai_transmit(&enable_msg, portMAX_DELAY));
}

void canStop(void){

    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(CAN_LOG, "CAN drivers stopped");

}

