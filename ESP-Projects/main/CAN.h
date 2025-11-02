#ifndef CL_CAN_H
#define CL_CAN_H

#include <esp_adc/adc_continuous.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/twai.h"

//LIBRARY CONSTANTS
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

//FUNCTIONS
void talonPercentOut(int16_t speed);
void setPIDValues();
void setTargetVelocity(int velocity);
void canSetup();

#endif // CL_CAN_H