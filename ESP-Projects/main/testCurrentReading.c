#include "utils.h"
#include "can2.h"
#include "freertos/FreeRTOS.h"
#include "pdp.h"
#include <stdio.h>
#include "esp_log.h"

uint8_t send_buff[8] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
twai_frame_t forward_msg = {
    .header.id = 0x204b540 | 27,     // Message ID
    .header.ide = true,              // Use 29-bit extended ID format
    .buffer = send_buff,             // Pointer to data to transmit
    .buffer_len = sizeof(send_buff), // Length of data to transmit
};

void app_main(void)
{
    PDPInit(10);
    canSetup();
    TalonFX motor = talonFXInit(27);
    float current;

    while (1)
    {
        setTargetFX(&motor, 60);
        requestCurrentReadingsPDP();
        current = getChannelCurrentPDP(14);
        printf("\nCurrent Reading from PDP Channel 14: %.3f Amps", current);
    }
}