#include "utils.h"
#include "can2.h"
#include "pdp.h"

uint8_t prompt_buff[6] = {0x00, 0x00, 0x00, 0x00, 0x20, 0x00};      // buffer to prompt PDP for current readings
twai_frame_t promptChannel_msg = {
    .header.id = 0x8041640 | 10,       // Message ID
    .header.ide = true,                // Use 29-bit extended ID format
    .buffer = prompt_buff,             // Pointer to data to transmit
    .buffer_len = sizeof(prompt_buff), // Length of data to transmit
};

void app_main()
{
    PDPInit(10);
    float current;
    TalonFX fx = talonFXInit(27);
    TalonSRX srx = talonSRXInit(4, false);
    printf("Can Setup\n");
    canSetup();
    printf("Can Setup Done\n");
    for (;;)
    {
        sendEn();
      //setSRX(&srx, 0.1);
        setFX(&fx, 0.1);
        ESP_ERROR_CHECK(twai_node_transmit(g_node_hdl, &promptChannel_msg, -1)); // Timeout = 0: returns immediately if queue is full
        //requestCurrentReadingsPDP();
        //current = getChannelCurrentPDP(13);
        //printf("\nCurrent read at channel %d -\t %.3f", 13, current);
    }
}
