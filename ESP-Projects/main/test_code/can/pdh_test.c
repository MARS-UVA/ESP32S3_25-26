#include "utils.h"
#include "can2.h"
#include "pdh.h"


void app_main()
{
    PDHInit(2);
    float current;
    TalonFX fx = talonFXInit(1, 0);
    printf("Can Setup\n");
    canSetup();
    printf("Can Setup Done\n");


    for (;;)
    {
        sendEn();
        setFX(&fx, 0.5);
        //ESP_ERROR_CHECK(twai_node_transmit(g_node_hdl, &promptChannel_msg, -1)); // Timeout = 0: returns immediately if queue is full
        current = getChannelCurrentPDH(0);
        printf("\nCurrent read at channel %d -\t %.3f", 0, current);
    }
}
