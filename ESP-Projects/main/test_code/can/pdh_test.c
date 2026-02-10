#include "utils.h"
#include "can2.h"
#include "pdh.h"

void app_main()
{
    PDHInit(62);
    float current;
    TalonFX fx = talonFXInit(33, 1);
    printf("Can Setup\n");
    canSetup();
    printf("Can Setup Done\n");

    for (;;)
    {
        sendEn();
        setFX(&fx, 0.4);
        // ESP_ERROR_CHECK(twai_node_transmit(g_node_hdl, &promptChannel_msg, -1)); // Timeout = 0: returns immediately if queue is full
        current = getChannelCurrentPDH(1);
        // printstuff();
        printf("Curret at channel %d:\t%f\n", fx.channel, current);
    }
}
