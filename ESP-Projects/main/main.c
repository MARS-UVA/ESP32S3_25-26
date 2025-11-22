#include "control_startup.c"

void app_main()
{
    initializeTalons();
    UART_setup();
    canSetup(g_node_hdl);

    SerialPacket packet;

    while (true) {   
        // packet = UART_read(); 

        // directControl(packet);

        setTargetFX(g_node_hdl, &frontLeft, 200);
    }
}