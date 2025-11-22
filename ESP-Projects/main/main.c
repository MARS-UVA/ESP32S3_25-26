#include "control_startup.c"

void app_main()
{
    initializeTalons();
    UART_setup();
    canSetup();

    SerialPacket packet;

    while (true)
    {
        packet = UART_read();
        directControl(packet);
        // setTargetFX(&frontLeft, 200);
    }
}