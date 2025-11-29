#include "utils.h"
#include "uart.h"
#include "can2.h"

void app_main()
{
    int G = 32;
    int R = 27;
    int B = 12;
    bool state = false;
    UART_setup();
    ledSetup(G);
    ledSetup(R);
    ledSetup(B);

    xTaskCreate(UART_rx_task, "uart_rx_task", 1024, NULL, 3, NULL);
    xTaskCreate(UART_can_task, "uart_can_task", 1024, NULL, 7, NULL);
    while (1)
    {
        ledToggle(G, &state);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    return;
}
