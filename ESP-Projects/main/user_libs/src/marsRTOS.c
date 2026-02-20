#include "marsRTOS.h"

void UART_rx_task()
{
    SerialPacket pkt = {1, 0, 0, 0, 0, 0, 0, 0};

    while (1)
    {
        UART_read(&pkt);
        if (pkt.invalid == 0)
        {
            // OutPacket packet = {pkt.invalid, pkt.header, pkt.top_left_wheel, pkt.back_left_wheel, pkt.top_right_wheel, pkt.back_right_wheel, pkt.drum, pkt.drum, pkt.actuator, pkt.actuator};
            // UART_write(&packet);
            xQueueOverwrite(uart_queue, &pkt);
            pkt.invalid = 1; // Mark invalid so packet is not reused
        }
        vTaskDelay(1);
    }
}