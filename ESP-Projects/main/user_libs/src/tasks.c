#include "tasks.h"


void UART_rx_task()
{
    ControlPacket_OneRobot pkt = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    while (1)
    {
        UART_read(&pkt);
        if (pkt.invalid == 0)
        {
            // OutPacket packet = {pkt.invalid, pkt.header, pkt.top_left_wheel, pkt.back_left_wheel, pkt.top_right_wheel, pkt.back_right_wheel, pkt.drum, pkt.drum, pkt.actuator, pkt.actuator};
            // UART_write(&packet);
            xQueueOverwrite(control_queue, &pkt);
            pkt.invalid = 1; // Mark invalid so packet is not reused
        }
        vTaskDelay(1);
    }
}

void UART_tx_task() //
{
    CurrVoltPacket_OneRobot packet = {0, 1, 0, 1, 0, 0, 1, 0, 0, 0};

    while (1)
    {
        /*packet.invalid = 0;
        packet.header = 0x0;

        packet.top_left_wheel = fxMotors[0]->current;
        packet.back_left_wheel = fxMotors[1]->current;
        packet.top_right_wheel = fxMotors[2]->current;
        packet.back_right_wheel = fxMotors[3]->current;
        packet.bucket_left = fxMotors[4]->current;
        packet.bucket_right = fxMotors[5]->current;

        packet.left_actuator = srxMotors[0]->current;
        packet.right_actuator = srxMotors[1]->current;
        */

        UART_write(&packet);
        vTaskDelay(100);
    }
}

void one_robot_control_can_task()
{
    ControlPacket_OneRobot motor_state = {0, 0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    ControlPacket_OneRobot new_data;

    while (1)
    {
        if (xQueueReceive(control_queue, &new_data, 0) == pdTRUE)
        {
            motor_state = new_data;
        }
        directControl(motor_state);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void current_update_task(PDP *pdp)
{
    int delaytime_ms = 1000;
    while (1)
    {
        int sem_wait_time_ms = 100;
        requestCurrentReadingsPDP(pdp);
        bool recvd = awaitCurrentReadingsPDP(pdp, sem_wait_time_ms);

        if (!recvd)
        {
            vTaskDelay(delaytime_ms);
            continue;
        }

        for (uint8_t i = 0; i < 6; i++)
        {
            // requestCurrentReadingsPDP();
            fxMotors[i]->current = getChannelCurrentPDP(pdp, fxMotors[i]->channel);
            // fxMotors[i]->current = 1.0;
            // vTaskDelay(1);
        }
        for (uint8_t i = 0; i < 2; i++)
        {
            // requestCurrentReadingsPDP();
            srxMotors[i]->current = getChannelCurrentPDP(pdp, srxMotors[i]->channel);
            // srxMotors[i]->current = 2.0;
            // vTaskDelay(1);
        }
        vTaskDelay(delaytime_ms);
        // ESP_LOGI("CURRENT TEST", "Current:\t%.3f\n", fxMotors[0]->current);
    }
}