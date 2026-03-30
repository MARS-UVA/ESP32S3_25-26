#include "tasks.h"


void UART_rx_task()
{
    ControlPacket_ConstructionRobot pkt = {1, 0, 0, 0, 0, 0, 0, 0};

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

void UART_tx_task(PDH *pdh)
{
    CurrVoltPacket_ConstructionRobot packet = Init_CurrVolt_Construction_Robot_Packet();

    while (1)
    {
        packet.front_left_wheel = getChannelCurrentPDH(pdh, fxMotors[0]->channel);
        packet.back_left_wheel = getChannelCurrentPDH(pdh, fxMotors[1]->channel);
        packet.front_right_wheel = getChannelCurrentPDH(pdh, fxMotors[2]->channel);
        packet.back_right_wheel = getChannelCurrentPDH(pdh, fxMotors[3]->channel);
        
        packet.actuator = getChannelCurrentPDH(pdh, srxMotors[0]->channel);

        packet.main_battery = (float)(getInputVoltagePDH(pdh));
        UART_write(&packet);
        vTaskDelay(100);
    }
}

void construction_robot_control_can_task()
{
    ControlPacket_ConstructionRobot motor_state = {0, 0, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f};
    ControlPacket_ConstructionRobot new_data;

    while (1)
    {
        if (xQueueReceive(control_queue, &new_data, 0) == pdTRUE)
        {
            motor_state = new_data;
        }
        sendEn();
        directControl(motor_state);
        //ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(g_node_hdl, TIMEOUT));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void current_update_task(PDH *pdh)
{

    while (1)
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            pdh->channelCurrents[i] = getChannelCurrentPDH(pdh, fxMotors[i]->channel);
            // fxMotors[i]->current = 1.0;
            // vTaskDelay(1);
        }
        for (uint8_t i = 0; i < 2; i++)
        {
            pdh->channelCurrents[i + 6] = getChannelCurrentPDH(pdh, srxMotors[i]->channel);
            // srxMotors[i]->current = 2.0;
            // vTaskDelay(1);
        }
        vTaskDelay(1000);
        // ESP_LOGI("CURRENT TEST", "Current:\t%.3f\n", fxMotors[0]->current);
    }
}
