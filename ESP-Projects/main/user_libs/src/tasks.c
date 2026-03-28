#include "tasks.h"


void UART_rx_task()
{
    ControlPacket_ExcavationRobot pkt = {1, 0, 0, 0, 0, 0, 0, 0, 0};

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
    CurrVoltPacket_ExcavationRobot packet = Init_CurrVolt_Excavation_Robot_Packet();

    while (1)
    {
        packet.front_left_wheel = getChannelCurrentPDH(pdh, fxMotors[0]->channel);
        packet.back_left_wheel = getChannelCurrentPDH(pdh, fxMotors[1]->channel);
        packet.front_right_wheel = getChannelCurrentPDH(pdh, fxMotors[2]->channel);
        packet.back_right_wheel = getChannelCurrentPDH(pdh, fxMotors[3]->channel);
        packet.bucket_ladder = getChannelCurrentPDH(pdh, fxMotors[4]->channel);
        packet.conveyor_belt = getChannelCurrentPDH(pdh, fxMotors[5]->channel);

        packet.left_track_actuator = getChannelCurrentPDH(pdh, srxMotors[0]->channel);
        packet.right_track_actuator = getChannelCurrentPDH(pdh, srxMotors[1]->channel);

        packet.main_battery = (float)(getInputVoltagePDH(pdh));
        //Add AUX later
        UART_write(&packet);
        vTaskDelay(100);
    }
}

void excavation_robot_control_can_task()
{
    ControlPacket_ExcavationRobot motor_state = {0, 0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    ControlPacket_ExcavationRobot new_data;

    while (1)
    {
        if (xQueueReceive(control_queue, &new_data, 0) == pdTRUE)
        {
            motor_state = new_data;
            ESP_LOGI("CAN", "Motor Control:\t%d %d %d %d %d %d %d\n", motor_state.front_left_wheel, motor_state.back_left_wheel, motor_state.front_right_wheel, motor_state.back_right_wheel, motor_state.bucket_ladder, motor_state.conveyor_belt, motor_state.track_actuator);        
        }

        directControl(motor_state);
        vTaskDelay(pdMS_TO_TICKS(2));
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