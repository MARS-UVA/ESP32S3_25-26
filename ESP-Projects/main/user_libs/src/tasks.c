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

void UART_tx_task(PDH *pdh) //
{
    CurrVoltPacket_OneRobot packet = Init_CurrVolt_Packet();
    TempPacket_OneRobot temperature_packet;

    while (1)
    {
        printf("Current value at channel 3: %.2f\n", getChannelCurrentPDH(pdh, 2));
        packet.front_left_wheel = getChannelCurrentPDH(pdh, fxMotors[0]->channel);
        packet.back_left_wheel = getChannelCurrentPDH(pdh, fxMotors[1]->channel);
        packet.front_right_wheel = getChannelCurrentPDH(pdh, fxMotors[2]->channel);
        packet.back_right_wheel = getChannelCurrentPDH(pdh, fxMotors[3]->channel);
        packet.front_drum = getChannelCurrentPDH(pdh, fxMotors[4]->channel);
        packet.back_drum = getChannelCurrentPDH(pdh, fxMotors[5]->channel);

        packet.front_actuator = getChannelCurrentPDH(pdh, srxMotors[0]->channel);
        packet.back_actuator = getChannelCurrentPDH(pdh, srxMotors[1]->channel);

        packet.main_battery = (float)(getInputVoltagePDH(pdh));
        
        //updateAuxVoltage();
        //packet.aux_battery = getAuxVoltage();

        UART_write(&packet);

        if (xQueueReceive(temperature_queue, &temperature_packet, 0) == pdTRUE)
        {
            printf("Debug: temperature: %d\n", temperature_packet.front_left_wheel_temp);
            UARTWriteTemperature(&temperature_packet);
        }
        vTaskDelay(100);
    }
}

void temperature_update_task()
{
    for (;;)
    {
        TempPacket_OneRobot temp_packet = getTemperatureOneRobot();
        xQueueOverwrite(temperature_queue, &temp_packet);
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

void current_update_task(PDH *pdh)
{

    while (1)
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            fxMotors[i]->current = getChannelCurrentPDH(pdh, fxMotors[i]->channel);
            // fxMotors[i]->current = 1.0;
            // vTaskDelay(1);
        }
        for (uint8_t i = 0; i < 2; i++)
        {
            srxMotors[i]->current = getChannelCurrentPDH(pdh, srxMotors[i]->channel);
            // pdh->channelCurrents[i + 6] = getChannelCurrentPDH(pdh, srxMotors[i]->channel);
            // srxMotors[i]->current = 2.0;
            // vTaskDelay(1);
        }
        vTaskDelay(1000);
        // ESP_LOGI("CURRENT TEST", "Current:\t%.3f\n", fxMotors[0]->current);
    }
}

void motor_task()
{
    for (;;)
    {
            
        test_run_motor();
        vTaskDelay(1);
    }
}


// OLD CURRENT UPDATE TASK USING PDP, NEW ONE IN PDH.C
/**
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
    **/