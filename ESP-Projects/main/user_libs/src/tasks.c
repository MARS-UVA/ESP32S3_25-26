#include "tasks.h"

extern QueueHandle_t control_queue; // queue stores the control packet values
extern QueueHandle_t temperature_queue;
extern QueueHandle_t currvolt_queue;
extern QueueHandle_t position_queue;

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
        // Add AUX later
        UART_write(&packet);
        vTaskDelay(100);
    }
}

void temperature_update_task()
{
    for (;;)
    {
        TempPacket_ExcavationRobot temp_packet = getTemperatureExcavationRobot();
        ESP_LOGI("Temperature", "Temperature: \t %f %f %f %f", temp_packet.front_left_wheel_temp, temp_packet.front_right_wheel_temp, temp_packet.back_left_wheel_temp, temp_packet.back_right_wheel_temp);
        xQueueOverwrite(temperature_queue, &temp_packet);
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
            // ESP_LOGI("CAN", "Motor Control:\t%d %d %d %d %d %d %d\n", motor_state.front_left_wheel, motor_state.back_left_wheel, motor_state.front_right_wheel, motor_state.back_right_wheel, motor_state.bucket_ladder, motor_state.conveyor_belt, motor_state.track_actuator);
        }

        sendEn();
        directControl(motor_state);
        // ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(g_node_hdl, TIMEOUT));
        vTaskDelay(pdMS_TO_TICKS(6));
    }
}

void current_voltage_update_task(PDH *pdh)
{

    while (1)
    {
        CurrVoltPacket_ExcavationRobot current_voltage_packet = getCurrentVoltageExcavationRobot(pdh);
        ESP_LOGI("Current", "Current: \t %f %f %f %f", current_voltage_packet.front_left_wheel, current_voltage_packet.front_right_wheel, current_voltage_packet.back_left_wheel, current_voltage_packet.back_right_wheel);
        xQueueOverwrite(currvolt_queue, &current_voltage_packet);
        vTaskDelay(100);
    }
}

void motor_task()
{
    for (;;)
    {
        // sendEn();
        test_run_motor();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
