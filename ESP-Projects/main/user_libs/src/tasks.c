#include "tasks.h"

void UART_rx_task()
{
    ControlPacket_OneRobot pkt = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    while (1)
    {
        UART_read(&pkt);
        if (pkt.invalid == 0)
        {
            xQueueOverwrite(control_queue, &pkt);
            pkt.invalid = 1; // Mark invalid so packet is not reused
        }
        vTaskDelay(1);
    }
}

void UART_tx_task()
{
    // Declare packets that will be sent to the Jetson
    CurrVoltPacket_OneRobot current_voltage_packet;
    TempPacket_OneRobot temperature_packet;
    PositionPacket_OneRobot position_packet;

    // Continuously check for new packets in the queues and send them over UART
    for (;;)
    {
        if (xQueueReceive(current_voltage_queue, &current_voltage_packet, 0) == pdTRUE)
        {
            UART_write(&current_voltage_packet, sizeof(CurrVoltPacket_OneRobot));
        }

        if (xQueueReceive(position_queue, &position_packet, 0) == pdTRUE)
        {
            UART_write(&position_packet, sizeof(PositionPacket_OneRobot));
        }

        if (xQueueReceive(temperature_queue, &temperature_packet, 0) == pdTRUE)
        {
            UART_write(&temperature_packet, sizeof(TempPacket_OneRobot));
        }
        vTaskDelay(10);
    }
}

void enable_task() {
    while (1) {
        sendEn();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void current_voltage_update_task(PDH *pdh)
{
    // Continuously read current/voltage data from the robot and update the current_voltage_queue
    for (;;)
    {
        CurrVoltPacket_OneRobot current_voltage_packet = getCurrentVoltageOneRobot(pdh);
        xQueueOverwrite(current_voltage_queue, &current_voltage_packet);
        vTaskDelay(100);
    }
}

void position_update_task()
{
    // Continuously read position data from the robot and update the position_queue
    for (;;)
    {
        PositionPacket_OneRobot position_packet = calculatePulse(&frontActuator, &backActuator);
        xQueueOverwrite(position_queue, &position_packet);
        vTaskDelay(100);
    }
}

void temperature_update_task()
{
    // Continuously read temperature data from the robot and update the temperature_queue
    for (;;)
    {
        TempPacket_OneRobot temperature_packet = getTemperatureOneRobot();
        xQueueOverwrite(temperature_queue, &temperature_packet);
        vTaskDelay(100);
    }
}

void one_robot_control_can_task()
{
    ControlPacket_OneRobot motor_state = {0, 0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    // ControlPacket_OneRobot motor_state = {0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x7F, 0x7F}; // test packet with max speed forward and no actuator movement
    ControlPacket_OneRobot new_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(15);

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, frequency);
        if (xQueueReceive(control_queue, &new_data, 0) == pdTRUE)
        {
            motor_state = new_data;
        }
        directControl(motor_state);
    }
}

void CAN_enable_task()
{
    for (;;)
    {
        sendEn();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// TODO: Remove this function after testing
void motor_task()
{
    for (;;)
    {
        // sendEn();
        test_run_motor();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
