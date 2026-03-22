#include "uart.h"
#include "packets.h"

void app_main()
{
    UART_setup();
    CurrVoltPacket_OneRobot packet = Init_CurrVolt_Packet();
    packet.front_left_wheel = 0.75f;
    packet.back_left_wheel = 0.75f;
    packet.front_right_wheel = 0.75f;
    packet.back_right_wheel = 0.75f;
    packet.front_drum = 0.50f;
    packet.back_drum = 0.50f;
    packet.front_actuator = 0.25f;
    packet.back_actuator = 0.25f;
    packet.main_battery = 0.25f;
    packet.aux_battery = 0.25f;    

    while (1) {
    UART_write(&packet, sizeof(packet));
    vTaskDelay(pdMS_TO_TICKS(300));
    }
};

    /*
    CurrVoltPacket_OneRobot

    uint8_t invalid;
    uint8_t header;
    uint8_t reserved_bit1;
    uint8_t reserved_bit2;

    float front_left_wheel;
    float back_left_wheel;
    float front_right_wheel;
    float back_right_wheel;
    float front_drum;
    float back_drum;
    float front_actuator;
    float back_actuator;
    float main_battery;
    float aux_battery;
    */
    