#include "uart.c"
#include "packets.h"

void app_main()
{
    UART_setup();
    CurrVoltPacket_OneRobot packet =  {
        .front_left_wheel = 0.75f,
        .back_left_wheel = 0.75f,
        .front_right_wheel = 0.75f,
        .back_right_wheel = 0.75f,
        .front_drum = 0.50f,
        .back_drum = 0.50f,
        .front_actuator = 0.25f,
        .back_actuator = 0.25f,
        .main_battery = 0.25f,
        .aux_battery = 0.25f     
    };
    UART_write(uint8_t *packet, sizeof(packet));
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
    