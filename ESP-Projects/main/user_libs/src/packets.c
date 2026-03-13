#pragma once

#include "packets.h"

#define CONTROL_PACKET_ID 0x00
#define CURRENT_BUS_VOLTAGE_PACKET_ID 0x01
#define TEMP_PACKET_ID 0x02
#define POSITION_PACKET_ID 0x03

CurrVoltPacket_OneRobot Init_CurrVolt_Packet()
{
    CurrVoltPacket_OneRobot pkt;
    pkt.invalid = 0xFF;
    pkt.header = CURRENT_BUS_VOLTAGE_PACKET_ID;
    pkt.reserved_bit1 = 0x00;
    pkt.reserved_bit2 = 0x00;

    pkt.front_left_wheel = 0.0f;
    pkt.front_right_wheel = 0.0f;
    pkt.back_left_wheel = 0.0f;
    pkt.back_right_wheel = 0.0f; 
    pkt.front_drum = 0.0f;
    pkt.back_drum = 0.0f; 
    pkt.front_actuator = 0.0f;
    pkt.back_actuator = 0.0f;
    pkt.main_battery = 0.0f;
    pkt.aux_battery = 0.0f;    
    return pkt;
}

TempPacket_OneRobot Init_Temp_Packet()
{
    TempPacket_OneRobot pkt;
    pkt.invalid = 0xFF;
    pkt.header = TEMP_PACKET_ID;
    pkt.reserved_bit1 = 0x00;
    pkt.reserved_bit2 = 0x00;

    pkt.front_left_wheel_temp = 0.0f;
    pkt.front_right_wheel_temp = 0.0f;
    pkt.back_left_wheel_temp = 0.0f;
    pkt.back_right_wheel_temp = 0.0f; 
    pkt.front_drum_temp = 0.0f;
    pkt.back_drum_temp = 0.0f; 
    return pkt;
}

PositionPacket_OneRobot Init_Position_Packet()
{
    PositionPacket_OneRobot pkt;
    pkt.invalid = 0xFF;
    pkt.header = POSITION_PACKET_ID;
    pkt.reserved_bit1 = 0x00;
    pkt.reserved_bit2 = 0x00;

    pkt.front_actuator_position = 0.0f;
    pkt.back_actuator_position = 0.0f;
    return pkt;
}