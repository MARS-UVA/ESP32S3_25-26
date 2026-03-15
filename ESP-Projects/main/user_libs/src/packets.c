#pragma once

#include "packets.h"

#define CONTROL_PACKET_ID 0x00
#define CURRENT_BUS_VOLTAGE_PACKET_ID 0x01
#define TEMP_PACKET_ID 0x02
#define POSITION_PACKET_ID 0x03

// Construction Robot

CurrVoltPacket_ConstructionRobot Init_CurrVolt_Construction_Robot_Packet()
{
    CurrVoltPacket_ConstructionRobot pkt;
    pkt.invalid = 0xFF;
    pkt.header = CURRENT_BUS_VOLTAGE_PACKET_ID;
    pkt.reserved_bit1 = 0x00;
    pkt.reserved_bit2 = 0x00;

    pkt.front_left_wheel = 0.0f;
    pkt.front_right_wheel = 0.0f;
    pkt.back_left_wheel = 0.0f;
    pkt.back_right_wheel = 0.0f; 
    pkt.actuator = 0.0f;
    pkt.main_battery = 0.0f;
    return pkt;
}

TempPacket_ConstructionRobot Init_Temp_Construction_Robot_Packet()
{
    TempPacket_ConstructionRobot pkt;
    pkt.invalid = 0xFF;
    pkt.header = TEMP_PACKET_ID;
    pkt.reserved_bit1 = 0x00;
    pkt.reserved_bit2 = 0x00;

    pkt.front_left_wheel_temp = 0.0f;
    pkt.front_right_wheel_temp = 0.0f;
    pkt.back_left_wheel_temp = 0.0f;
    pkt.back_right_wheel_temp = 0.0f; 
    return pkt;
}

PositionPacket_ConstructionRobot Init_Position_Construction_Robot_Packet()
{
    PositionPacket_ConstructionRobot pkt;
    pkt.invalid = 0xFF;
    pkt.header = POSITION_PACKET_ID;
    pkt.reserved_bit1 = 0x00;
    pkt.reserved_bit2 = 0x00;

    pkt.actuator_position = 0.0f;
    return pkt;
}