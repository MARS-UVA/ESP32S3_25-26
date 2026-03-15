#pragma once

#include "packets.h"

#define CONTROL_PACKET_ID 0x00
#define CURRENT_BUS_VOLTAGE_PACKET_ID 0x01
#define TEMP_PACKET_ID 0x02
#define POSITION_PACKET_ID 0x03

// Excavation Robot

CurrVoltPacket_ExcavationRobot Init_CurrVolt_Excavation_Robot_Packet()
{
    CurrVoltPacket_ExcavationRobot pkt;
    pkt.invalid = 0xFF;
    pkt.header = CURRENT_BUS_VOLTAGE_PACKET_ID;
    pkt.reserved_bit1 = 0x00;
    pkt.reserved_bit2 = 0x00;

    pkt.front_left_wheel = 0.0f;
    pkt.front_right_wheel = 0.0f;
    pkt.back_left_wheel = 0.0f;
    pkt.back_right_wheel = 0.0f; 
    pkt.bucket_ladder = 0.0f;
    pkt.conveyor_belt = 0.0f; 
    pkt.left_track_actuator = 0.0f;
    pkt.right_track_actuator = 0.0f;
    pkt.main_battery = 0.0f;
    pkt.aux_battery = 0.0f;    
    return pkt;
}

TempPacket_ExcavationRobot Init_Temp_Excavation_Robot_Packet()
{
    TempPacket_ExcavationRobot pkt;
    pkt.invalid = 0xFF;
    pkt.header = TEMP_PACKET_ID;
    pkt.reserved_bit1 = 0x00;
    pkt.reserved_bit2 = 0x00;

    pkt.front_left_wheel_temp = 0.0f;
    pkt.front_right_wheel_temp = 0.0f;
    pkt.back_left_wheel_temp = 0.0f;
    pkt.back_right_wheel_temp = 0.0f; 
    pkt.bucket_ladder_temp = 0.0f;
    pkt.conveyor_belt_temp = 0.0f; 
    return pkt;
}

PositionPacket_ExcavationRobot Init_Position_Excavation_Robot_Packet()
{
    PositionPacket_ExcavationRobot pkt;
    pkt.invalid = 0xFF;
    pkt.header = POSITION_PACKET_ID;
    pkt.reserved_bit1 = 0x00;
    pkt.reserved_bit2 = 0x00;

    pkt.left_track_actuator_position = 0.0f;
    pkt.right_track_actuator_position = 0.0f;
    return pkt;
}