#ifndef POWERLIB_CONTROL
#define POWERLIB_CONTROL

#include "can.h"
#include "uart.h"
#include "packets.h"

#include "talonFX.h"
#include "talonSRX.h"

// Excavation robot
//  Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 38
#define BACK_LEFT_WHEEL_ID 13
#define FRONT_RIGHT_WHEEL_ID 36
#define BACK_RIGHT_WHEEL_ID 37
#define BUCKET_LADDER_ID 27 // needs updating
#define CONVEYOR_BELT_ID 59
#define LEFT_TRACK_ACTUATOR_ID 2
#define RIGHT_TRACK_ACTUATOR_ID 4 // needs updating

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 14
#define BACK_LEFT_WHEEL_CHANNEL_ID 17
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 12
#define BACK_RIGHT_WHEEL_CHANNEL_ID 13
#define BUCKET_LADDER_CHANNEL_ID 19
#define CONVEYOR_BELT_CHANNEL_ID 18
#define LEFT_TRACK_ACTUATOR_CHANNEL_ID 15
#define RIGHT_TRACK_ACTUATOR_CHANNEL_ID 16

extern QueueHandle_t uart_queue;
void initializeTalons(PDH *pdh);
void directControl(ControlPacket_ExcavationRobot pkt);
void test_run_motor();
TempPacket_ExcavationRobot getTemperatureExcavationRobot();
CurrVoltPacket_ExcavationRobot getCurrentVoltageExcavationRobot(PDH *pdh);
void canSetupRobot(PDH *pdh, TalonFX **motors, size_t count);

// STURCT
typedef struct
{
  PDH *pdh;
  TalonFX **motors;
  size_t count;
} RobotRegistry;

#endif

// start digging in yo butt twin

/*
     ⣀⣤⣴⣶⣶⣾⣿⣷⣶⣶⣦⣄⡀⠀⠀⠀
⠀⢠⣴⣿⣿⣿⣿⣿⣭⣭⣭⣭⣭⣿⣿⣿⣿⣧⣀⠀
⢰⣿⣿⣿⣿⣿⣯⣿⡶⠶⠶⠶⠶⣶⣭⣽⣿⣿⣷⣆
⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⠈⢿⣿⣿⡿⠋⠉⠁⠈⠉⠛⠉⠀⠀⠀⠈⠻⣿⡿⠃
⠀⠀⠀⠉⠁⠀⢴⣐⢦⠀⠀⠀⣴⡖⣦⠀⠀⠈⠀⠀
⠀⠀⠀⠀⠀⠀⠈⠛⠋⠀⠀⠀⠈⠛⠁⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⣀⡀⠀⠀⠀⣀⠀⠀⠀⢀⡀⠀⠀⠀⠀
⠀⠀⢀⡔⣻⣭⡇⠀⣼⣿⣿⣿⡇⠦⣬⣟⢓⡄⠀⠀
⠀⠀⠀⠉⠁⠀⠀⠀⣿⣿⣿⣿⡇⠀⠀⠉⠉⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠻⠿⠿⠟⠁⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⣠⢼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⡄⠀⠀⠀
⠀⠀⣀⣤⣴⣾⣿⣷⣭⣭⣭⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⡀⠀⠀
⠀⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣸⣿⣿⣧⠀⠀

  /\_/\                                                     /\_/\
 ( o.o )                                                   ( ^.^ )
  > ^ <                                                     > ^ <

   _                           _
  (_)                         | |
   _  __ _ _   _ _ __ __ _  __| |_   _
  | |/ _` | | | | '__/ _` |/ _` | | | |
  | | (_| | |_| | | | (_| | (_| | |_| |
  | |\__,_|\__, |_|  \__,_|\__,_|\__, |
 _/ |       __/ |                 __/ |
|__/       |___/                 |___/

           _           _
          | |         | |
          | |__   __ _| |_ ___  ___
          | '_ \ / _` | __/ _ \/ __|
          | | | | (_| | ||  __/\__ \
          |_| |_|\__,_|\__\___||___/

           _    _ _   _
          | |  (_) | | |
          | | ___| |_| |_ ___ _ __  ___
          | |/ / | __| __/ _ \ '_ \/ __|
          |   <| | |_| ||  __/ | | \__ \
          |_|\_\_|\__|\__\___|_| |_|___/

  ♡*+.♡*+.♡*+.♡*+.♡*+.♡*+.♡*+.♡*+.♡*+.♡*+.♡*+.♡
*/
