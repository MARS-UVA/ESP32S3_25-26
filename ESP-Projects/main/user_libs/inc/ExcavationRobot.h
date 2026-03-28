#ifndef POWERLIB_CONTROL
#define POWERLIB_CONTROL

#include "can.h"
#include "uart.h"
#include "packets.h"

#include "talonFX.h"
#include "talonSRX.h"

//Excavation robot
// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 33
#define BACK_LEFT_WHEEL_ID 33
#define FRONT_RIGHT_WHEEL_ID 33
#define BACK_RIGHT_WHEEL_ID 33
#define BUCKET_LADDER_ID 33  // needs updating
#define CONVEYOR_BELT_ID 33
#define LEFT_TRACK_ACTUATOR_ID 14
#define RIGHT_TRACK_ACTUATOR_ID 11 // needs updating

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 12
#define BACK_LEFT_WHEEL_CHANNEL_ID 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 3
#define BACK_RIGHT_WHEEL_CHANNEL_ID 1
#define BUCKET_LADDER_CHANNEL_ID 0
#define CONVEYOR_BELT_CHANNEL_ID 2
#define LEFT_TRACK_ACTUATOR_CHANNEL_ID 15
#define RIGHT_TRACK_ACTUATOR_CHANNEL_ID 14

extern QueueHandle_t uart_queue;
void initializeTalons(PDH *pdh);
void directControl(ControlPacket_ExcavationRobot pkt);
#endif














































//start digging in yo butt twin

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
