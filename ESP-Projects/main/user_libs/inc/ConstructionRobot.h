#ifndef POWERLIB_CONTROL
#define POWERLIB_CONTROL

#include "can.h"
#include "uart.h"
#include "packets.h"

#include "talonFX.h"
#include "talonSRX.h"

//Excavation robot
// Define CAN IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_ID 38
#define BACK_LEFT_WHEEL_ID 13
#define FRONT_RIGHT_WHEEL_ID 36
#define BACK_RIGHT_WHEEL_ID 37
#define ACTUATOR_ID 35
#define VIBRATOR_ID 3  

// Define channel IDs of each motor/actuator
#define FRONT_LEFT_WHEEL_CHANNEL_ID 14
#define BACK_LEFT_WHEEL_CHANNEL_ID 13
#define FRONT_RIGHT_WHEEL_CHANNEL_ID 16
#define BACK_RIGHT_WHEEL_CHANNEL_ID 15
#define ACTUATOR_CHANNEL_ID 12
#define VIBRATOR_CHANNEL_ID 17

extern QueueHandle_t uart_queue;
void initializeTalons(PDH *pdh);
void directControl(ControlPacket_ConstructionRobot pkt);
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
  the last four digits of lukes ssn is 0596
*/
