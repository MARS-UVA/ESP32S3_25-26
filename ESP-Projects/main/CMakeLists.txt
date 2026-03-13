idf_component_register(SRCS "one_robot_main.c" "./user_libs/src/OneRobot.c" "./user_libs/src/" "./user_libs/src/tasks.c"
 "./user_libs/src/can.c" "./user_libs/src/talonFX.c" "./user_libs/src/pdh.c" 
 "./user_libs/src/utils.c" "./user_libs/src/talonSRX.c" "./user_libs/src/pdp.c" "./user_libs/src/uart.c"
                       INCLUDE_DIRS "." "./user_libs/inc")

