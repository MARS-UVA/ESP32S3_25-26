idf_component_register(SRCS "one_robot_main.c" "./one_robot_main.c" "./user_libs/src/OneRobot.c" "./user_libs/src/tasks.c" "./user_libs/src/can.c" "./user_libs/src/talonFX.c" "./user_libs/src/talonSRX.c" "./user_libs/src/pdh.c" "./user_libs/src/utils.c" "./user_libs/src/uart.c" "./user_libs/src/packets.c" "./user_libs/src/actuators.c" "./user_libs/src/pid.c" "./user_libs/src/adc.c" "./user_libs/src/i2c.c"
                       INCLUDE_DIRS "." "./user_libs/inc")

