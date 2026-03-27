idf_component_register(SRCS "send_position_test.c" "./user_libs/src/can.c" "./user_libs/src/talonFX.c" "./user_libs/src/talonSRX.c" "./user_libs/src/OneRobot.c" "./user_libs/src/utils.c" "./user_libs/src/uart.c" "./user_libs/src/actuators.c" "./user_libs/src/pid.c"
                       INCLUDE_DIRS "." "./user_libs/inc")

