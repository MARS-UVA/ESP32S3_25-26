idf_component_register(SRCS "construction_robot_main.c" "./user_libs/src/ConstructionRobot.c" "./user_libs/src/" "./user_libs/src/tasks.c" "./user_libs/src/can.c" "./user_libs/src/talonFX.c" "./user_libs/src/talonSRX.c" "./user_libs/src/pdh.c" "./user_libs/src/utils.c" "./user_libs/src/uart.c" "./user_libs/src/packets.c"
                       INCLUDE_DIRS "." "./user_libs/inc")
