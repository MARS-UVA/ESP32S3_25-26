idf_component_register(SRCS  "./main.c" "./user_libs/can/can2.c" "./user_libs/utils/utils.c" "./test_code/can/can2_test.c" "./user_libs/uart/uart.c" "./user_libs/control/control_startup.c"
                       INCLUDE_DIRS "." "./user_libs/can" "./user_libs/utils" "./user_libs/uart" "./user_libs/control")

