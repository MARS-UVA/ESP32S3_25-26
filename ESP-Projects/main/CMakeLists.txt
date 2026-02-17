idf_component_register(SRCS "main.c" "./main.c" "./user_libs/wifi/wifi.c" "./user_libs/can/can2.c" "./user_libs/utils/utils.c"  "./user_libs/uart/uart.c" "./user_libs/control/control_startup.c" "./user_libs/can/pdp.c" "./user_libs/ADC/adc2.c" "./user_libs/pid/pid.c" "./user_libs/pid/actuators.c"
                       INCLUDE_DIRS "." "./user_libs/can" "./user_libs/utils" "./user_libs/uart" "./user_libs/control" "./user_libs/wifi" "./user_libs/pid")

