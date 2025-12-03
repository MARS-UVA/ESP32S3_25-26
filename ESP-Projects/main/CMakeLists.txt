idf_component_register(SRCS  "main.c" "./user_libs/can/can2.c" "./user_libs/utils/utils.c" "./user_libs/uart/uart.c" "./user_libs/can/pdp.c" "./user_libs/wifi/wifi.c"
                       INCLUDE_DIRS "." "./user_libs/can" "./user_libs/utils" "./user_libs/uart" "./user_libs/wifi/")

