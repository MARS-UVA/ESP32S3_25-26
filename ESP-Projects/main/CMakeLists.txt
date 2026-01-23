idf_component_register(SRCS  "./adc_test.c" "./user_libs/ADC/adc.c" "./user_libs/can/can2.c" "./user_libs/utils/utils.c"  "./user_libs/uart/uart.c" "./user_libs/control/control_startup.c"
                       INCLUDE_DIRS "." "./user_libs/ADC" "./user_libs/can" "./user_libs/utils" "./user_libs/uart" "./user_libs/control" "./test_code/basic")

