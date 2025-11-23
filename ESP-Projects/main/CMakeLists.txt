<<<<<<< Updated upstream
idf_component_register(SRCS "FM_CurrentSetting.c" "hello_world.c"
                       INCLUDE_DIRS ".")

=======
idf_component_register(SRCS  "./testCurrentReading.c" "./user_libs/can/can2.c" "./user_libs/utils/utils.c" "./user_libs/can/pdp.c"
                       INCLUDE_DIRS "." "./user_libs/can" "./user_libs/utils"  "./user_libs/control")
>>>>>>> Stashed changes

