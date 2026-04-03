#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "uart.h"
#include "can.h"
#include "actuators.h"
#include "OneRobot.h"
#include "packets.h"

extern Actuator leftActuator;
extern Actuator rightActuator;

void readHallEffect()
{
    while (1)
    {
        PositionPacket packet = calculatePulse(&leftActuator, &rightActuator);
        //UART_write_position(&packet);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void moveActuators()
{
    ControlPacket packet = {0, 0x0, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 200};
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(3000));
        packet.actuator = 255;
        for (int i = 0; i < 3*100; i += 1) {
            directControl(packet);
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
        packet.actuator = 0;
        for (int i = 0; i < 2.5*100; i += 1) {
            directControl(packet);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

void app_main()
{
    initializeTalons();
    //UART_setup();
    canSetup();

    //xTaskCreate(printActuatorPositions, "printActuatorPositions", 4096, NULL, 10, NULL);
    xTaskCreate(readHallEffect, "readHallEffect", 4096, NULL, 9, NULL);
    xTaskCreate(moveActuators, "moveActuators",  4096, NULL, 8, NULL);
    //xTaskCreate(printPulse, "printPulse",  4096, NULL, 8, NULL);
    //xTaskCreate(evaluteActuators, "evaluateActuators",  4096, NULL, 9, NULL);

    return;
}