#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#include "uart.h"
#include "can.h"
#include "actuators.h"
#include "OneRobot.h"
#include "packets.h"

void readHallEffect(void *pvParameters)
{
    while (1)
    {
        PositionPacket_OneRobot packet =
            calculatePulse(&frontActuator, &backActuator);

        // Print or transmit position if needed

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void moveActuators(void *pvParameters)
{
    ControlPacket_OneRobot packet = {
        .front_left_wheel = 127,
        .front_right_wheel = 127,
        .front_bucket_drum = 127,
        .back_bucket_drum = 127,
        .front_actuator = 127,
        .back_actuator = 127,
    };

    // Explicitly stop both actuators
    directControl(packet);

    while (1)
    {
        // Low-speed test command for back actuator
        packet.back_actuator = 140;

        for (int i = 0; i < 5; i++)
        {
            directControl(packet);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // Stop actuator
        packet.back_actuator = 127;
        directControl(packet);

        // Wait before another test
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main(void)
{
    canSetup();
    initializeTalons();

    xTaskCreate(
        readHallEffect,
        "readHallEffect",
        4096,
        NULL,
        9,
        NULL
    );

    xTaskCreate(
        moveActuators,
        "moveActuators",
        4096,
        NULL,
        8,
        NULL
    );
}