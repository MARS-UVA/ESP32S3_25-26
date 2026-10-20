#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "can.h"
#include "actuators.h"  // Must declare TalonSRX, talonSRXInit(), setSRX().

/*
 * Replace these with your existing controller configuration.
 * Keep disabled until CAN protocol, stop behavior, Hall wiring,
 * and physical direction have been verified.
 */
#define RUN_MOTION_TEST       0
#define ACTUATOR_CAN_ID        1
#define ACTUATOR_CHANNEL_ID    0
#define ACTUATOR_INVERTED      false

#define TEST_OUTPUT           0.15
#define MOTION_MS             250
#define STOP_GAP_MS           2000
#define COMMAND_PERIOD_MS     20
#define FEEDBACK_PERIOD_MS    100

/*
 * REQUIRED adapters to your existing Hall/pulse-count implementation.
 *
 * hall_init_one():
 *   Initialize both quadrature inputs, count state, and ISR/PCNT.
 *   Return true only when initialization succeeds.
 *
 * hall_position_snapshot():
 *   Return a race-free snapshot of the SIGNED quadrature count.
 *   Use an ISR-safe critical section or the appropriate PCNT API.
 *   Do not infer measured direction from the motor command.
 */
extern bool hall_init_one(void);
extern int32_t hall_position_snapshot(void);

static TalonSRX actuator;

/* CAN status is drained here but not decoded into position. */
static bool can_rx_callback(twai_node_handle_t node,
                            const twai_rx_done_event_data_t *event,
                            void *context)
{
    (void)event;
    (void)context;

    uint8_t data[8];
    twai_frame_t frame = {
        .buffer = data,
        .buffer_len = sizeof(data),
    };

    (void)twai_node_receive_from_isr(node, &frame);
    return false;  // No higher-priority task was awakened.
}

static TickType_t command_delay_ticks(void)
{
    TickType_t ticks = pdMS_TO_TICKS(COMMAND_PERIOD_MS);
    return ticks > 0 ? ticks : 1;
}

/*
 * Reissue the command throughout a bounded phase.
 * A time deadline avoids treating loop iterations as milliseconds.
 */
static void run_phase(double output, uint32_t duration_ms,
                      const char *label)
{
    const int64_t deadline =
        esp_timer_get_time() + (int64_t)duration_ms * 1000;
    int64_t next_report = 0;

    while (esp_timer_get_time() < deadline) {
        setSRX(&actuator, output);

        int64_t now = esp_timer_get_time();
        if (now >= next_report) {
            printf("phase=%s command=%.2f position_ticks=%" PRId32 "\n",
                   label, output, hall_position_snapshot());
            next_report = now + (int64_t)FEEDBACK_PERIOD_MS * 1000;
        }

        vTaskDelay(command_delay_ticks());
    }

    /* A delay alone does not stop a motor controller. */
    setSRX(&actuator, 0.0);
}

static void actuator_test_task(void *argument)
{
    (void)argument;

    run_phase(0.0, STOP_GAP_MS, "startup_stop");

#if RUN_MOTION_TEST
    /*
     * Positive output is only ASSUMED to extend.
     * Verify this mechanically before enabling the test.
     */
    run_phase(+TEST_OUTPUT, MOTION_MS, "extend");
    run_phase(0.0, STOP_GAP_MS, "stop");
    run_phase(-TEST_OUTPUT, MOTION_MS, "retract");
#endif

    /* No automatic repetition of the motion test. */
    for (;;) {
        run_phase(0.0, 1000, "stopped");
    }
}

void app_main(void)
{
    /*
     * Configure ESP-IDF console output as UART, e.g. UART0 at
     * 115200 baud. printf() then provides plain-text UART feedback.
     * If using a separate UART, initialize it here and replace printf.
     */

    /* Struct initialization only: this does not configure the Talon. */
    actuator = talonSRXInit(ACTUATOR_CAN_ID,
                           ACTUATOR_CHANNEL_ID,
                           ACTUATOR_INVERTED);

    if (!hall_init_one()) {
        printf("Hall initialization failed; motion test not started.\n");
        return;
    }

    /* Your supplied canSetup requires both arguments. */
    canSetup(can_rx_callback, NULL);

    /* CAN must be initialized before the first command is sent. */
    setSRX(&actuator, 0.0);

    if (xTaskCreate(actuator_test_task, "actuator_test",
                    4096, NULL, 5, NULL) != pdPASS) {
        /* Remain alive and keep requesting stop on allocation failure. */
        for (;;) {
            setSRX(&actuator, 0.0);
            vTaskDelay(command_delay_ticks());
        }
    }
}