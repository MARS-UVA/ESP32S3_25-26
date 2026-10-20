/* Standalone adaptation of the supplied project; do not link its libraries too.
 * Hall feedback uses ONE rising-edge input and COMMAND-INFERRED direction,
 * matching the original approach. This is NOT a quadrature decoder.
 */
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RUN_MOTION_TEST 0       /* Set to 1 only after checking README.md. */
#define TX_GPIO_NUM GPIO_NUM_1
#define RX_GPIO_NUM GPIO_NUM_2
#define HALL_PIN GPIO_NUM_11    /* Original FRONT input; GPIO12 was BACK. */
#define ACTUATOR_ID 16          /* Original front Talon, NOT confirmed hardware. */
#define ACTUATOR_CHANNEL_ID 13  /* Stored metadata; not used in setSRX frames. */
#define ACTUATOR_INVERTED false
#define EXTEND_SIGN (-1)        /* Original front command convention; VERIFY. */
#define TEST_OUTPUT 0.15
#define MOTION_MS 250
#define STOP_GAP_MS 2000
#define COMMAND_PERIOD_MS 10
#define CAN_WAIT_MS 10
#define CAN_ID_SET_SRX 0x2040200u

typedef struct {
    uint8_t id;
    bool inverted;
    uint8_t channel;
    float current;
} TalonSRX;

static twai_node_handle_t g_node_hdl;
static TalonSRX actuator;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t position_ticks;
static uint64_t edges, unassigned_edges;
static int count_direction;
static bool tx_failed, faulted;

/* Persistent, immutable after preparation: safe even if CAN times out. */
static uint8_t payload[3][8];
static twai_frame_t frames[3]; /* STOP, EXTEND, RETRACT */
static uint8_t en_buff[] = {0x01, 0x00};
static twai_frame_t en_msg = {
    .header.id = 0x401bf, .header.ide = true,
    .buffer = en_buff, .buffer_len = sizeof(en_buff),
};

static TickType_t ticks_ms(unsigned ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks ? ticks : 1;
}

static TalonSRX talonSRXInit(uint8_t n_id, uint8_t c_id, bool inv)
{
    return (TalonSRX){.id=n_id, .inverted=inv, .channel=c_id, .current=0};
}

static void prepare_frames(void)
{
    const double values[] = {0, EXTEND_SIGN * TEST_OUTPUT,
                               -EXTEND_SIGN * TEST_OUTPUT};
    for (int i = 0; i < 3; ++i) {
        /* Preserve supplied encoding; unsigned shifts avoid signed-shift issues. */
        uint32_t word = (uint32_t)(int32_t)(values[i] * 1023.0);
        payload[i][0] = (word >> 16) & 255;
        payload[i][1] = (word >> 8) & 255;
        payload[i][2] = word & 255;
        payload[i][6] = 0x0b;
        payload[i][7] = actuator.inverted ? 0x40 : 0;
        frames[i] = (twai_frame_t){
            .header.id = CAN_ID_SET_SRX | actuator.id,
            .header.ide = true,
            .buffer = payload[i], .buffer_len = 8,
        };
    }
}

static bool can_rx(twai_node_handle_t node,
                   const twai_rx_done_event_data_t *event, void *context)
{
    (void)event; (void)context;
    uint8_t data[8];
    twai_frame_t frame = {.buffer=data, .buffer_len=sizeof(data)};
    (void)twai_node_receive_from_isr(node, &frame);
    return false; /* Status frames are drained, not interpreted. */
}

static bool can_tx(twai_node_handle_t node,
                   const twai_tx_done_event_data_t *event, void *context)
{
    (void)node; (void)context;
    if (!event->is_tx_success) {
        portENTER_CRITICAL_ISR(&lock);
        tx_failed = true;
        portEXIT_CRITICAL_ISR(&lock);
    }
    return false;
}

static void canSetup(bool (*rx)(twai_node_handle_t,
                     const twai_rx_done_event_data_t *, void *), void *data)
{
    twai_onchip_node_config_t config = {
        .io_cfg.tx=TX_GPIO_NUM, .io_cfg.rx=RX_GPIO_NUM,
        .bit_timing.bitrate=1000000, .tx_queue_depth=2,
        .fail_retry_cnt=0,
    };
    twai_event_callbacks_t callbacks = {.on_rx_done=rx, .on_tx_done=can_tx};
    ESP_ERROR_CHECK(twai_new_node_onchip(&config, &g_node_hdl));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(g_node_hdl, &callbacks, data));
    ESP_ERROR_CHECK(twai_node_enable(g_node_hdl));
}

/* Called only by the motion task (or app_main before that task starts). */
static esp_err_t send_frame(const twai_frame_t *frame)
{
    esp_err_t result = twai_node_transmit(g_node_hdl, frame, CAN_WAIT_MS);
    if (result == ESP_OK)
        result = twai_node_transmit_wait_all_done(g_node_hdl, CAN_WAIT_MS);
    portENTER_CRITICAL(&lock);
    bool failed = tx_failed;
    portEXIT_CRITICAL(&lock);
    return result != ESP_OK ? result : (failed ? ESP_FAIL : ESP_OK);
}

/* Test-only adaptation of setSRX: accepts just 0 and the two test outputs.
 * Returns errors instead of aborting/rebooting during motor operation.
 */
static esp_err_t setSRX(TalonSRX *srx, double value)
{
    if (srx != &actuator) return ESP_ERR_INVALID_ARG;
    int index;
    if (value == 0) index = 0;
    else if (value == EXTEND_SIGN * TEST_OUTPUT) index = 1;
    else if (value == -EXTEND_SIGN * TEST_OUTPUT) index = 2;
    else return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&lock);
    count_direction = index == 1 ? 1 : index == 2 ? -1 : 0;
    portEXIT_CRITICAL(&lock);
    return send_frame(&frames[index]);
}

static esp_err_t sendEn(void) { return send_frame(&en_msg); }

static void hall_isr(void *argument)
{
    (void)argument;
    portENTER_CRITICAL_ISR(&lock);
    ++edges;
    position_ticks += count_direction;
    if (count_direction == 0) ++unassigned_edges;
    portEXIT_CRITICAL_ISR(&lock);
}

/* Complete one-input adaptation; no external Hall helper is required. */
static void hallEffectInit(int pin)
{
    gpio_config_t config = {
        .pin_bit_mask=1ULL << pin, .mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE, .pull_down_en=GPIO_PULLDOWN_DISABLE,
        .intr_type=GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(pin, hall_isr, NULL));
}

static void feedback_task(void *argument)
{
    (void)argument;
    for (;;) {
        portENTER_CRITICAL(&lock);
        int64_t position = position_ticks;
        uint64_t total = edges, unknown = unassigned_edges;
        int direction = count_direction;
        bool fault = faulted;
        portEXIT_CRITICAL(&lock);
        printf("estimated_ticks=%" PRId64 " edges=%" PRIu64
               " unassigned_edges=%" PRIu64 " assumed_dir=%d fault=%d\n",
               position, total, unknown, direction, fault);
        vTaskDelay(ticks_ms(100));
    }
}

static bool phase(double output, unsigned duration_ms, bool enable)
{
    int64_t end = esp_timer_get_time() + (int64_t)duration_ms * 1000;
    do {
        if (setSRX(&actuator, output) != ESP_OK) return false;
        /* No independent enable task: a stalled motion loop cannot feed enable. */
        if (enable && sendEn() != ESP_OK) return false;
        vTaskDelay(ticks_ms(COMMAND_PERIOD_MS));
    } while (esp_timer_get_time() < end);
    return setSRX(&actuator, 0) == ESP_OK; /* Explicit stop after every phase. */
}

static void latch_fault(void)
{
    /* Best effort, not a guarantee of physical stop. Do not renew enable. */
    (void)setSRX(&actuator, 0);
    (void)twai_node_disable(g_node_hdl);
    portENTER_CRITICAL(&lock);
    faulted = true;
    portEXIT_CRITICAL(&lock);
    /* Never re-enable: queued frames might otherwise be transmitted later. */
}

static void motion_task(void *argument)
{
    (void)argument;
    if (!phase(0, STOP_GAP_MS, false)) goto fault;
    if (RUN_MOTION_TEST) {
        if (!phase(EXTEND_SIGN * TEST_OUTPUT, MOTION_MS, true)) goto fault;
        if (!phase(0, STOP_GAP_MS, false)) goto fault;
        if (!phase(-EXTEND_SIGN * TEST_OUTPUT, MOTION_MS, true)) goto fault;
    }
    for (;;) {
        if (!phase(0, 1000, false)) goto fault;
    }
fault:
    latch_fault();
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* SDK config routes printf to UART0, 115200 baud, TX43/RX44 on ESP32-S3.
     * Do not also run the original UART_setup(), which uses the same pins.
     */
    actuator = talonSRXInit(ACTUATOR_ID, ACTUATOR_CHANNEL_ID, ACTUATOR_INVERTED);
    prepare_frames();
    hallEffectInit(HALL_PIN);
    canSetup(can_rx, NULL);
    if (setSRX(&actuator, 0) != ESP_OK) {
        latch_fault();
        printf("Initial stop transmission failed; test cancelled.\n");
        return;
    }
    if (xTaskCreate(feedback_task, "feedback", 4096, NULL, 3, NULL) != pdPASS) {
        latch_fault();
        return;
    }
    if (xTaskCreate(motion_task, "motion", 4096, NULL, 5, NULL) != pdPASS)
        latch_fault();
}
