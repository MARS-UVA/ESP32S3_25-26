/* Standalone adaptation of the supplied project; do not link its libraries too.
 * Hall feedback decodes quadrature A2/B2 on GPIO11/GPIO7, both edges (x4).
 * Position is relative to startup; encoder polarity/calibration need checking.
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
#include "driver/uart.h"


#define RUN_MOTION_TEST 1      /* Set to 1 only after checking README.md. */
#define TX_GPIO_NUM GPIO_NUM_2  /* ESP32 -> TJA1051TK-3 TXD; user-confirmed wiring. */
#define RX_GPIO_NUM GPIO_NUM_1  /* ESP32 <- TJA1051TK-3 RXD; user-confirmed wiring. */
#define HALL_A_PIN GPIO_NUM_11  /* A2; user-confirmed wiring. */
#define HALL_B_PIN GPIO_NUM_7   /* B2; user-confirmed wiring. */
#define ENCODER_SIGN 1         /* Set to -1 if physical extension counts down. */
#define ACTUATOR_ID 4           /* SRX device ID confirmed by the user. */
#define ACTUATOR_CHANNEL_ID 13  /* Stored metadata; not used in setSRX frames. */
#define ACTUATOR_INVERTED false
#define EXTEND_SIGN (-1)        /* Original front command convention; VERIFY. */
#define TEST_OUTPUT 0.50
#define MOTION_MS 5000
#define STOP_GAP_MS 3000
#define COMMAND_PERIOD_MS 10
#define CAN_WAIT_MS 10
#define CAN_ID_SET_SRX 0x2040200

typedef struct {
    uint8_t id;
    bool inverted;
    uint8_t channel;
    float current;
} TalonSRX;

const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

static twai_node_handle_t g_node_hdl;
static TalonSRX actuator;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t position_ticks;
static uint64_t transitions, invalid_transitions;
static uint8_t previous_ab;
static int command_direction; /* Diagnostic only; never used to count position. */
/* Index = (previous A/B << 2) | current A/B; A is the high bit.
 * Positive sequence: 00 -> 01 -> 11 -> 10 -> 00.
 */
static const int8_t quadrature_step[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};
static bool tx_failed, faulted;
static uint32_t received_frames;
/* First failure only: the best-effort stop must not overwrite its cause. */
static struct {
    bool valid, waited, completion_failed, status_valid;
    uint32_t id, rx_frames;
    esp_err_t enqueue_result, wait_result;
    twai_node_status_t status;
} first_failure;

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
    if (twai_node_receive_from_isr(node, &frame) == ESP_OK) {
        portENTER_CRITICAL_ISR(&lock);
        ++received_frames;
        portEXIT_CRITICAL_ISR(&lock);
    }
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
    esp_err_t enqueue = twai_node_transmit(g_node_hdl, frame, CAN_WAIT_MS);
    esp_err_t wait = ESP_OK;
    if (enqueue == ESP_OK)
        wait = twai_node_transmit_wait_all_done(g_node_hdl, CAN_WAIT_MS);
    portENTER_CRITICAL(&lock);
    bool failed = tx_failed;
    uint32_t rx_frames = received_frames;
    portEXIT_CRITICAL(&lock);
    esp_err_t result = enqueue != ESP_OK ? enqueue :
                      wait != ESP_OK ? wait : failed ? ESP_FAIL : ESP_OK;
    if (result != ESP_OK && !first_failure.valid) {
        first_failure.valid = true;
        first_failure.id = frame->header.id;
        first_failure.enqueue_result = enqueue;
        first_failure.waited = enqueue == ESP_OK;
        first_failure.wait_result = wait;
        first_failure.completion_failed = failed;
        first_failure.rx_frames = rx_frames;
        first_failure.status_valid = twai_node_get_info(
            g_node_hdl, &first_failure.status, NULL) == ESP_OK;
    }
    return result;
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
    command_direction = index == 1 ? 1 : index == 2 ? -1 : 0;
    portEXIT_CRITICAL(&lock);
    return send_frame(&frames[index]);
}

static esp_err_t sendEn(void) { return send_frame(&en_msg); }

static void hall_isr(void *argument)
{
    (void)argument;
    portENTER_CRITICAL_ISR(&lock);
    uint8_t ab = (gpio_get_level(HALL_A_PIN) << 1) |
                  gpio_get_level(HALL_B_PIN);
    uint8_t changed = previous_ab ^ ab;
    if (changed == 3) {
        /* Both bits changed: missed edge/noise; reject rather than guess. */
        ++invalid_transitions;
    } else if (changed != 0) {
        position_ticks += ENCODER_SIGN * quadrature_step[(previous_ab << 2) | ab];
        ++transitions;
    }
    previous_ab = ab; /* Resynchronize even after an invalid transition. */
    portEXIT_CRITICAL_ISR(&lock);
}

/* Both channels of ONE actuator. Initialize while mechanically stationary. */
static void hallEffectInit(void)
{
    gpio_config_t config = {
        .pin_bit_mask=(1ULL << HALL_A_PIN) | (1ULL << HALL_B_PIN),
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE, .pull_down_en=GPIO_PULLDOWN_DISABLE,
        .intr_type=GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HALL_A_PIN, hall_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(HALL_B_PIN, hall_isr, NULL));
    previous_ab = (gpio_get_level(HALL_A_PIN) << 1) |
                  gpio_get_level(HALL_B_PIN);
    ESP_ERROR_CHECK(gpio_set_intr_type(HALL_A_PIN, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(HALL_B_PIN, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_intr_enable(HALL_A_PIN));
    ESP_ERROR_CHECK(gpio_intr_enable(HALL_B_PIN));
}

/*static void feedback_task(void *argument)
{
    (void)argument;
    for (;;) {
        portENTER_CRITICAL(&lock);
        int64_t position = position_ticks;
        uint64_t total = transitions, invalid = invalid_transitions;
        int direction = command_direction;
        bool fault = faulted;
        portEXIT_CRITICAL(&lock);
        printf("position_ticks=%" PRId64 " transitions=%" PRIu64
               " invalid_transitions=%" PRIu64 " command_dir=%d fault=%d\n",
               position, total, invalid, direction, fault);
        vTaskDelay(ticks_ms(100));
    }
}*/

static void feedback_task(void *argument)
{
    (void)argument;

    char message[160];

    for (;;) {
        portENTER_CRITICAL(&lock);
        int64_t position = position_ticks;
        uint64_t total = transitions, invalid = invalid_transitions;
        int direction = command_direction;
        bool fault = faulted;
        portEXIT_CRITICAL(&lock);

        int len = snprintf(
            message,
            sizeof(message),
            "position_ticks=%" PRId64 " transitions=%" PRIu64
            " invalid_transitions=%" PRIu64 " command_dir=%d fault=%d\r\n",
            position, total, invalid, direction, fault
        );

        if (len > 0) {
            uart_write_bytes(UART_NUM_1, message, len);
        }

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
    /* Print only AFTER attempting stop and disabling CAN. */
    if (first_failure.valid) {
        printf("CAN failure: id=0x%08" PRIx32 " enqueue=%s wait=%s"
               " tx_failed_latched=%d rx_frames=%" PRIu32 "\n",
               first_failure.id, esp_err_to_name(first_failure.enqueue_result),
               first_failure.waited ? esp_err_to_name(first_failure.wait_result)
                                    : "not_attempted",
               first_failure.completion_failed, first_failure.rx_frames);
        if (first_failure.status_valid)
            printf("CAN state before shutdown: state=%d TEC=%u REC=%u\n",
                   (int)first_failure.status.state,
                   (unsigned)first_failure.status.tx_error_count,
                   (unsigned)first_failure.status.rx_error_count);
    }
}

static void motion_task(void *argument)
{
    (void)argument;

    /* Start stopped. */
    if (!phase(0, STOP_GAP_MS, false)) goto fault;

    for (;;) {
        if (RUN_MOTION_TEST) {
            if (!phase(EXTEND_SIGN * TEST_OUTPUT,
                       MOTION_MS, true)) goto fault;

            if (!phase(0, STOP_GAP_MS, false)) goto fault;

            if (!phase(-EXTEND_SIGN * TEST_OUTPUT,
                       MOTION_MS, true)) goto fault;
        }

        if (!phase(0, STOP_GAP_MS, false)) goto fault;
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
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(
        UART_NUM_1, 17, 18,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE
    ));
    
    actuator = talonSRXInit(ACTUATOR_ID, ACTUATOR_CHANNEL_ID, ACTUATOR_INVERTED);
    prepare_frames();
    hallEffectInit();
    canSetup(can_rx, NULL);
    printf("CAN configuration: TX=%d RX=%d bitrate=1000000 SRX_ID=%d"
           " motion_test=%d\n", TX_GPIO_NUM, RX_GPIO_NUM,
           ACTUATOR_ID, RUN_MOTION_TEST);
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
