/*
 * ICM-42670-P bring-up / test for the MARS ESP32 custom devkit (ESP-IDF >= 5.2, new i2c_master driver)
 *
 * Reads: acceleration (g, m/s^2), angular rate (deg/s), die temperature (C),
 *        roll/pitch (absolute, from gravity) and yaw (RELATIVE, integrated gyro - drifts).
 *
 * NOTE: the ICM-42670-P is a 6-axis IMU (accel + gyro). It has NO magnetometer, so it cannot give a
 *       true compass heading. "Yaw" below is rotation since boot, not north.
 *
 * Register addresses/values from TDK DS-000451 rev 1.0.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"


#include "uart.h"
#include "control_startup.h"
#include "can.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "talonFX.h"
#include "OneRobot.h"
#include "tasks.h"
#include "pdh.h"

// can_rx_context_t can;
PDH pdh;

TaskHandle_t control_can_handle = NULL;
TaskHandle_t temperature_update_handle = NULL;
TaskHandle_t current_voltage_update_handle = NULL;
TaskHandle_t position_update_handle = NULL;
TaskHandle_t uart_rx_handle = NULL;
TaskHandle_t uart_tx_handle = NULL;
TaskHandle_t enable_handle = NULL;
TaskHandle_t can_enable_handle = NULL;

static const char *TAG = "icm42670";

/* ---------- Bus config (pins from the team's existing code - verify against devkit pinout) ---------- */
#define I2C_MASTER_SCL_IO      GPIO_NUM_9
#define I2C_MASTER_SDA_IO      GPIO_NUM_10
#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_FREQ_HZ     100000        /* chip supports up to 1 MHz; keep 100k while relying on weak pull-ups */
#define I2C_TIMEOUT_MS         100

/* ---------- I2C address: b110100X, X = AP_AD0 pin. AP_AD0 is unconnected on the schematic, so probe both. ---------- */
#define ICM_ADDR_AD0_LOW       0x68
#define ICM_ADDR_AD0_HIGH      0x69

/* ---------- User Bank 0 registers ---------- */
#define REG_MCLK_RDY           0x00
#define REG_SIGNAL_PATH_RESET  0x02
#define REG_TEMP_DATA1         0x09   /* burst start: TEMP(2) ACCEL XYZ(6) GYRO XYZ(6) = 14 bytes, 0x09..0x16 */
#define REG_PWR_MGMT0          0x1F
#define REG_GYRO_CONFIG0       0x20
#define REG_ACCEL_CONFIG0      0x21
#define REG_GYRO_CONFIG1       0x23
#define REG_ACCEL_CONFIG1      0x24
#define REG_INTF_CONFIG0       0x35
#define REG_WHO_AM_I           0x75
#define WHO_AM_I_EXPECTED      0x67

#define SOFT_RESET_BIT         (1 << 4)
#define SENSOR_DATA_BIG_ENDIAN (1 << 4)   /* INTF_CONFIG0 bit 4; reset value 0x30 -> big endian */

/* ---------- Chosen settings ---------- */
/* GYRO_CONFIG0: [6:5] FS_SEL, [3:0] ODR.  10 = +/-500 dps, 1001 = 100 Hz */
#define GYRO_FS_BITS           0x2
#define GYRO_SENS_LSB_PER_DPS  65.5f
/* ACCEL_CONFIG0: [6:5] FS_SEL, [3:0] ODR.  10 = +/-4 g, 1001 = 100 Hz */
#define ACCEL_FS_BITS          0x2
#define ACCEL_SENS_LSB_PER_G   8192.0f
#define ODR_100HZ              0x9
#define UI_FILT_BW_34HZ        0x5        /* GYRO/ACCEL_CONFIG1 [2:0] */

/* PWR_MGMT0: [3:2] GYRO_MODE=11 (LN), [1:0] ACCEL_MODE=11 (LN) */
#define PWR_GYRO_ACCEL_LN      0x0F

#define SAMPLE_PERIOD_MS       10         /* 100 Hz, matches ODR */
#define GRAVITY_MS2            9.80665f
#define RAD2DEG                57.2957795f
#define COMP_ALPHA             0.98f      /* complementary filter weight on gyro */

typedef struct {
    float ax_g, ay_g, az_g;      /* acceleration, g */
    float gx_dps, gy_dps, gz_dps;/* angular rate, deg/s (bias-corrected) */
    float temp_c;                /* die temperature, C (reads warmer than ambient) */
    float roll_deg, pitch_deg;   /* absolute w.r.t. gravity */
    float yaw_deg;               /* relative since boot - drifts, NOT compass heading */
} imu_data_t;

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static float s_gbias[3] = {0};

/* ---------------- low-level register access ---------------- */
static esp_err_t icm_read(uint8_t reg, uint8_t *buf, size_t len)
{
    /* write register address, repeated start, read len bytes (chip auto-increments) */
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t icm_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* read-modify-write so reserved bits keep their reset values (datasheet requirement) */
static esp_err_t icm_update_bits(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t cur;
    esp_err_t err = icm_read(reg, &cur, 1);
    if (err != ESP_OK) return err;
    return icm_write(reg, (cur & ~mask) | (val & mask));
}

/* ---------------- bus + device setup ---------------- */
static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, /* ~45k, weak - add 4.7k-10k external if bus is flaky */
    };
    return i2c_new_master_bus(&bus_cfg, &s_bus);
}

static esp_err_t icm_attach(void)
{
    const uint16_t candidates[] = {ICM_ADDR_AD0_LOW, ICM_ADDR_AD0_HIGH};
    for (int i = 0; i < 2; i++) {
        if (i2c_master_probe(s_bus, candidates[i], I2C_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGI(TAG, "Device ACKed at 0x%02X", candidates[i]);
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = candidates[i],
                .scl_speed_hz = I2C_MASTER_FREQ_HZ,
            };
            return i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
        }
    }
    ESP_LOGE(TAG, "No ACK at 0x68 or 0x69 - check power, SDA/SCL pins, pull-ups, AP_CS");
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t icm_init(void)
{
    uint8_t v;

    /* 1. soft reset (device powers up in sleep; reset gives a known state) */
    ESP_ERROR_CHECK(icm_write(REG_SIGNAL_PATH_RESET, SOFT_RESET_BIT));
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 2. identity check */
    ESP_ERROR_CHECK(icm_read(REG_WHO_AM_I, &v, 1));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expect 0x%02X)", v, WHO_AM_I_EXPECTED);
    if (v != WHO_AM_I_EXPECTED) return ESP_ERR_INVALID_RESPONSE;

    ESP_ERROR_CHECK(icm_read(REG_MCLK_RDY, &v, 1));
    ESP_LOGI(TAG, "MCLK_RDY reg = 0x%02X", v);

    /* 3. make sure data registers are big-endian (reset default, but don't assume) */
    ESP_ERROR_CHECK(icm_update_bits(REG_INTF_CONFIG0, SENSOR_DATA_BIG_ENDIAN, SENSOR_DATA_BIG_ENDIAN));

    /* 4. full-scale + ODR (bits 7 and 4 are reserved = 0) */
    ESP_ERROR_CHECK(icm_write(REG_GYRO_CONFIG0,  (GYRO_FS_BITS  << 5) | ODR_100HZ));
    ESP_ERROR_CHECK(icm_write(REG_ACCEL_CONFIG0, (ACCEL_FS_BITS << 5) | ODR_100HZ));

    /* 5. low-pass filters to 34 Hz (below Nyquist of 100 Hz ODR) */
    ESP_ERROR_CHECK(icm_update_bits(REG_GYRO_CONFIG1,  0x07, UI_FILT_BW_34HZ));
    ESP_ERROR_CHECK(icm_update_bits(REG_ACCEL_CONFIG1, 0x07, UI_FILT_BW_34HZ));

    /* 6. turn sensors on. No register writes for 200 us after; gyro needs ~30 ms to start. */
    ESP_ERROR_CHECK(icm_write(REG_PWR_MGMT0, PWR_GYRO_ACCEL_LN));
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

/* ---------------- data ---------------- */
static inline int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

/* raw read of temp/accel/gyro in one 14-byte burst so all values come from the same sample */
static esp_err_t icm_read_raw(int16_t *t, int16_t a[3], int16_t g[3])
{
    uint8_t b[14];
    esp_err_t err = icm_read(REG_TEMP_DATA1, b, sizeof(b));
    if (err != ESP_OK) return err;
    *t = be16(&b[0]);
    for (int i = 0; i < 3; i++) {
        a[i] = be16(&b[2 + 2 * i]);
        g[i] = be16(&b[8 + 2 * i]);
    }
    /* -32768 (0x8000) is the reset value = sensor off / no data yet */
    if (a[2] == INT16_MIN && g[2] == INT16_MIN) return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}

/* average gyro at rest -> bias. Keep the board still during this! */
static void icm_calibrate_gyro(int samples)
{
    int32_t sum[3] = {0};
    int16_t t, a[3], g[3];
    int n = 0;
    ESP_LOGI(TAG, "Calibrating gyro bias - keep board still...");
    for (int i = 0; i < samples; i++) {
        if (icm_read_raw(&t, a, g) == ESP_OK) {
            for (int k = 0; k < 3; k++) sum[k] += g[k];
            n++;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    for (int k = 0; k < 3 && n > 0; k++) s_gbias[k] = (sum[k] / (float)n) / GYRO_SENS_LSB_PER_DPS;
    ESP_LOGI(TAG, "Gyro bias (dps): %.3f %.3f %.3f", s_gbias[0], s_gbias[1], s_gbias[2]);
}

/* converts raw -> units and updates orientation. dt in seconds. */
static esp_err_t imu_update(imu_data_t *d, float dt)
{
    int16_t t, a[3], g[3];
    esp_err_t err = icm_read_raw(&t, a, g);
    if (err != ESP_OK) return err;

    d->temp_c = (t / 128.0f) + 25.0f;                 /* datasheet: TEMP_DATA/128 + 25 */
    d->ax_g = a[0] / ACCEL_SENS_LSB_PER_G;
    d->ay_g = a[1] / ACCEL_SENS_LSB_PER_G;
    d->az_g = a[2] / ACCEL_SENS_LSB_PER_G;
    d->gx_dps = g[0] / GYRO_SENS_LSB_PER_DPS - s_gbias[0];
    d->gy_dps = g[1] / GYRO_SENS_LSB_PER_DPS - s_gbias[1];
    d->gz_dps = g[2] / GYRO_SENS_LSB_PER_DPS - s_gbias[2];

    /* tilt from gravity (only valid when not accelerating hard) */
    float acc_roll  = atan2f(d->ay_g, d->az_g) * RAD2DEG;
    float acc_pitch = atan2f(-d->ax_g, sqrtf(d->ay_g * d->ay_g + d->az_g * d->az_g)) * RAD2DEG;

    /* complementary filter: gyro for short-term, accel pulls out long-term drift */
    d->roll_deg  = COMP_ALPHA * (d->roll_deg  + d->gx_dps * dt) + (1.0f - COMP_ALPHA) * acc_roll;
    d->pitch_deg = COMP_ALPHA * (d->pitch_deg + d->gy_dps * dt) + (1.0f - COMP_ALPHA) * acc_pitch;

    /* yaw: nothing to correct it against without a magnetometer -> pure integration, drifts */
    d->yaw_deg += d->gz_dps * dt;
    if (d->yaw_deg > 180.0f)  d->yaw_deg -= 360.0f;
    if (d->yaw_deg < -180.0f) d->yaw_deg += 360.0f;
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_bus_init());
    ESP_ERROR_CHECK(icm_attach());
    ESP_ERROR_CHECK(icm_init());
    icm_calibrate_gyro(200); /* ~2 s */

    imu_data_t d = {0};
    int64_t last_us = esp_timer_get_time();
    TickType_t wake = xTaskGetTickCount();
    uint32_t n = 0;

    TalonFX testmotor = talonFXInit(36, 19);
    TalonFX* motors[1] = {&testmotor};
    
    // PDHInit(&pdh, 62);
    // canSetupRobot(&pdh, &motors[0], 1);
    
    canSetupTalonFX(&motors[0], 1);

    vTaskDelay(pdMS_TO_TICKS(10));

    while (1) {

        sendEn();
        setFX(&testmotor, d.roll_deg/180.0f);

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_us) / 1e6f;
        last_us = now_us;

        esp_err_t err = imu_update(&d, dt);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
            continue;
        }

        if (++n % 10 == 0) { /* print at 10 Hz */
            float amag = sqrtf(d.ax_g * d.ax_g + d.ay_g * d.ay_g + d.az_g * d.az_g);
            printf("A[g] %+6.3f %+6.3f %+6.3f |A|=%5.3f (%.2f m/s^2)  "
                   "G[dps] %+7.2f %+7.2f %+7.2f  T=%5.2fC  "
                   "roll %+6.1f pitch %+6.1f yaw(rel) %+6.1f\n",
                   d.ax_g, d.ay_g, d.az_g, amag, amag * GRAVITY_MS2,
                   d.gx_dps, d.gy_dps, d.gz_dps, d.temp_c,
                   d.roll_deg, d.pitch_deg, d.yaw_deg);
        }




    }
}
