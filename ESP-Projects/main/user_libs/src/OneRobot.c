#include "OneRobot.h"

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX frontBucketDrum;
TalonFX backBucketDrum;

TalonSRX frontActuatorSRX;
TalonSRX backActuatorSRX;

Actuator frontActuator;
Pot frontActuatorPot;
PIDController frontActuatorPID;

Actuator backActuator;
Pot backActuatorPot;
PIDController backActuatorPID;

TalonFX *fxMotors[] = {&frontLeft, &backLeft, &frontRight, &backRight, &frontBucketDrum, &backBucketDrum};
TalonSRX *srxMotors[] = {&frontActuatorSRX, &backActuatorSRX};

static i2c_master_bus_handle_t aux_bus_handle;
static const i2c_sensor_config_t INA219_PROFILE = {
    .name = "INA219",
    .address = INA219_SENSOR_ADDR,
    .registers = (uint8_t[]){INA219_REG_BUSVOLTAGE},
    .register_count = 1};
static i2c_sensor_t aux_voltage_sensor = {.config = &INA219_PROFILE};

// Initialize Talon "objects"
void initializeTalons()
{
    frontLeft = talonFXInit(FRONT_LEFT_WHEEL_ID, FRONT_LEFT_WHEEL_CHANNEL_ID);
    backLeft = talonFXInit(BACK_LEFT_WHEEL_ID, BACK_LEFT_WHEEL_CHANNEL_ID);
    frontRight = talonFXInit(FRONT_RIGHT_WHEEL_ID, FRONT_RIGHT_WHEEL_CHANNEL_ID);
    backRight = talonFXInit(BACK_RIGHT_WHEEL_ID, BACK_RIGHT_WHEEL_CHANNEL_ID);

    backBucketDrum = talonFXInit(BACK_BUCKET_DRUM_ID, BACK_BUCKET_DRUM_CHANNEL_ID);
    frontBucketDrum = talonFXInit(FRONT_BUCKET_DRUM_ID, FRONT_BUCKET_DRUM_CHANNEL_ID);

    frontActuatorSRX = talonSRXInit(FRONT_ACTUATOR_ID, FRONT_ACTUATOR_CHANNEL_ID, false);
    backActuatorSRX = talonSRXInit(BACK_ACTUATOR_ID, BACK_ACTUATOR_CHANNEL_ID, true);

    // potSetup((adc_channel_t[]){ADC_PIN_FRONT, ADC_PIN_BACK}, 2);
    // frontActuatorPot = potInit(90, 1260, ADC_PIN_FRONT);
    // backActuatorPot = potInit(90, 1260, ADC_PIN_BACK);

    // position
    // frontActuatorPID = initPID(2, 0.15, 0.001);
    // backActuatorPID = initPID(2, 0.15, 0.001);

    // velocity
    // frontActuatorPID = initPID(0.9, 0.5, 0);
    // backActuatorPID = initPID(0.9, 0.5, 0);

    // canSetupTalons();

    hallEffectInit(HALL_PIN_FRONT, HALL_PIN_BACK);

    frontActuator = initActuator(&frontActuatorSRX, &frontActuatorPot, &frontActuatorPID, &frontDirection);
    backActuator = initActuator(&backActuatorSRX, &backActuatorPot, &backActuatorPID, &backDirection);
}

void canSetupTalons()
{
    canSetupTalonFX(&fxMotors[0], 6);
}

void directControl(ControlPacket_OneRobot pkt)
{
    int8_t leftSpeed = pkt.front_left_wheel;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1);

    int8_t rightSpeed = pkt.front_right_wheel;
    setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)) * -1);
    setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)));

    setTargetFX(&frontBucketDrum, ((int8_t)(pkt.front_bucket_drum - 127)));
    setTargetFX(&backBucketDrum, ((int8_t)(pkt.back_bucket_drum - 127)));

    float actuatorOutput = (float)(pkt.back_actuator - 127) / 128;
    if (pkt.back_actuator > 127)
    {
        //actuatorOutput = 1;
        *backActuator.direction = 1;
    }
    else if (pkt.back_actuator < 127)
    {
        //actuatorOutput = -1;
        *backActuator.direction = -1;
    }
    setSRX(backActuator.controller, 1 * actuatorOutput);

    actuatorOutput = (float)(pkt.front_actuator - 127) / 128;
    if (pkt.front_actuator > 127)
    {
        //actuatorOutput = 1;
        *frontActuator.direction = 1;
    }
    else if (pkt.front_actuator < 127)
    {
        //actuatorOutput = -1;
        *frontActuator.direction = -1;
    }
    setSRX(frontActuator.controller, -1 * actuatorOutput);

    // moveSyncActuatorsToPosition(&leftActuator, &rightActuator, actuatorPosition);
    // moveSyncActuatorsToVelocity(&leftActuator, &rightActuator, -1);
}

void initAuxVoltageSensor(void)
{
    I2C_Create_Bus(&aux_bus_handle);
    I2C_Add_Sensor(&aux_bus_handle, &aux_voltage_sensor);
}

// readapt the functions responsible for current updating and return
float updateAuxVoltage(void)
{
    uint8_t data[2];
    I2C_Burst_Read_Register(&aux_voltage_sensor, 0, data, 2);

    return (((uint16_t)(data[0] << 5) | (uint16_t)(data[1] >> 3)) / 250.0f);
}

TempPacket_OneRobot getTemperatureOneRobot()
{
    TempPacket_OneRobot packet = Init_Temp_Packet();
    packet.front_left_wheel_temp = getTemperatureTalonFX(&frontLeft);
    packet.back_left_wheel_temp = getTemperatureTalonFX(&backLeft);
    packet.front_right_wheel_temp = getTemperatureTalonFX(&frontRight);
    packet.back_right_wheel_temp = getTemperatureTalonFX(&backRight);
    packet.front_drum_temp = getTemperatureTalonFX(&frontBucketDrum);
    packet.back_drum_temp = getTemperatureTalonFX(&backBucketDrum);
    return packet;
}

CurrVoltPacket_OneRobot getCurrentVoltageOneRobot(PDH *pdh)
{
    CurrVoltPacket_OneRobot packet = Init_CurrVolt_Packet();
    packet.front_left_wheel = getChannelCurrentPDH(pdh, fxMotors[0]->channel);
    packet.back_left_wheel = getChannelCurrentPDH(pdh, fxMotors[1]->channel);
    packet.front_right_wheel = getChannelCurrentPDH(pdh, fxMotors[2]->channel);
    packet.back_right_wheel = getChannelCurrentPDH(pdh, fxMotors[3]->channel);
    packet.front_drum = getChannelCurrentPDH(pdh, fxMotors[4]->channel);
    packet.back_drum = getChannelCurrentPDH(pdh, fxMotors[5]->channel);

    packet.front_actuator = getChannelCurrentPDH(pdh, srxMotors[0]->channel);
    packet.back_actuator = getChannelCurrentPDH(pdh, srxMotors[1]->channel);

    packet.main_battery = (float)(getInputVoltagePDH(pdh));

    // FIXME: Get auxiliary battery voltage working
    packet.aux_battery = updateAuxVoltage();
    printf("Aux: %f V\n", packet.aux_battery);

    return packet;
}

/**
 * @internal
 * @brief Setups CAN for the PDH.
 *
 * @param handle    TWAI node handle for the on-chip TWAI peripheral.
 * @param edata     Pointer to TWAI event data struct containing received frame data.
 * @param user_ctx  Pointer to user context, which in this case is a RobotRegistry struct containing pointers to the PDH and TalonFX motors for callback access.
 * @return true if a higher priority task was woken by this callback and a context switch is required, false otherwise
 */
static bool robot_twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff)};

    if (ESP_OK != twai_node_receive_from_isr(handle, &rx_frame))
    {
        return false;
    }

    RobotRegistry *reg = (RobotRegistry *)user_ctx;
    for (size_t i = 0; i < reg->count; i++)
    {
        receiveCANTalonFX(reg->motors[i], &rx_frame, (uint64_t *)recv_buff);
    }
    receiveCANPDH(reg->pdh, &rx_frame, (uint64_t *)&recv_buff);
    return false;
}

void canSetupRobot(PDH *pdh, TalonFX **motors, size_t count)
{
    static bool canInitialized = false;
    if (!canInitialized)
    {
        static RobotRegistry registry;
        registry.pdh = pdh;
        registry.motors = motors;
        registry.count = count;
        twai_onchip_node_config_t node_config = {
            .io_cfg.tx = TX_GPIO_NUM,            // TWAI TX GPIO pin
            .io_cfg.rx = RX_GPIO_NUM,            // TWAI RX GPIO pin
            .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
            .io_cfg.quanta_clk_out = -1,         // FIX: Disable clock out (prevents GPIO 0 conflict)
            .io_cfg.bus_off_indicator = -1,      // FIX: Disable bus-off indicator
            .bit_timing.sp_permill = 800,
            .tx_queue_depth = 32, // Transmit queue depth set to 32
        };
        twai_event_callbacks_t can_cbs = {
            .on_rx_done = robot_twai_rx_cb,
        };
        ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &g_node_hdl));
        ESP_ERROR_CHECK(twai_node_register_event_callbacks(g_node_hdl, &can_cbs, &registry));
        ESP_ERROR_CHECK(twai_node_enable(g_node_hdl));
        canInitialized = true;
    }
}
