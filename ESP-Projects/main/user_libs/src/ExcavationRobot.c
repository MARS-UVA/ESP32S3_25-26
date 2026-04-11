#include "control_startup.h"
#include "ExcavationRobot.h"
#include "talonFX.h"
#include "pdh.h"

// Excavation robot
TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX bucketLadder;
TalonFX conveyorBelt;

TalonSRX leftTrackActuator;
TalonSRX rightTrackActuator;

TalonFX *fxMotors[] = {&frontLeft, &backLeft, &frontRight, &backRight, &bucketLadder, &conveyorBelt};
TalonSRX *srxMotors[] = {&leftTrackActuator, &rightTrackActuator};

// Initialize Excavation Talons "objects"
void initializeTalons(PDH *pdh)
{
    frontLeft = talonFXInit(FRONT_LEFT_WHEEL_ID, FRONT_LEFT_WHEEL_CHANNEL_ID);
    backLeft = talonFXInit(BACK_LEFT_WHEEL_ID, BACK_LEFT_WHEEL_CHANNEL_ID);
    frontRight = talonFXInit(FRONT_RIGHT_WHEEL_ID, FRONT_RIGHT_WHEEL_CHANNEL_ID);
    backRight = talonFXInit(BACK_RIGHT_WHEEL_ID, BACK_RIGHT_WHEEL_CHANNEL_ID);

    bucketLadder = talonFXInit(BUCKET_LADDER_ID, BUCKET_LADDER_CHANNEL_ID);
    conveyorBelt = talonFXInit(CONVEYOR_BELT_ID, CONVEYOR_BELT_CHANNEL_ID);

    leftTrackActuator = talonSRXInit(LEFT_TRACK_ACTUATOR_ID, LEFT_TRACK_ACTUATOR_CHANNEL_ID, false);
    rightTrackActuator = talonSRXInit(RIGHT_TRACK_ACTUATOR_ID, RIGHT_TRACK_ACTUATOR_CHANNEL_ID, false);
}

void directControl(ControlPacket_ExcavationRobot pkt)
{
    int8_t leftSpeed = pkt.front_left_wheel;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)) * -1);

    int8_t rightSpeed = pkt.front_right_wheel;
    setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)) * 1);
    setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);

    int8_t bucketLadderSpeed = pkt.bucket_ladder;
    setTargetFX(&bucketLadder, ((int8_t)(bucketLadderSpeed - 127)));

    int8_t conveyorBeltSpeed = pkt.conveyor_belt;
    setTargetFX(&conveyorBelt, ((int8_t)(conveyorBeltSpeed - 127)));

    float actuatorOutput = (pkt.track_actuator - 127) / 127.0;
    setSRX(&leftTrackActuator, actuatorOutput);
    setSRX(&rightTrackActuator, actuatorOutput);
}

TempPacket_ExcavationRobot getTemperatureExcavationRobot()
{
    TempPacket_ExcavationRobot packet = Init_Temp_Excavation_Robot_Packet();
    packet.front_left_wheel_temp = getTemperatureTalonFX(&frontLeft);
    packet.back_left_wheel_temp = getTemperatureTalonFX(&backLeft);
    packet.front_right_wheel_temp = getTemperatureTalonFX(&frontRight);
    packet.back_right_wheel_temp = getTemperatureTalonFX(&backRight);
    return packet;
}

CurrVoltPacket_ExcavationRobot getCurrentVoltageExcavationRobot(PDH *pdh)
{
    CurrVoltPacket_ExcavationRobot packet = Init_CurrVolt_Excavation_Robot_Packet();
    packet.front_left_wheel = getChannelCurrentPDH(pdh, fxMotors[0]->channel);
    packet.back_left_wheel = getChannelCurrentPDH(pdh, fxMotors[1]->channel);
    packet.front_right_wheel = getChannelCurrentPDH(pdh, fxMotors[2]->channel);
    packet.back_right_wheel = getChannelCurrentPDH(pdh, fxMotors[3]->channel);
    packet.bucket_ladder = getChannelCurrentPDH(pdh, fxMotors[4]->channel);
    packet.conveyor_belt = getChannelCurrentPDH(pdh, fxMotors[5]->channel);

    packet.left_track_actuator = getChannelCurrentPDH(pdh, srxMotors[0]->channel);
    packet.right_track_actuator = getChannelCurrentPDH(pdh, srxMotors[1]->channel);

    packet.main_battery = (float)(getInputVoltagePDH(pdh));

    // FIXME: Get auxiliary battery voltage working
    // updateAuxVoltage();
    // packet.aux_battery = getAuxVoltage();

    return packet;
}

void test_run_motor()
{
    sendEn();
    setTargetFX(&frontLeft, 200);
    setTargetFX(&backLeft, 200);
    setTargetFX(&frontRight, 200);
    setTargetFX(&backRight, 200);
}

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
            .io_cfg.quanta_clk_out = -1,         // FIX: Disable clock out (prevents GPIO 0 conflict)
            .io_cfg.bus_off_indicator = -1,      // FIX: Disable bus-off indicator
            .bit_timing.bitrate = ROBOT_BITRATE, // 1Mbps bitrate
            .tx_queue_depth = 32,                // Transmit queue depth set to 32
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
