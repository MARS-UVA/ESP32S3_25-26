#include "control_startup.h"
#include "OneRobot.h"

TalonFX frontLeft;
TalonFX backLeft;
TalonFX frontRight;
TalonFX backRight;
TalonFX frontBucketDrum;
TalonFX backBucketDrum;

TalonSRX frontActuator;
TalonSRX backActuator;

TalonFX *fxMotors[] = {&frontLeft, &backLeft, &frontRight, &backRight, &frontBucketDrum, &backBucketDrum};
TalonSRX *srxMotors[] = {&frontActuator, &backActuator};

static i2c_master_bus_handle_t aux_bus_handle;
static const i2c_sensor_config_t INA219_PROFILE = {
    .name = "INA219",
    .address = INA219_SENSOR_ADDR,
    .registers = (uint8_t[]) {INA219_REG_BUSVOLTAGE},
    .register_count = 1
};
static i2c_sensor_t aux_voltage_sensor = { .config = &INA219_PROFILE };
static float aux_battery_voltage = 0.0f;

// Initialize Talon "objects"
void initializeTalons()
{
    frontLeft = talonFXInit(FRONT_LEFT_WHEEL_ID, FRONT_LEFT_WHEEL_CHANNEL_ID);
    backLeft = talonFXInit(BACK_LEFT_WHEEL_ID, BACK_LEFT_WHEEL_CHANNEL_ID);
    frontRight = talonFXInit(FRONT_RIGHT_WHEEL_ID, FRONT_RIGHT_WHEEL_CHANNEL_ID);
    backRight = talonFXInit(BACK_RIGHT_WHEEL_ID, BACK_RIGHT_WHEEL_CHANNEL_ID);

    backBucketDrum = talonFXInit(BACK_BUCKET_DRUM_ID, BACK_BUCKET_DRUM_CHANNEL_ID);
    frontBucketDrum = talonFXInit(FRONT_BUCKET_DRUM_ID, FRONT_BUCKET_DRUM_CHANNEL_ID);

    frontActuator = talonSRXInit(FRONT_ACTUATOR_ID, FRONT_ACTUATOR_CHANNEL_ID, false);
    backActuator = talonSRXInit(BACK_ACTUATOR_ID, BACK_ACTUATOR_CHANNEL_ID, true);
}

void canSetupTalons()
{
    canSetupTalonFX(&fxMotors[0], 6);
    // canSetupTalonFX(&backLeft);
    // canSetupTalonFX(&frontRight);
    // canSetupTalonFX(&backRight);

    // canSetupTalonFX(&frontBucketDrum);
    // canSetupTalonFX(&backBucketDrum);
}

void directControl(ControlPacket_OneRobot pkt)
{
    sendEn();
    int8_t leftSpeed = pkt.front_left_wheel;
    setTargetFX(&frontLeft, ((int8_t)(leftSpeed - 127)) * -1);
    setTargetFX(&backLeft, ((int8_t)(leftSpeed - 127)));

    int8_t rightSpeed = pkt.front_right_wheel;
    setTargetFX(&frontRight, ((int8_t)(rightSpeed - 127)));
    setTargetFX(&backRight, ((int8_t)(rightSpeed - 127)) * -1);

    //setTargetFX(&frontBucketDrum, ((int8_t)(pkt.front_bucket_drum - 127)) * -1);
    setTargetFX(&backBucketDrum, ((int8_t)(pkt.back_bucket_drum - 127)));

    float actuatorOutput = 0;
    if (pkt.back_actuator > 127)
    {
        actuatorOutput = 1;
    }
    else if (pkt.back_actuator < 127)
    {
        actuatorOutput = -1;
    }
    setSRX(&backActuator, -1 * actuatorOutput);

    actuatorOutput = 0;
    if (pkt.front_actuator > 127)
    {
        actuatorOutput = 1;
    }
    else if (pkt.front_actuator < 127)
    {
        actuatorOutput = -1;
    }
    setSRX(&frontActuator, -1 * actuatorOutput);
}

/**void initAuxVoltageSensor(void)
{
    I2C_Create_Bus(&aux_bus_handle);
    I2C_Add_Sensor(&aux_bus_handle, &aux_voltage_sensor);
    aux_battery_voltage = 0.0f;
}

//readapt the functions responsible for current updating and return
void updateAuxVoltage(void)
{
    uint8_t data[2];

    I2C_Burst_Read_Register(&aux_voltage_sensor, 0, data, 2);
    aux_battery_voltage = ((((uint16_t)(data[0] << 8) | (uint16_t)(data[1])) >> 3) / 250.0f);
}

float getAuxVoltage(void)
{
    return aux_battery_voltage;
}**/

// TODO: Remove this function after testing
void test_run_motor(void)
{
    sendEn();
    setTargetFX(&frontLeft, 200);
    setTargetFX(&backLeft, 200);
    setTargetFX(&frontRight, 200);
    setTargetFX(&backRight, 200);
}


// TODO: Make it so this logic does not only work for one robot.
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
