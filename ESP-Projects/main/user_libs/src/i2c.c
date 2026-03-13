#include "i2c.h"


//Internal utilities
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static i2c_master_bus_config_t _I2C_Bus_Config_ = {
    .i2c_port = I2C_MASTER_NUM,      // sets the i2c port (either 0 or 1) to use for communication
    .sda_io_num = I2C_MASTER_SDA_IO, // configuring pins for sda and scl
    .scl_io_num = I2C_MASTER_SCL_IO, // configuring pins for sda and scl
    .clk_source = I2C_CLK_SRC_DEFAULT, // you can change clock source, but generally just use default
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

static i2c_device_config_t _I2C_Get_Device_Config_(uint16_t address) {
    i2c_device_config_t dev_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7, // length of data inside a given register
		.device_address = address,
		.scl_speed_hz = I2C_MASTER_FREQ_HZ,
	};
    return dev_config;
}

static uint8_t* _I2C_Create_Write_Buffer_(uint8_t reg_addr, const uint8_t *data, size_t len) {
    uint8_t* write_buf = malloc(len+1); // Allocate a buffer to hold [register_address + data]
	if (write_buf == NULL) return NULL;
    write_buf[0] = reg_addr;
	memcpy(&write_buf[1], data, len);
    return write_buf;
}













//BUS STUFF
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
esp_err_t I2C_Create_Bus(i2c_master_bus_handle_t *bus_handle) {
	return i2c_new_master_bus(&_I2C_Bus_Config_, bus_handle);
}
esp_err_t I2C_Delete_Bus(i2c_master_bus_handle_t *bus_handle) {
	return i2c_del_master_bus(*bus_handle);
}

//Adding/Removing sensors
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
esp_err_t I2C_Add_Sensor(i2c_master_bus_handle_t* bus_handle, i2c_sensor_t* sensor) {
	i2c_device_config_t dev_config = _I2C_Get_Device_Config_(sensor->config->address);
	return i2c_master_bus_add_device(*bus_handle, &dev_config, &sensor->handle);
}
esp_err_t I2C_Remove_Sensor(i2c_sensor_t* sensor) {
	return i2c_master_bus_rm_device(sensor->handle);
}


//Indirect read/write
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
esp_err_t I2C_Read_Register(i2c_sensor_t* sensor, size_t index, uint8_t* result) {
	return _I2C_Read_Register_(&sensor->handle, sensor->config->registers[index], result, 1);
}

esp_err_t I2C_Read_Registers(i2c_sensor_t* sensor, uint8_t result[]) {
	for (int i=0; i<sensor->config->register_count; i++) {
		esp_err_t error = _I2C_Read_Register_(&sensor->handle, sensor->config->registers[i], &result[i], 1);
		if (error != ESP_OK) return error; //TODO: re-implement log error.
	}
	return ESP_OK;
}

esp_err_t I2C_Burst_Read_Register(i2c_sensor_t* sensor, size_t index, uint8_t result[], size_t len) {
	return _I2C_Read_Register_(&sensor->handle, sensor->config->registers[index], result, len);
}


//Direct read/write
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//This is used for the rare case you need to access a register that has an address larger than 8 bits.
esp_err_t _I2C_Write_Read_Register_(i2c_master_dev_handle_t* handle, const uint8_t* wdata, size_t wlen, uint8_t* rdata, size_t rlen) {
	return i2c_master_transmit_receive(*handle, wdata, wlen, rdata, rlen, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t _I2C_Read_Register_(i2c_master_dev_handle_t* handle, uint8_t reg_addr, uint8_t *data, size_t len) {
	return i2c_master_transmit_receive(*handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS); 
}

//This function allows you to send a register address followed by any number of data bytes (useful for configuration or multi-byte commands).
esp_err_t _I2C_Write_Register_(i2c_master_dev_handle_t* handle, uint8_t reg_addr, const uint8_t *data, size_t len) {
	uint8_t *write_buf = _I2C_Create_Write_Buffer_(reg_addr, data, len);
    if (write_buf == NULL) return ESP_ERR_NO_MEM;
	esp_err_t err = i2c_master_transmit(*handle, write_buf, len+1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
	free(write_buf);
	return err;
}

//This function allows you to send a basic command to the sensor. EX: BH1750 has power up and power down commands
esp_err_t _I2C_Write_Byte_(i2c_master_dev_handle_t* handle, const uint8_t *data) {
    return i2c_master_transmit(*handle, data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}