#include "LC709203F.h"

static void writeRegister(uint8_t address, uint16_t value)
{
	HAL_I2C_Mem_Write(&hi2c3, BATT_I2C_DEV_ADDR, address, LC709203F_ADDR_SIZE, (void*)(&value), LC709203F_REG_SIZE, BATT_I2C_TIMEOUT);
}

static uint16_t readRegister(uint8_t address)
{
	static uint16_t value = 0;
	HAL_I2C_Mem_Read(&hi2c3, BATT_I2C_DEV_ADDR, address, LC709203F_ADDR_SIZE, (void*)(&value), LC709203F_REG_SIZE, BATT_I2C_TIMEOUT);
	return value;
}

LC709203F_status LC709203F_init() {
	LC709203F_set_power_mode(LC709203F_POWER_OPERATE);
	LC709203F_set_pack_size(LC709203F_APA_500MAH);
	LC709203F_set_profile(0x1); // 4.2V profile
	LC709203F_send_temp_mode(LC709203F_TEMPERATURE_THERMISTOR);

	if (LC709203F_get_ic_version() != LC709203F_IC_VERSION) {
		return BATT_ERR;
	} else {
		return BATT_OK;
	}
}

uint16_t LC709203F_get_ic_version() {
	return readRegister(LC709203F_CMD_ICVERSION);
}

void LC709203F_set_power_mode(lc709203_powermode_t val) {
	writeRegister(LC709203F_CMD_POWERMODE, (uint16_t)val);
}

void LC709203F_set_pack_size(lc709203_adjustment_t val) {
	writeRegister(LC709203F_CMD_APA, (uint16_t)val);
}

void LC709203F_set_profile(uint16_t val) {
	writeRegister(LC709203F_CMD_BATTPROF, val);
}

void LC709203F_send_temp_mode(lc709203_tempmode_t val) {
	writeRegister(LC709203F_CMD_STATUSBIT, (uint16_t)val);
}

float LC709203F_get_percent() {
	return readRegister(LC709203F_CMD_CELLITE) / 10.0;
}
