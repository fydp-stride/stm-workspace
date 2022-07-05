#include "sensor_service.h"


int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, sizeof(uint8_t), 1000);
	HAL_SPI_Transmit(handle, (uint8_t*)bufp, len, 1000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	return 0;
}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	reg |= 0x80;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, sizeof(uint8_t), 1000);
	HAL_SPI_Receive(handle, bufp, len, 1000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	return 0;
}

uint8_t ism330dhcx_init(stmdev_ctx_t* dev_ctx) {
	dev_ctx->write_reg = platform_write;
	dev_ctx->read_reg = platform_read;

	HAL_Delay(10);

	uint8_t deviceId;
	ism330dhcx_device_id_get(dev_ctx, &deviceId);

	// Check accelerometer device ID
//	if (deviceId != ISM330DHCX_ID) {
//		printf("Error: invalid device ID %u\r\n", deviceId);
//		return -1;
//	}

	// Reset accelerometer
	if (ism330dhcx_reset_set(dev_ctx, PROPERTY_ENABLE) != 0) {
		printf("Error: reset failed\r\n");
		return -1;
	}

	// Auto increment register address during a multi-byte access
	if (ism330dhcx_auto_increment_set(dev_ctx, PROPERTY_ENABLE) != 0) {
		printf("Error: set auto-increment failed\r\n");
		return -1;
	}

	// Ensure register not updated until block is read
	if (ism330dhcx_block_data_update_set(dev_ctx, PROPERTY_ENABLE) != 0) {
		printf("Error: set block data update failed\r\n");
		return -1;
	}

	// Enable FIFO for temporary storage
	if (ism330dhcx_fifo_mode_set(dev_ctx, ISM330DHCX_FIFO_MODE) != 0) {
		printf("Error: set FIFO failed\r\n");
		return -1;
	}

	// Set max acceleration measurement to 16g
	if (ism330dhcx_xl_full_scale_set(dev_ctx, ISM330DHCX_16g) != 0) {
		printf("Error: set scale failed\r\n");
		return -1;
	}

	// Set acceleration output rate to 6667Hz
	if (ism330dhcx_xl_data_rate_set(dev_ctx, ISM330DHCX_XL_ODR_6667Hz) != 0) {
		printf("Error: set output data rate failed\r\n");
		return -1;
	}

	return 0;
}

uint8_t ism330dhcx_sample_accel(stmdev_ctx_t* dev_ctx, triple_axis* accel_data) {
	int16_t raw[3];
	if (ism330dhcx_acceleration_raw_get(dev_ctx, raw) != 0) {
		return -1;
	}

	accel_data->x = ism330dhcx_from_fs16g_to_mg(raw[0]);
	accel_data->y = ism330dhcx_from_fs16g_to_mg(raw[1]);
	accel_data->z = ism330dhcx_from_fs16g_to_mg(raw[2]);
	return 0;
}

float compute_yaw(triple_axis* accel) {
	return atan2(accel->x , accel->y) * 57.3;
}

float compute_roll(triple_axis* accel) {
	return atan2(accel->x, accel->z) * 57.3;
}

float compute_pitch(triple_axis* accel) {
	return atan2((- accel->y) , sqrt(accel->x * accel->x + accel->z * accel->z)) * 57.3;
}

