#include <math.h>
#include "sensor_service.h"


sensorStatus accel_init() {
	ADXL_InitTypeDef adxl_init;
	adxl_init.SPIMode = SPIMODE_4WIRE;
	adxl_init.LPMode = LPMODE_NORMAL;
	adxl_init.Rate = BWRATE_3200;
	adxl_init.IntMode = INT_ACTIVEHIGH;
	adxl_init.Resolution = RESOLUTION_10BIT;
	adxl_init.Justify = JUSTIFY_SIGNED;
	adxl_init.AutoSleep = AUTOSLEEPOFF;
	adxl_init.Range = RANGE_16G;
	adxl_init.LinkMode = LINKMODEOFF;

	if (ADXL_Init(&adxl_init) != ADXL_OK) {
	  return SENSOR_ERR;
	}

	return SENSOR_OK;
}

void accel_sample(accel_vec* accel_data, angle_vec* angle_data) {
	float res[3];
	ADXL_getAccel(res, OUTPUT_FLOAT);

	accel_data->x = res[0];
	accel_data->y = res[1];
	accel_data->z = res[2];

	angle_data->yaw = atan2(accel_data->x , accel_data->y) * 57.3;
	angle_data->roll = atan2(accel_data->x, accel_data->z) * 57.3;
	angle_data->pitch = atan2((-accel_data->y) , sqrt(accel_data->x * accel_data->x + accel_data->z * accel_data->z)) * 57.3;
}


sensorStatus imu_init(I2C_HandleTypeDef *hi2c_device) {
	bno055_setup();
	bno055_setOperationModeNDOF();

	if (bno055_getSystemStatus() != BNO055_SYSTEM_STATUS_FUSION_ALGO_RUNNING) {
		return SENSOR_ERR;
	}

	return SENSOR_OK;
}

void imu_sample(accel_vec* accel_data, angle_vec* angle_data) {
	bno055_vector_t v_accel = bno055_getVectorAccelerometer();
	accel_data->x = v_accel.x;
	accel_data->y = v_accel.y;
	accel_data->z = v_accel.z;

	bno055_vector_t v_angle = bno055_getVectorEuler();
	angle_data->yaw = v_angle.x;
	angle_data->roll = v_angle.y;
	angle_data->pitch = v_angle.z;
}
