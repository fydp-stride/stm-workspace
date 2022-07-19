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

void accel_sample(triple_axis_accel* accel_data, triple_axis_angle* angle_data) {
	float res[3];
	ADXL_getAccel(res, OUTPUT_FLOAT);

	accel_data->x = res[0];
	accel_data->y = res[1];
	accel_data->z = res[2];

	angle_data->yaw = atan2(accel_data->x , accel_data->y) * 57.3;
	angle_data->roll = atan2(accel_data->x, accel_data->z) * 57.3;
	angle_data->pitch = atan2((-accel_data->y) , sqrt(accel_data->x * accel_data->x + accel_data->z * accel_data->z)) * 57.3;
}

