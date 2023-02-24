#include <math.h>
#include "sensor_service.h"

static angle_vec angle_offset;
static angle_vec last_angle;

sensorStatus imu_init(I2C_HandleTypeDef *hi2c_device) {
	angle_offset.roll = 0;
	angle_offset.yaw = 0;
	angle_offset.pitch = 0;

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
	angle_data->yaw = v_angle.x - angle_offset.yaw;
	angle_data->roll = v_angle.y - angle_offset.roll;
	angle_data->pitch = v_angle.z - angle_offset.pitch;
	last_angle.yaw = v_angle.x;
	last_angle.roll = v_angle.y;
	last_angle.pitch = v_angle.z;
}

void imu_calibrate() {
	angle_offset.yaw = last_angle.yaw;
	angle_offset.roll = last_angle.roll;
	angle_offset.pitch = last_angle.pitch;

	printf(
		"Offset: yaw=%f, roll=%f, pitch=%f\r\n", 
		angle_offset.yaw, angle_offset.roll, angle_offset.pitch
	);
}
