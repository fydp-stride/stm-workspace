#include "sensor_service.h"

static angle_vec angle_offset;
static angle_vec last_angle;

sensorStatus imu_init(I2C_HandleTypeDef *hi2c_device) {
	angle_offset.roll = 0;
	angle_offset.yaw = 0;
	angle_offset.pitch = 0;

	bno055_setup();
	if (bno055_set_accel_range(BNO055_ACCEL_RANGE_16G) != BNO055_RETURN_OK) {
		return SENSOR_ERR;
	}
	bno055_setOperationMode(BNO055_OPERATION_MODE_AMG);

	enum bno055_system_status_t system_status = bno055_getSystemStatus();
	if (system_status != BNO055_SYSTEM_STATUS_FUSION_ALOG_NOT_RUNNING) {
		return SENSOR_ERR;
	}

	HAL_Delay(200);
	bno055_vector_t v_accel = bno055_getVectorAccelerometer();
	bno055_vector_t v_mag = bno055_getVectorMagnetometer();

	float madgwick_beta = 0.1;
	madgwick_init(madgwick_beta, v_accel.x, v_accel.y, v_accel.z, v_mag.x, v_mag.y);

	return SENSOR_OK;
}

void imu_sample(accel_vec* accel_data, angle_vec* angle_data, float elapsed_time_us) {
	bno055_vector_t v_accel = bno055_getVectorAccelerometer();
	accel_data->x = (float)v_accel.x;
	accel_data->y = (float)v_accel.y;
	accel_data->z = (float)v_accel.z;

	bno055_vector_t v_gyro = bno055_getVectorGyroscope();
	bno055_vector_t v_mag = bno055_getVectorMagnetometer();

	madgwick_update_9dof(
		v_gyro.x, v_gyro.y, v_gyro.z, 
		v_accel.x, v_accel.y, v_accel.z, 
		v_mag.x, v_mag.y, v_mag.z,
		elapsed_time_us / 1e6
	);

	// bno055_vector_t v_angle = bno055_getVectorEuler();
	
	madgwick_angle_data_t v_angle;
	madgwick_get_angles(&v_angle);

	angle_data->roll = (float)v_angle.x - angle_offset.roll;
	angle_data->pitch = (float)v_angle.y - angle_offset.pitch;
	angle_data->yaw = (float)v_angle.z - angle_offset.yaw;
	
	last_angle.roll = v_angle.x;
	last_angle.pitch = v_angle.y;
	last_angle.yaw = v_angle.z;
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
