#include "data_service.h"

float compute_force(triple_axis_accel* accel) {
	float ax = accel->x / 1e3;
	float ay = accel->y / 1e3;
	float az = accel->z / 1e3;
	return user_mass * sqrt(ax * ax + ay * ay + az * az);
}
