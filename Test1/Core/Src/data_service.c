#include "data_service.h"

float compute_force(triple_axis_accel* accel) {
	float ax = accel->x * GRAV_ACCEL;
	float ay = accel->y * GRAV_ACCEL;
	float az = accel->z * GRAV_ACCEL;
	return user_mass * sqrt(ax * ax + ay * ay + az * az);
}
