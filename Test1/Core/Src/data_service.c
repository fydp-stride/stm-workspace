#include "data_service.h"

float compute_force(accel_vec* accel) {
	float ax = accel->x;
	float ay = accel->y;
	float az = accel->z;
	return user_mass * sqrt(ax * ax + ay * ay + az * az);
}
