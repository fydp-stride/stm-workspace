#include "data_service.h"

float user_mass;

float compute_force(triple_axis_accel* accel) {
	return user_mass * sqrt(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
}
