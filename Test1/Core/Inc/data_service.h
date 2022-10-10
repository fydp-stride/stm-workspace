#include <math.h>
#include "sensor_service.h"

#define GRAV_ACCEL 9.81

typedef struct {
  float force;
  angle_vec angle;
  uint32_t elapsed_time;
} force_point;

typedef struct {
  float force;
  int32_t index;
} peak_force_point;

extern float user_mass;

float compute_force(accel_vec* accel);

