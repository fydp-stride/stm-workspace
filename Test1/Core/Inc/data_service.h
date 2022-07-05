#include <math.h>
#include "sensor_service.h"

typedef struct {
  float force;
  triple_axis_angle angle;
  uint32_t timestamp;
} data_time_point;


extern float user_mass;

float compute_force(triple_axis_accel* accel);
