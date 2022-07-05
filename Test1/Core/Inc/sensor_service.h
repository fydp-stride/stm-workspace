#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include "ADXL.h"

typedef struct {
  float x;
  float y;
  float z;
} triple_axis_accel;

typedef struct {
  float roll;
  float yaw;
  float pitch;
} triple_axis_angle;

typedef enum {ACCEL_OK,ACCEL_ERR} accelStatus;

accelStatus accel_init();
void accel_sample(triple_axis_accel* accel_data, triple_axis_angle* angle_data);

#endif
