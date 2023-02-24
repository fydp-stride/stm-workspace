#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include "ADXL.h"
#include "bno055_stm32.h"

typedef struct {
  float x;
  float y;
  float z;
} accel_vec;

typedef struct {
  float roll;
  float yaw;
  float pitch;
} angle_vec;

typedef enum {SENSOR_OK,SENSOR_ERR} sensorStatus;

sensorStatus imu_init(I2C_HandleTypeDef *hi2c_device);
void imu_sample(accel_vec* accel_data, angle_vec* angle_data);
void imu_calibrate();

#endif
