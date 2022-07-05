#include "ism330dhcx_reg.h"
#include "stm32l4xx_hal.h"
#include "main.h"
#include "retarget.h"
#include <stdio.h>

typedef struct
{
	float x;
	float y;
	float z;
} triple_axis;

typedef struct {
	triple_axis accel;
	uint32_t dt;
} accel_datapoint;


int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
uint8_t ism330dhcx_init(stmdev_ctx_t* dev_ctx);
uint8_t ism330dhcx_sample_accel(stmdev_ctx_t* dev_ctx, triple_axis* accel_data);
float compute_yaw(triple_axis* accel);
float compute_roll(triple_axis* accel);
float compute_pitch(triple_axis* accel);
