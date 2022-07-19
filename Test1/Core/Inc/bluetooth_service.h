#ifndef BLUETOOTH_SERVICE_H
#define BLUETOOTH_SERVICE_H

#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "retarget.h"
#include "sensor_service.h"

#define IMPULSE_CMD 0x01
#define MAX_FORCE_CMD 0x02
#define ANGLE_CMD 0x03
#define BATT_CMD 0x04
#define WEIGHT_CMD 0x05
#define RESPONSE_CMD 0x06
#define BT_SEND_TIMEOUT 100
#define BT_RECV_TIMEOUT 10000
#define SYNC_BYTE 0xFF
#define BT_BUF_SIZE 500

#define BT_CS_Pin GPIO_PIN_6
#define BT_CS_GPIO_Port GPIOA

typedef struct
{
  uint8_t cmd;
  uint8_t len;
} bt_header;

uint8_t bt_send_spi(void* handle, void* buf, uint16_t len);
uint8_t bt_recv_spi(void* handle, void* buf, uint16_t len);
uint8_t bt_send(void* handle, bt_header* header, void* data);
uint8_t bt_send_impulse(void* handle, float impulse);
uint8_t bt_recv(void* handle, bt_header* header, void* data);

#endif
