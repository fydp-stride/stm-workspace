#ifndef BLUETOOTH_SERVICE_H
#define BLUETOOTH_SERVICE_H

#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "retarget.h"

#define IMPULSE_CMD 0x01
#define RESPONSE_CMD 0x02
#define BT_SEND_TIMEOUT 100
#define BT_RECV_TIMEOUT 10000
#define SYNC_BYTE 0xFF
#define BT_BUF_SIZE 500

typedef struct
{
  uint8_t cmd;
  uint8_t len;
} bt_header;

uint8_t bt_buf[BT_BUF_SIZE];

uint8_t bt_send(void* handle, bt_header* header, void* data);
uint8_t bt_recv(void* handle, bt_header* header, void* data);

#endif
