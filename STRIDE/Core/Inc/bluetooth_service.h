#ifndef BLUETOOTH_SERVICE_H
#define BLUETOOTH_SERVICE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "retarget.h"
#include "sensor_service.h"

#define BT_IMPULSE_CMD 0x01
#define BT_MAX_FORCE_CMD 0x02
#define BT_ANGLE_CMD 0x03
#define BT_BATT_CMD 0x04
#define BT_WEIGHT_CMD 0x05
#define BT_RESPONSE_CMD 0x06
#define BT_CALIBRATE_CMD 0x07

#define BT_SEND_TIMEOUT 100
#define BT_RECV_TIMEOUT 10000
#define SYNC_BYTE 0xFF
#define BT_BUF_SIZE 300

#define BT_RECV_RESET 0
#define BT_RECV_SYNC 1
#define BT_RECV_CMD 2
#define BT_RECV_LEN 3
#define BT_RECV_DATA 4

#define BT_CS_Pin GPIO_PIN_6
#define BT_CS_GPIO_Port GPIOA

typedef struct
{
  uint8_t sync;
  uint8_t cmd;
  uint8_t len;
} bt_header;

typedef enum {BT_OK, BT_ERR, BT_BUSY} bt_status;

void bt_init(UART_HandleTypeDef* dev_huart, void (*recv_callback)(bt_header*, uint8_t*));
uint8_t bt_send(uint8_t command, void* data, uint8_t len);
uint8_t bt_send_str_value(uint8_t command, char* value, int len);
uint8_t bt_send_str_uint16(uint8_t command, uint16_t value);
uint8_t bt_send_float(uint8_t command, float value);
uint8_t bt_send_float_array(uint8_t command, float* array, uint8_t len);
uint8_t bt_send_str_float(uint8_t command, float value);
uint8_t bt_send_str_float_array(uint8_t command, float* array, uint8_t len);
uint8_t bt_recv(bt_header* header, void* data);
void bt_recv_callback();
bt_header* bt_get_recv_header();
uint8_t* bt_get_recv_data();

#endif
