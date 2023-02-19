#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "retarget.h"

#define DBG_BUF_SIZE 250

#define DBG_LPTHRESH_CMD 0x01
#define DBG_CALIBRATE_CMD 0x02

#define DBG_LPTHRESH_CMD_STR "lp"
#define DBG_CALIBRATE_CMD_STR "cal"

void dbg_init(UART_HandleTypeDef* dev_huart, void (*cmd_callback)(uint8_t, char*));
void dbg_callback();

#endif