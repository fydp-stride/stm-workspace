#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "fatfs.h"
#include "fatfs_sd.h"
#include "retarget.h"
#include "stdio.h"

typedef enum {SD_OK, SD_ERR} sd_status;

sd_status sd_logger_init();

#endif
