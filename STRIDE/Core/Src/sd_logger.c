#include "sd_logger.h"


#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })


sd_status sd_logger_init(FIL* fil) {
	FATFS fs;
	DIR dir;
	FILINFO filinfo;

	if(f_mount(&fs, "", 0) != FR_OK) {
		return SD_ERR;
	}
	if (f_opendir(&dir, "") != FR_OK) {
		return SD_ERR;
	}

	int max_file_seq = 0;
	do {
	    f_readdir(&dir, &filinfo);
	    if(filinfo.fname[0] != 0) {
	    	max_file_seq = max(atoi(filinfo.fname), max_file_seq);
	    }
	} while(filinfo.fname[0] != 0);

	char filename_buf[100];
	sprintf(filename_buf, "%d", max_file_seq + 1);
	if(f_open(fil, filename_buf, FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
		return SD_ERR;
	}

	return SD_OK;
}

