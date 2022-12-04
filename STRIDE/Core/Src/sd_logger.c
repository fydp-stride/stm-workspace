#include "sd_logger.h"


#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })


sd_status sd_logger_init(FIL* fil) {
	FATFS fs;
	DIR dir;
	FILINFO filinfo;
	FRESULT fres;

	fres = f_mount(&fs, "", 0);
	if(fres != FR_OK) {
		printf("[Warning] [sd_logger_init] SD card logger initialization failed RET=%d (f_mount)\r\n", fres);
		return SD_ERR;
	}
	fres = f_opendir(&dir, "");
	if (fres != FR_OK) {
		printf("[Warning] [sd_logger_init] SD card logger initialization failed RET=%d (f_opendir)\r\n", fres);
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
	fres = f_open(fil, filename_buf, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if(fres != FR_OK) {
		printf("[Warning] [sd_logger_init] SD card logger initialization failed RET=%d (f_open)\r\n", fres);
		return SD_ERR;
	}

	return SD_OK;
}

sd_status sd_logger_terminate(FIL* fil) {
	FRESULT fres;

	fres = f_close(fil);
	if (fres != FR_OK) {
		printf("[Warning] [sd_logger_init] SD card logger close log file failed RET=%d (f_open)\r\n", fres);
		return SD_ERR;
	}

	fres = f_mount(NULL, "", 0);
	if(fres != FR_OK) {
		printf("[Warning] [sd_logger_unmount] SD card logger unmount failed RET=%d (f_mount)\r\n", fres);
		return SD_ERR;
	}

	return SD_OK;
}

