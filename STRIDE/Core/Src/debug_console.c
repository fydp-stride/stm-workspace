#include "debug_console.h"

static UART_HandleTypeDef* handle;
static void (*cmd_cb)(uint8_t, char*);

static uint8_t char_buf;

char dbg_buf[DBG_BUF_SIZE];
uint16_t dbg_buf_len = 0;


void dbg_init(UART_HandleTypeDef* dev_huart, void (*cmd_callback)(uint8_t, char*)) {
    handle = dev_huart;
    cmd_cb = cmd_callback;

    HAL_UART_Receive_IT(handle, &char_buf, sizeof(char));
}

void dbg_callback() {
    if (char_buf == '\r') {
        // enter key pressed
        printf("\r\n");

        dbg_buf[dbg_buf_len] = '\0';

        // split string into command and value
        char* cmd_str = strtok(dbg_buf, " ");
        char* value = strtok(NULL, "");

        uint8_t cmd = 0;
        if (strcmp(cmd_str, DBG_LPTHRESH_CMD_STR) == 0) {
            cmd = DBG_LPTHRESH_CMD;
        } else if (strcmp(cmd_str, DBG_CALIBRATE_CMD_STR) == 0) {
            cmd = DBG_CALIBRATE_CMD;
        }

        (*cmd_cb)(cmd, value);

        dbg_buf_len = 0;
    } else if (char_buf == '\x7F' || char_buf == '\b') {
        // backspace key pressed
        if (dbg_buf_len > 0) {
            printf("\b \b");
            dbg_buf_len--;
        }
    } else if (char_buf >= ' ' && char_buf <= '~') {
        // space or any other physical symbol pressed
        printf("%c", char_buf);

        dbg_buf[dbg_buf_len] = (char)char_buf;
        dbg_buf_len++;
    }

    HAL_UART_Receive_IT(handle, &char_buf, sizeof(char));
}
