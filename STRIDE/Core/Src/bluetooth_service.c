#include "bluetooth_service.h"

static UART_HandleTypeDef* handle;
static void (*msg_cb)(bt_header*, uint8_t*);

static uint8_t bt_buf[BT_BUF_SIZE];

static uint8_t bt_recv_buf[BT_BUF_SIZE];
static uint8_t bt_recv_stage = 0;

bt_header bt_recv_hdr;
uint8_t bt_recv_data[BT_BUF_SIZE];

void bt_init(UART_HandleTypeDef* dev_huart, void (*msg_callback)(bt_header*, uint8_t*)) {
	handle = dev_huart;
	msg_cb = msg_callback;

	bt_recv_stage = 0;
	// starting waiting for messages to be received
	// the first byte will likely be a sync byte
	HAL_UART_Receive_IT(handle, bt_recv_buf, sizeof(uint8_t));
}

uint8_t bt_send(uint8_t command, void* data, uint8_t len) {
	// bluetooth module is not able to send more than 95 bytes of data (excluding header size)
	// all at once.
	if (len > BT_MAX_DATA_SIZE) {
		return BT_ERR;
	}
	uint8_t* data_bytes = (uint8_t*)data;
	bt_header* header_buf = (bt_header*)bt_buf;
	header_buf->sync = SYNC_BYTE; // synchronization byte
	header_buf->cmd = command;

	uint16_t buf_len = sizeof(bt_header);
	for (uint8_t i = 0; i < len; i++) {
		if (data_bytes[i] == SYNC_BYTE) {
			// escape 0xAA by repeating it
			bt_buf[buf_len] = SYNC_BYTE;
			bt_buf[buf_len + 1] = SYNC_BYTE;
			buf_len += 2;
		} else {
			bt_buf[buf_len] = data_bytes[i];
			buf_len++;
		}
	}

	header_buf->len = buf_len - sizeof(bt_header);

	if (HAL_UART_Transmit(handle, bt_buf, buf_len, BT_SEND_TIMEOUT) != 0) {
		return BT_ERR;
	}

	return BT_OK;
}

uint8_t bt_send_float(uint8_t command, float value) {
	return bt_send(command, &value, sizeof(float));
}

uint8_t bt_send_float_array(uint8_t command, float* array, uint8_t len) {
	return bt_send(command, array, sizeof(float) * len);
}

uint8_t bt_send_str_value(uint8_t command, char* value, int len) {
	if (len <= 0) {
		return BT_ERR;
	}
	char* str_buf = (char*)(bt_buf + sizeof(bt_header));
	strncpy(str_buf, value, len);
	bt_header* header_buf = (bt_header*)bt_buf;
	header_buf->sync = SYNC_BYTE;
	header_buf->cmd = command;
	header_buf->len = len;
	
	// for (unsigned int i = 0; i < (len + sizeof(bt_header)); i++) {
	// 	printf("%x ", bt_buf[i]);
	// }
	// printf("(");
	// for (unsigned int i = sizeof(bt_header); i < (len + sizeof(bt_header)); i++) {
	// 	printf("%c", bt_buf[i]);
	// }
	// printf(")\r\n");

	HAL_UART_Transmit(handle, bt_buf, header_buf->len + sizeof(bt_header), BT_SEND_TIMEOUT);
	return BT_OK;
}

uint8_t bt_send_str_uint16(uint8_t command, uint16_t value) {
	char str_buf[BT_BUF_SIZE];
	int len = sprintf(str_buf, "%d", value);
	return bt_send_str_value(command, str_buf, len);
}

uint8_t bt_send_str_float(uint8_t command, float value) {
	char str_buf[BT_BUF_SIZE];
	int len = sprintf(str_buf, "%.2f", value);
	return bt_send_str_value(command, str_buf, len);
}

uint8_t bt_send_str_float_array(uint8_t command, float* array, uint8_t len) {
	if (len == 0) {
		return BT_ERR;
	}
	char str_buf[BT_BUF_SIZE];
	int str_len = 0;
	for (int i = 0; i < len; i++) {
		char* buf_offset = (char*)(str_buf + str_len);
		int res = sprintf(buf_offset, "%.2f,", array[i]);
		if (res == -1) {
			return BT_ERR;
		}
		str_len += res;
	}
	return bt_send_str_value(command, str_buf, str_len);
}

uint8_t bt_recv(bt_header* header, void* data) {
	uint8_t cmd;
	uint8_t len;

	do {
		uint8_t sync;
		do {
			if (HAL_UART_Receive(handle, &sync, sizeof(uint8_t), BT_RECV_TIMEOUT) != 0) {
				printf("No sync\r\n");
				return BT_ERR;
			}
		} while (sync != SYNC_BYTE);
		if (HAL_UART_Receive(handle, &cmd, sizeof(uint8_t), BT_RECV_TIMEOUT) != 0) {
			printf("No command\r\n");
			return BT_ERR;
		}
	} while (cmd == SYNC_BYTE);

	if (HAL_UART_Receive(handle, &len, sizeof(uint8_t), BT_RECV_TIMEOUT) != 0) {
		printf("No length\r\n");
		return BT_ERR;
	}
	if (HAL_UART_Receive(handle, bt_buf, len, BT_RECV_TIMEOUT) != 0) {
		printf("No data\r\n");
		return BT_ERR;
	}
	char* data_bytes = (char*)data;
	uint8_t data_len = 0;
	uint8_t i = 0;
	while (i < len) {
 		if (bt_buf[i] == SYNC_BYTE) {
 			data_bytes[data_len] = SYNC_BYTE;
			i += 2;
		} else {
			data_bytes[data_len] = bt_buf[i];
			i++;
		}
 		data_len++;
	}
	header->cmd = cmd;
	header->len = data_len;

	return 0;
}

void bt_recv_callback()
{
	// there are 4 stages to receiving a bluetooth message
	// 0 -> RESET
	// 1 -> Receive SYNC
	// 2 -> Receive CMD
	// 3 -> Receive LEN
	// 4 -> Receive DATA
	// the bluetooth state is incremented each time the UART receive callback is triggered
	bt_recv_stage++;

	if ((bt_recv_stage == BT_RECV_SYNC && bt_recv_buf[0] != SYNC_BYTE) || (bt_recv_stage == BT_RECV_CMD && bt_recv_buf[0] == SYNC_BYTE)) {
		// If we don't get a sync byte, or we get a misplaced sync byte, return to RESET
		bt_recv_stage = BT_RECV_RESET;
	} else if (bt_recv_stage == BT_RECV_CMD) {
		// Record the command
		bt_recv_hdr.cmd = bt_recv_buf[0];
	} else if (bt_recv_stage == BT_RECV_LEN) {
		// Record the length of the data
		bt_recv_hdr.len = bt_recv_buf[0];
	} else if (bt_recv_stage == BT_RECV_DATA) {
		// Record the data
		char* data_bytes = (char*)bt_recv_data;
		// data_bytes is indexed by data_len because it may be shorter than recv_buf due to duplicate sync bytes
		uint8_t data_len = 0;
		uint8_t i = 0;
		while (i < bt_recv_hdr.len) {
			// two sync bytes are used to escape the sync byte value
			if (bt_recv_buf[i] == SYNC_BYTE) {
				data_bytes[data_len] = SYNC_BYTE;
				i += 2;
			} else {
				data_bytes[data_len] = bt_recv_buf[i];
				i++;
			}
			data_len++;
		}
		bt_recv_hdr.len = data_len;
		bt_recv_stage = BT_RECV_RESET;

		// callback function to handle received message
		(*msg_cb)(&bt_recv_hdr, bt_recv_data);
	}

	// a call to HAL_UART_Receive_IT is followed by a callback to HAL_UART_RxCpltCallback
	if (bt_recv_stage < BT_RECV_LEN) {
		// We are receiving 1 byte header data: sync, cmd or len
		HAL_UART_Receive_IT(handle, bt_recv_buf, sizeof(uint8_t));
	} else if (bt_recv_stage == BT_RECV_LEN && bt_recv_hdr.len == 0) {
		// if message length is 0, there is no data to receive, so just invoke callback right away
		(*msg_cb)(&bt_recv_hdr, bt_recv_data);
	} else {
		// We are receiving the data
		HAL_UART_Receive_IT(handle, bt_recv_buf, (uint16_t)bt_recv_hdr.len);
	}
}
