#include "bluetooth_service.h"

uint8_t bt_buf[BT_BUF_SIZE];

uint8_t bt_recv_buf[BT_BUF_SIZE];
uint16_t bt_recv_stage= 0;
uint8_t is_busy_receiving = 0;

bt_header bt_recv_user_mass_hdr;

// uint8_t bt_send_spi(void* handle, void* buf, uint16_t len) {
// 	HAL_GPIO_WritePin (BT_CS_GPIO_Port, BT_CS_Pin, GPIO_PIN_RESET);
// 	uint8_t res = HAL_SPI_Transmit (handle, buf, len, BT_SEND_TIMEOUT);
// 	HAL_GPIO_WritePin (BT_CS_GPIO_Port, BT_CS_Pin, GPIO_PIN_SET);
// 	return res;
// }

// uint8_t bt_recv_spi(void* handle, void* buf, uint16_t len) {
// 	HAL_GPIO_WritePin (BT_CS_GPIO_Port, BT_CS_Pin, GPIO_PIN_RESET);
// 	uint8_t res = HAL_SPI_Receive (handle, buf, len, BT_SEND_TIMEOUT);
// 	HAL_GPIO_WritePin (BT_CS_GPIO_Port, BT_CS_Pin, GPIO_PIN_SET);
// 	return res;
// }

static void print_hex(uint8_t* data, uint32_t len) {
	for (uint32_t i = 0; i < len; i++) {
		printf("%02x ", data[i]);
	}
	printf("\r\n");
}

uint8_t bt_send(void* handle, uint8_t command, void* data, uint8_t len) {
	if (len < 0 || len == SYNC_BYTE) {
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

	print_hex(bt_buf, buf_len);
	if (HAL_UART_Transmit(handle, bt_buf, buf_len, BT_SEND_TIMEOUT) != 0) {
		return BT_ERR;
	}

	return BT_OK;
}

uint8_t bt_send_float(void* handle, uint8_t command, float value) {
	return bt_send(handle, command, &value, sizeof(float));
}

uint8_t bt_send_float_array(void*handle, uint8_t command, float* array, uint8_t len) {
	return bt_send(handle, command, array, sizeof(float) * len);
}

uint8_t bt_send_str_float(void* handle, uint8_t command, float value) {
	char* str_buf = (char*)(bt_buf + sizeof(bt_header));
	bt_header* header_buf = (bt_header*)bt_buf;
	header_buf->sync = 0xff;
	header_buf->cmd = command;
	header_buf->len = sprintf(str_buf, "%.2f", value);
	if (header_buf->len == -1) {
		return BT_ERR;
	}
	HAL_UART_Transmit(handle, bt_buf, header_buf->len + sizeof(bt_header), BT_SEND_TIMEOUT);
	return BT_OK;
}

uint8_t bt_send_str_float_array(void* handle, uint8_t command, float* array, uint8_t len) {
	if (len == 0) {
		return BT_ERR;
	}
	bt_header* header_buf = (bt_header*)bt_buf;
	header_buf->sync = 0xff;
	header_buf->cmd = command;
	header_buf->len = 0;
	for (int i = 0; i < len; i++) {
		char* str_buf = (char*)(bt_buf + sizeof(bt_header) + header_buf->len);
		int res = sprintf(str_buf, "%.2f,", array[i]);
		if (res == -1) {
			return BT_ERR;
		}
		header_buf->len += res;
	}
	HAL_UART_Transmit(handle, bt_buf, header_buf->len + sizeof(bt_header), BT_SEND_TIMEOUT);
	return BT_OK;
}

uint8_t bt_recv(void* handle, bt_header* header, void* data) {
	uint8_t cmd;
	uint8_t len;

	do {
		uint8_t sync;
		do {
			if (HAL_UART_Receive(handle, &sync, sizeof(uint8_t), BT_RECV_TIMEOUT) != 0) {
				return BT_ERR;
			}
		} while (sync != SYNC_BYTE);
		if (HAL_UART_Receive(handle, &cmd, sizeof(uint8_t), BT_RECV_TIMEOUT) != 0) {
			return BT_ERR;
		}
	} while (cmd == SYNC_BYTE);

	if (HAL_UART_Receive(handle, &len, sizeof(uint8_t), BT_RECV_TIMEOUT) != 0) {
		return BT_ERR;
	}
	if (HAL_UART_Receive(handle, bt_buf, len, BT_RECV_TIMEOUT) != 0) {
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	is_busy_receiving = 0;
	bt_recv_stage++;
}

//uint8_t bt_process_recv(void* recv_buf, uint8_t len, bt_header* header, void* data) {
//	// +1 sync byte
//	if (len < sizeof(bt_header) + 1) {
//		return BT_ERR;
//	}
//	header->cmd = recv_buf[0];
//	header->len = recv_buf[1];
//	// +1 sync byte
//	if (sizeof(bt_header) + header->len + 1 != len) {
//		return BT_ERR;
//	}
//	uint8_t data_len = 0;
//	uint8_t i = 0;
//	while (i < len) {
//		if (bt_buf[i] == SYNC_BYTE) {
//			data[data_len] = SYNC_BYTE;
//			i += 2;
//		} else {
//			data[data_len] = bt_buf[i];
//			i++;
//		}
//		data_len++;
//	}
//
//	return 0;
//}

uint8_t bt_try_recv(void* handle, bt_header* header, void* data, uint8_t max_len) {
	if (is_busy_receiving) {
		return BT_BUSY;
	}
	uint8_t has_received_msg = 0;
	if ((bt_recv_stage == BT_RECV_SYNC && bt_recv_buf[0] != SYNC_BYTE) || (bt_recv_stage == BT_RECV_CMD && bt_recv_buf[0] == SYNC_BYTE)) {
		bt_recv_stage = BT_RECV_RESET;
	} else if (bt_recv_stage == BT_RECV_CMD) {
		header->cmd = bt_recv_buf[0];
	} else if (bt_recv_stage == BT_RECV_LEN) {
		header->len = bt_recv_buf[0];
	} else if (bt_recv_stage == BT_RECV_DATA) {
		char* data_bytes = (char*)data;
		uint8_t data_len = 0;
		uint8_t i = 0;
		while (i < header->len && data_len <= max_len) {
			if (bt_recv_buf[i] == SYNC_BYTE) {
				data_bytes[data_len] = SYNC_BYTE;
				i += 2;
			} else {
				data_bytes[data_len] = bt_recv_buf[i];
				i++;
			}
			data_len++;
		}
		if (data_len <= max_len) {
			header->len = data_len;
			has_received_msg = 1;
		}
		bt_recv_stage = BT_RECV_RESET;
	}
	is_busy_receiving = 1;
	if (bt_recv_stage < BT_RECV_LEN) {
		HAL_UART_Receive_IT(handle, bt_recv_buf, sizeof(uint8_t));
	} else {
		HAL_UART_Receive_IT(handle, bt_recv_buf, (uint16_t)header->len);
	}

	return has_received_msg ? BT_OK : BT_BUSY;
}

uint8_t bt_try_recv_user_mass(void* handle, float* user_mass) {
	uint8_t res = bt_try_recv(handle, &bt_recv_user_mass_hdr, user_mass, sizeof(float));
	return (res == BT_OK && bt_recv_user_mass_hdr.cmd == BT_WEIGHT_CMD) ? BT_OK : BT_BUSY;
}
