#include "bluetooth_service.h"

uint8_t bt_buf[BT_BUF_SIZE];

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

uint8_t bt_send(void* handle, bt_header* header, void* data) {
	if (header->len < 0 || header->len == SYNC_BYTE) {
		return 1;
	}
	uint8_t* data_bytes = (uint8_t*)data;
	bt_buf[0] = SYNC_BYTE; // synchronization byte
	bt_buf[1] = header->cmd;

	uint16_t buf_len = sizeof(uint8_t) + sizeof(bt_header);
	for (uint8_t i = 0; i < header->len; i++) {
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

	header->len = buf_len - sizeof(char) - sizeof(bt_header);
	bt_buf[2] = header->len;

	if (HAL_UART_Transmit(handle, bt_buf, buf_len, BT_SEND_TIMEOUT) != 0) {
		return 1;
	}

	return 0;
}

uint8_t bt_send_impulse(void* handle, float impulse) {
	bt_header header;
	header.cmd = IMPULSE_CMD;
	header.len = sizeof(float);
	return bt_send(handle, &header, &impulse);
}

uint8_t bt_send_angles(void*handle, triple_axis_angle* angles, uint8_t len) {
	bt_header header;
	header.cmd = ANGLE_CMD;
	header.len = sizeof(triple_axis_angle) * len;
	return bt_send(handle, &header, angles);
}

uint8_t bt_recv(void* handle, bt_header* header, void* data) {
	uint8_t cmd;
	uint8_t len;

	do {
		uint8_t sync;
		do {
			if (HAL_UART_Receive(handle, &sync, sizeof(uint8_t), BT_RECV_TIMEOUT) != 0) {
				return 1;
			}
		} while (sync != SYNC_BYTE);
		if (HAL_UART_Receive(handle, &cmd, sizeof(uint8_t), BT_RECV_TIMEOUT) != 0) {
			return 1;
		}
	} while (cmd == SYNC_BYTE);

	if (HAL_UART_Receive(handle, &len, sizeof(uint8_t), BT_RECV_TIMEOUT) != 0) {
		return 1;
	}
	if (HAL_UART_Receive(handle, bt_buf, len, BT_RECV_TIMEOUT) != 0) {
		return 1;
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
