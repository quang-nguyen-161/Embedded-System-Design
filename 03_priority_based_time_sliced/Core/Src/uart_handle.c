/*
 * uart_handle.c
 *
 *  Created on: Jan 28, 2026
 *      Author: Windows
 */

#include "uart_handle.h"

extern UART_HandleTypeDef huart1;

extern uint32_t AHT10_PERIOD;
extern uint32_t SOIL_PERIOD;
extern uint32_t OLED_PERIOD;
extern uint32_t UART_PERIOD;

extern osThreadId aht10_taskHandle;
extern osThreadId soil_taskHandle;
extern osThreadId oled_taskHandle;
extern osThreadId uart_taskHandle;


void serial_print(const char *format,...)
{
	char buff[128];
	va_list args;
	va_start(args, format);
	int len = vsnprintf(buff, sizeof(buff), format, args);
	va_end(args);

	if (len > 0) {
	    if (len > sizeof(buff)) len = sizeof(buff);
	    HAL_UART_Transmit(&huart1, (uint8_t*)buff, len, 500);
	}
}


void uart_rx_IT() {
    uart_rx_index = 0;
    memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
    HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
}

// Hàm callback ngắt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (uart_rx_index < UART_RX_BUFFER_SIZE - 1) {
            uart_rx_buffer[uart_rx_index++] = uart_rx_data;

            if (uart_rx_data == '.') {
                uart_rx_buffer[uart_rx_index] = '\0';
                uart_rx_process((char *)uart_rx_buffer);
                uart_rx_index = 0;
                memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
            }
        } else {
            uart_rx_index = 0;
        }

        HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
    }
}

void uart_rx_process(char *data) {
	 uint32_t value;
	 char buffer[128];

	    if (sscanf(data, "Change_AHT: %lu.", &value) == 1) {
	        AHT10_PERIOD = value;
	        serial_print("Changed AHT delay!\r\n");
    } else if (sscanf(data, "Change_SOIL: %lu.", &value) == 1) {
    	SOIL_PERIOD = value;
    	serial_print("Changed SOIL delay!\r\n");
    } else if (sscanf(data, "Change_OLED: %lu.", &value) == 1) {
    	OLED_PERIOD = value;
    	serial_print("Changed OLED delay!\r\n");
    } else if (sscanf(data, "Change_UART: %lu.", &value) == 1) {
    	UART_PERIOD = value;
    	serial_print("Changed UART delay!\r\n");
    }
    else if (strncmp(data, "GetPriority.", 12) == 0) {
            // Lấy mức ưu tiên các task
            serial_print("AHT: %d\r\n", osThreadGetPriority(aht10_taskHandle));
            serial_print("SOIL: %d\r\n", osThreadGetPriority(soil_taskHandle));
            serial_print("OLED: %d\r\n", osThreadGetPriority(oled_taskHandle));
            serial_print("UART: %d\r\n", osThreadGetPriority(uart_taskHandle));
        }

	    else {
        serial_print("Invalid command format.\r\n");
    }

}
