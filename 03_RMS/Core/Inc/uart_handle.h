/*
 * uart_handle.h
 *
 *  Created on: Jan 28, 2026
 *      Author: Windows
 */

#ifndef INC_UART_HANDLE_H_
#define INC_UART_HANDLE_H_

#include "main.h"
#include "cmsis_os.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UART_RX_BUFFER_SIZE 64

static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_data;
static uint16_t uart_rx_index = 0;

void serial_print(const char *format,...);
void uart_rx_process(char *data);
void uart_rx_IT();

#endif /* INC_UART_HANDLE_H_ */
