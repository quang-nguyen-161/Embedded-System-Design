/*
 * uart_handle.c
 *
 *  Created on: Jan 28, 2026
 *      Author: Windows
 */

#include "uart_handle.h"

extern UART_HandleTypeDef huart1;
extern uint32_t period;
extern uint8_t period_change;
extern TIM_HandleTypeDef htim1;
typedef enum {
    EVENT_NONE = 0,
    EVENT_UART,
    EVENT_OLED,
    EVENT_TEMP,
    EVENT_MOISTURE
} EventType;

#define EVENT_QUEUE_SIZE 10

typedef struct {
    EventType buffer[EVENT_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
} EventQueue;

extern EventQueue uart_event_queue;


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

void event_queue_push(EventQueue *q, EventType event) {
    uint8_t next = (q->tail + 1) % EVENT_QUEUE_SIZE;
    if (next != q->head)
    {
        q->buffer[q->tail] = event;
        q->tail = next;
    }
}

EventType event_queue_pop(EventQueue *q) {
    if (q->head == q->tail) return EVENT_NONE;
    EventType event = q->buffer[q->head];
    q->head = (q->head + 1) % EVENT_QUEUE_SIZE;
    return event;
}

void uart_rx_IT() {
    uart_rx_index = 0;
    memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
    HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
}

void TIM1_SetPeriod(uint32_t period_ms)
{
    uint32_t psc = 7999;
    uint32_t arr = period_ms - 1;

    HAL_TIM_Base_Stop_IT(&htim1);

    __HAL_TIM_SET_PRESCALER(&htim1, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    HAL_TIM_Base_Start_IT(&htim1);
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

void uart_rx_process(char *data)
{
	uint32_t value;
	if (strncmp(data, "task: uart.", sizeof("task: uart.") - 1) == 0)
	{
		event_queue_push(&uart_event_queue, EVENT_UART);
		serial_print("Queued: UART event\r\n");
	}
	else if (strncmp(data, "task: oled.", sizeof("task: oled.") - 1) == 0)
	{
		event_queue_push(&uart_event_queue, EVENT_OLED);
		serial_print("Queued: OLED event\r\n");
	}
	else if (strncmp(data, "task: temp.", sizeof("task: temp.") - 1) == 0)
	{
		event_queue_push(&uart_event_queue, EVENT_TEMP);
		serial_print("Queued: TEMP event\r\n");
	}
	else if (strncmp(data, "task: moist.", sizeof("task: moist.") - 1) == 0)
	{
		event_queue_push(&uart_event_queue, EVENT_MOISTURE);
		serial_print("Queued: MOISTURE event\r\n");
	}
	else if (sscanf(data, "change_period: %lu.", &value) == 1)
		    	{
		    	    TIM1_SetPeriod(value);
		    	    period = value;
		    	    serial_print("period = %lu ms\r\n", value);
		    	}

	else
	{
		serial_print("Invalid command format\r\n");
	}
}
