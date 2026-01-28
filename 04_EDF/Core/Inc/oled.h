/*
 * oled.h
 *
 *  Created on: Jan 27, 2026
 *      Author: Windows
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include "main.h"

#include "fonts.h"

#define DC_cmd() HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 0)
#define DC_data() HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 1)
#define CS_write(x) HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, x)
#define RES_write(x) HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, x)

void oled_init();
void oled_pos(uint8_t y_pos, uint8_t x_pos);
void oled_clear();
void oled_print(uint8_t *str);
void oled_msg(uint8_t y_pos, uint8_t x_pos, char *str);
#endif /* INC_OLED_H_ */
