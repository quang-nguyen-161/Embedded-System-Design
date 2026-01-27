/*
 * oled.c
 *
 *  Created on: Jan 27, 2026
 *      Author: Windows
 */

#include "oled.h"

extern SPI_HandleTypeDef hspi1;

void send_cmd(uint8_t cmd)
{
	DC_cmd();
	CS_write(0);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	CS_write(1);
}

void send_data(uint8_t data)
{
	DC_data();
	CS_write(0);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	CS_write(1);
}

void oled_init()
{
	CS_write(0);
	RES_write(1);
	send_cmd(0xA8);
		send_cmd(0x3F);

		send_cmd(0xD3);
		send_cmd(0x00);

		send_cmd(0x40);

		send_cmd(0xA1);

		send_cmd(0xC8);

		send_cmd(0xDA);
		send_cmd(0x12);

		send_cmd(0x81);
		send_cmd(0x3F);

		send_cmd(0xA4);

		send_cmd(0xA6);

		send_cmd(0xD5);
		send_cmd(0x80);

		send_cmd(0x8D);
		send_cmd(0x14);

		send_cmd(0xAF);

		send_cmd(0x20);
		send_cmd(0x10);
		HAL_Delay(1000);
}

void oled_pos(uint8_t y_pos, uint8_t x_pos)
{
	send_cmd(0x00 + (0x0F & x_pos));

		//Set higher column address
		send_cmd(0x10 + (0x0F & (x_pos>>4)));

		//Set row address
		send_cmd(0xB0 + y_pos);
}

void oled_clear()
{
	int i,j;
		oled_pos(0,0);
		for (i = 0;i < 8;i++)
				for (j = 0; j < 130; j++)
							{
								oled_pos(i,j);
								send_data(0x00);
							}

}

void oled_print(uint8_t *str)
{
	 int i,j;
		i=0;
		while(str[i])
		{
			for(j=0;j<6;j++)
					{
						send_data(ASCII[(str[i]-32)][j]);
					}
					i++;
		}
}

void oled_msg(uint8_t y_pos, uint8_t x_pos, char * str)
{
	oled_pos(y_pos, x_pos);
	oled_print(str);
}
