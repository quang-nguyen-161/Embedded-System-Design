/*
 * AHT10.c
 *
 *  Created on: Nov 6, 2025
 *      Author: Windows
 */

#include "aht10.h"


extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;


void aht10_init()
{
	uint8_t init_cmd[3] = {0xAC, 0x33, 0x00}; //init, data0, data1
	HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADDRESS, init_cmd, sizeof(init_cmd), 100);
	HAL_Delay(80);
}

int aht10_get_data(sensor_typedef *m_sensor)
{
	uint8_t receive_buff[6] = {0};
	uint32_t raw_RH;
	uint32_t raw_temp;

	aht10_init();

	HAL_I2C_Master_Receive(&hi2c1, AHT10_ADDRESS, receive_buff, sizeof(receive_buff), 100);

	if (!(receive_buff[0] & 0x80))
	{
		 raw_RH = ((uint16_t)receive_buff[1] << 12) |
		                 ((uint16_t)receive_buff[2] << 4)  |
		                 (receive_buff[3] >> 4);
		     m_sensor->humidity =(uint16_t) (((float)raw_RH * 100.0f / 1048576.0f)*100);

		        raw_temp = (((uint16_t)receive_buff[3] & 0x0F) << 16) |
		                    ((uint16_t)receive_buff[4] << 8)         |
		                     receive_buff[5];
		     m_sensor->temp =(uint16_t) (((float)raw_temp * 200.0f / 1048576.0f - 50.0f)*100);
		     return 1;
	}
	return 1;
}

uint16_t read_adc_once(ADC_HandleTypeDef *hadc)
{
    HAL_ADC_Start(hadc);

    // wait for conversion
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
        return 0; // timeout or error

    uint16_t val = HAL_ADC_GetValue(hadc);

    HAL_ADC_Stop(hadc);

    return val;
}

uint16_t adc_get()
{
	    uint32_t sum = 0;

	    /* ---- Take 10 ADC samples ---- */
	    for (uint8_t i = 0; i < 10; i++)
	    {
	        sum += read_adc_once(&hadc1);
	        HAL_Delay(10);
	    }

	    /* ---- Average ADC value ---- */
	    uint32_t adc = sum / 10;
	    return (uint16_t) adc;
}

uint16_t moisture_get(
                      uint32_t MOISTURE_DRY,
                      uint32_t MOISTURE_WET, sensor_typedef *m_sensor)
{
    uint32_t sum = 0;

    /* ---- Take 10 ADC samples ---- */
    for (uint8_t i = 0; i < 10; i++)
    {
        sum += read_adc_once(&hadc1);
        HAL_Delay(100);
    }

    /* ---- Average ADC value ---- */
    uint32_t adc = sum / 10;

    int32_t pct_x100;

    /* ---- ADC → moisture percent ×100 ---- */
    if (MOISTURE_WET < MOISTURE_DRY)
    {
        // Typical soil sensor: wet ADC < dry ADC
        pct_x100 = (int32_t)(MOISTURE_DRY - adc) * 10000L /
                   (int32_t)(MOISTURE_DRY - MOISTURE_WET);
    }
    else
    {
        pct_x100 = (int32_t)(adc - MOISTURE_DRY) * 10000L /
                   (int32_t)(MOISTURE_WET - MOISTURE_DRY);
    }

    /* ---- Clamp to 0–100.00% ---- */
    if (pct_x100 < 0)      pct_x100 = 0;
    if (pct_x100 > 10000)  pct_x100 = 10000;
    m_sensor->soil_moisture = (uint16_t)pct_x100;
    return (uint16_t)pct_x100;
}



