/*
 * led-rgb.c
 *
 *  Created on: Jan 16, 2020
 *      Author: Filipe Augusto
 */

#include "led-rgb.h"

void ligar(uint8_t cor)
{
	uint8_t RGB[3] = {0x04, 0x02, 0x01};
	RGB[0] = RGB[0] & cor;
	RGB[1] = RGB[1] & cor;
	RGB[2] = RGB[2] & cor;

	for(int i = 0; i < 3; i++)
		if(RGB[i] != 0)
			RGB[i] = 1;


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RGB[0]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RGB[1]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RGB[2]);
}

void apagar()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}
