/*
 * led-rgb.h
 *
 *  Created on: Jan 16, 2020
 *      Author: Filipe Augusto
 */

#ifndef INC_LED_RGB_H_
#define INC_LED_RGB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define AZUL 	 0x01
#define VERDE 	 0x02
#define CIANO 	 0x03
#define VERMELHO 0x04
#define ROSA 	 0x05
#define AMARELO  0x06
#define BRANCO 	 0x07

void ligar(uint8_t cor);
void apagar(void);



#ifdef __cplusplus
}
#endif

#endif /* INC_LED_RGB_H_ */