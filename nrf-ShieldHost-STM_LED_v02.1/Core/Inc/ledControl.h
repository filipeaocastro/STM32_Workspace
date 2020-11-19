#ifndef ledControl_h
#define ledControl_h

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/** Led colors
 */
typedef enum
{
                    //  RGB
    RED = 0x04,     // 0100
    GREEN = 0x02,   // 0010
    BLUE = 0x01,    // 0001

    PURPLE = 0x05,  // 0101
    CYAN = 0x03,    // 0011
    YELLOW = 0x06,  // 0110

    WHITE = 0x07,   // 0111
    OFF = 0x00     // 0000
} led_color;


static led_color cor_atual = OFF;   // The actual LED color

uint16_t pin_R, pin_G, pin_B;   // Pins relating to each color of the LED
GPIO_TypeDef* port_R;   // Port of each pin of the LED
GPIO_TypeDef* port_G;
GPIO_TypeDef* port_B;

void init_led(uint16_t _pin_R, GPIO_TypeDef* _port_R, uint16_t _pin_G, GPIO_TypeDef* _port_G, uint16_t _pin_B, GPIO_TypeDef* _port_B);
void acende_led(led_color cor);
void apaga_led(void);
void toggle_led(led_color cor);
void hold_led(led_color cor, uint16_t milliseconds);


#ifdef __cplusplus
}
#endif

#endif
