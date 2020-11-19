#include "ledControl.h"

/** LED Description:
 * 
 *  RED:    Initiating
 *  GREEN:  Free to send RF messages (rfBridgeON == 1)
 *  CYAN: RF communication disabled until handshake (rfBridgeON == 0)
 *  BLUE:   Sending recieved messaged via RF to host via USB
 *  PURPLE: Handshake
 *  YELLOW: Sending message via RF
 * 
 */ 

/** Initiate the library by setting the pins and ports relative to each color
 * @param _pin_R The pin related to the RED color
 * @param _port_R The port of the RED color pin
 * @param _pin_G The pin related to the GREEN color
 * @param _port_G The port of the GREEN color pin
 * @param _pin_B The pin related to the BLUE color
 * @param _port_B The port of the BLUE color pin
 */
void init_led(uint16_t _pin_R, GPIO_TypeDef* _port_R, uint16_t _pin_G, GPIO_TypeDef* _port_G, uint16_t _pin_B, GPIO_TypeDef* _port_B)
{
    pin_R = _pin_R;
    pin_G = _pin_G;
    pin_B = _pin_B;

    port_R = _port_R;
    port_G = _port_G;
    port_B = _port_B;
}

/**
 * Turns on the LED with the especified color
 * @param cor The color
 */ 
void acende_led(led_color cor)
{
    HAL_GPIO_WritePin(port_R, pin_R, (cor & RED));
    HAL_GPIO_WritePin(port_G, pin_G, (cor & GREEN));
    HAL_GPIO_WritePin(port_B, pin_B, (cor & BLUE));

    //if(cor != OFF) led_aceso = 1;
    //else led_aceso = 0;
        
    cor_atual = cor;
}

// Turns off the LED
void apaga_led(void)
{
    acende_led(OFF);
}

/**
 * Toggle the LED state, if it's on, turns it off. If its off, turn on with the specified color
 * @param cor The color
 */ 
void toggle_led(led_color cor)
{
    if(cor_atual == OFF)
        acende_led(cor);
    else
        apaga_led();
}

/**
 * Hold the LED on during the time specified in milliseconds
 * @param cor The LED color
 * @param milliseconds Number of milliseconds that the LED will stay on
 */ 
void hold_led(led_color cor, uint16_t milliseconds)
{
    acende_led(cor);
    HAL_Delay(milliseconds);
    apaga_led();
}

