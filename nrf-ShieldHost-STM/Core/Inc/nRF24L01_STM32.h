#ifndef nRF24L01_STM32_h
#define nRF24L01_STM32_h

#ifdef __cplusplus
extern "C" {
#endif

#include "nRF24L01.h"
#include "stm32f1xx_hal.h"


// Definições da rotina de interrupção.
//Bits do Status Register do nRFL01:
//Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFOb. Write 1 to clear bit.
#define RX_DR               0x40
//Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX. If AUTO_ACK is activated,
//this bit is set high only when ACK is received. Write 1 to clear bit.
#define TX_DS               0x20
//Maximum number of TX retransmits interrupt Write 1 to clear bit. 
//If MAX_RT is asserted it must be cleared to enable further communication.
#define MAX_RT              0x10

#define PAYLOAD_WIDTH       32    // 30 bytes on TX payload
#define TX_RX_ADDR_WIDTH    5     // 5 bytes TX(RX) address width 


/** Available data rates
 * The input argument of rf_init must be defined in this @c enum
 */
typedef enum
{
    RF_DATA_RATE_1Mbps,
    RF_DATA_RATE_2Mbps,
    RF_DATA_RATE_250kbps
} rf_data_rate_t;

/** Available tx power modes
 * The input argument of rf_init must be defined in this @c enum
 */
typedef enum
{
    RF_TX_POWER_NEGATIVE_18dBm,
    RF_TX_POWER_NEGATIVE_12dBm,
    RF_TX_POWER_NEGATIVE_6dBm,
    RF_TX_POWER_0dBm
} rf_tx_power_t;




/* data */

/*
Pinos padrão
*/

// Pino do STM onde está conectado o CSN (GPIO_Output)

#define _RF_CSN_Pin         GPIO_PIN_4
#define _RF_CSN_GPIO_Port   GPIOA

// Pino do STM onde está conectado o CE (GPIO_Output)
#define _RF_CE_Pin          GPIO_PIN_3
#define _RF_CE_GPIO_Port    GPIOA

// Pino do STM onde está conectado o IRQ (GPIO_EXTI - FALLING)
#define _RF_IRQ_Pin         GPIO_PIN_0
#define _RF_IRQ_GPIO_Port   GPIOB

// Pino do STM onde está conectado o SCK (Comunicação SPI)
#define _RF_SCK_Pin         GPIO_PIN_5
#define _RF_SCK_GPIO_Port   GPIOA

// Pino do STM onde está conectado o MISO (Comunicação SPI)
#define _RF_MISO_Pin        GPIO_PIN_6
#define _RF_MISO_GPIO_Port  GPIOA

// Pino do STM onde está conectado o MOSI (Comunicação SPI)
#define _RF_MOSI_Pin        GPIO_PIN_7
#define _RF_MOSI_GPIO_Port  GPIOA

// Pino do STM onde está conectado o LED
#define _RF_LED_Pin        GPIO_PIN_13
#define _RF_LED_GPIO_Port  GPIOC

/**
 * Parâmetros da comunicação SPI
 **/

SPI_HandleTypeDef _spi;

//Endereço de transmissão (0x TX_ADDR). Usando default 0xE7E7E7E7E7 (when changing: LSB written first)
//Endereço de recepção (0x0A RX_ADDR_P0). Usando default 0xE7E7E7E7E7 (when changing: LSB written first)
//Usando o mesmo para RX e TX e será constante:
static uint8_t ADDR_HOST[TX_RX_ADDR_WIDTH] =  {0xE7,0xE7,0xE7,0xE7,0xE7};   // Define a static host adr
//payloads
//static uint8_t rx_buf[PAYLOAD_WIDTH];    // Define lenght of rx_buf and tx_buf
static uint8_t tx_buf[PAYLOAD_WIDTH];
//static uint8_t rx_payloadWidth = 0;
//static uint8_t rx_newPayload = 0;    // Flag to indicate that there's a new payload sensor
static uint8_t status;     // Contains the STATUS register reading
static uint8_t TX_OK = 0;  //Indicar MODO TX ativo (1) ou Inativo (0)
static uint8_t RX_OK = 0;  //Indicar MODO RX ativo (1) ou Inativo (0)


static void SPI_Write(uint8_t command, uint8_t * value);
void SPI_Write_Reg(uint8_t reg, uint8_t * value);
uint8_t SPI_Read(uint8_t command);
uint8_t SPI_Read_Status();
static uint8_t SPI_Read_Reg(uint8_t reg);
void SPI_Read_Buf(uint8_t command, uint8_t *dataBuf, uint16_t size);
void SPI_Write_Buf_Reg(uint8_t reg, uint8_t *value, uint16_t size);
void SPI_Write_Buf(uint8_t command, uint8_t *value, uint16_t size);
void SPI_Read_rx_buf(uint8_t * buf);




/* Constructor */
/*
void nRF24L01_STM32(SPI_HandleTypeDef spi, GPIO_InitTypeDef RF_SCK_Pin, GPIO_InitTypeDef RF_SCK_GPIO_Port,
GPIO_InitTypeDef RF_MISO_Pin, GPIO_InitTypeDef RF_MISO_GPIO_Port,
GPIO_InitTypeDef RF_MOSI_Pin, GPIO_InitTypeDef RF_MOSI_GPIO_Port,
GPIO_InitTypeDef RF_CE_Pin, GPIO_InitTypeDef RF_CE_GPIO_Port,
GPIO_InitTypeDef RF_CSN_Pin, GPIO_InitTypeDef RF_CSN_GPIO_Port,
GPIO_InitTypeDef RF_IRQ_Pin, GPIO_InitTypeDef RF_IRQ_GPIO_Port);*/

/* Constructor default */
void nRF24L01_STM32(SPI_HandleTypeDef spi);

void init(uint8_t rf_channel, rf_data_rate_t rf_data_rate, rf_tx_power_t rf_pwr);
void init_AA_EN(uint8_t rf_channel, rf_data_rate_t rf_data_rate, rf_tx_power_t rf_pwr);
void RX_Mode(void);
void RX_Mode_AA_EN(void);
void TX_Mode_NOACK(uint8_t* buf, uint8_t payloadLength);
void TX_Mode_AA_EN(uint8_t* buf, uint8_t payloadLength);
void RF_IRQ(uint8_t *buf, uint8_t *size, uint8_t *newPayload);
uint8_t getnRFStatus();
uint8_t SPI_read2(uint8_t reg);

















#ifdef __cplusplus
}
#endif

#endif
