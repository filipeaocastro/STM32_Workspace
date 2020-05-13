#include "nRF24L01_STM32.h"

/**
 * Construtor para definir os pinos da comunicação SPI e os pinos CE, CSN e IRQ no nRF24L01
 * @param RF_SCK_Pin 
 * @param RF_SCK_GPIO_Port 
 * @param RF_MISO_Pin
 * @param RF_MISO_GPIO_Port
 * @param RF_MOSI_Pin
 * @param RF_MOSI_GPIO_Port
 * @param RF_CE_Pin
 * @param RF_CE_GPIO_Port
 * @param RF_CSN_Pin
 * @param RF_CSN_GPIO_Port
 * @param RF_IRQ_Pin
 * @param RF_IRQ_GPIO_Port
 * 
 * Obs.: Pressupõe-se que os pinos CE e CSN já estejam definidos como GPIO_Output, 
 * e o pino IRQ como interrupção externa
*/
/*nRF24L01_STM32(SPI_HandleTypeDef spi,
GPIO_InitTypeDef RF_SCK_Pin, GPIO_InitTypeDef RF_SCK_GPIO_Port, 
GPIO_InitTypeDef RF_MISO_Pin, GPIO_InitTypeDef RF_MISO_GPIO_Port,
GPIO_InitTypeDef RF_MOSI_Pin, GPIO_InitTypeDef RF_MOSI_GPIO_Port,
GPIO_InitTypeDef RF_CE_Pin, GPIO_InitTypeDef RF_CE_GPIO_Port,
GPIO_InitTypeDef RF_CSN_Pin, GPIO_InitTypeDef RF_CSN_GPIO_Port,
GPIO_InitTypeDef RF_IRQ_Pin, GPIO_InitTypeDef RF_IRQ_GPIO_Port)
{
    _spi = spi;
    _RF_CSN_Pin = RF_CSN_Pin;
    _RF_CSN_GPIO_Port = RF_CSN_GPIO_Port;
    _RF_CE_Pin = RF_CE_Pin;
    _RF_CE_GPIO_Port = RF_CE_GPIO_Port;
    _RF_IRQ_Pin = RF_IRQ_Pin;
    _RF_IRQ_GPIO_Port = RF_IRQ_GPIO_Port;
    _RF_SCK_Pin = RF_SCK_Pin;
    _RF_SCK_GPIO_Port = RF_SCK_GPIO_Port;
    _RF_MISO_Pin = RF_MISO_Pin;
    _RF_MISO_GPIO_Port = RF_MISO_GPIO_Port;
    _RF_MOSI_Pin = RF_MOSI_Pin;
    _RF_MOSI_GPIO_Port = RF_MOSI_GPIO_Port;
}*/

/**
 * Construtor using the default pins
 * @param spi its an SPI_HandleTypeDef object that contains the SPI informations
 **/
void nRF24L01_STM32(SPI_HandleTypeDef spi)
{
    _spi = spi;
}

/**
 * Inicia a comunicação com o rádio e define alguns parâmetros
 * @param rf_channel	The rf channel that will be used
 * @param rf_data_rate	The radio's data rate in Mbps
 * @param rf_pwr		The radio's power in dB
 */

void init(uint8_t rf_channel, rf_data_rate_t rf_data_rate, rf_tx_power_t rf_pwr, uint8_t autoAck_enable)
{
	// Setup values of the registers
	uint8_t rf_setup_byte;
	uint8_t setup_aw_value = 0x03;	//Setup of Address Widths ('11' - 5 bytes)
	uint8_t en_aa_value;			//Auto Acknowledgment Function on pipe 0
	uint8_t en_rxaddr_value = 0x01;	//Enabled RX Addresses (only pipe 0)
	uint8_t setup_retr_value;		//Setup of Automatic Retransmission
	uint8_t dypnd_value = 0x01;		//Enable dynamic payload length
	uint8_t feature_value = 0x05;	//Feature Register
	uint8_t zero = 0x00;			// 0 (0) (2*5 - 11 + 1)
	uint8_t nrf_status_value = 0x07;// Status

    //uint8_t addr_host[TX_RX_ADDR_WIDTH] = {0xE7,0xE7,0xE7,0xE7,0xE7};

    //Aguardar sequencia de power-up _ start do CI (~12ms) 
    HAL_Delay(20);

    //rx_newPayload = 0;      // Init with no new payload
    //rx_payloadWidth = 0;    // It has no length
    status = 0;             // Stores the STATUS register status
    TX_OK = 0;              // initiates in stand-by
    RX_OK = 0;              // "

    // Set CSN high, no SPI transaction yet
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);
 
    // Disable RX TX
    HAL_GPIO_WritePin(_RF_CE_GPIO_Port, _RF_CE_Pin, GPIO_PIN_RESET);

    //Configuração:

    //W_REGISTER=001A AAAA: Read command and status registers. AAAAA = 5 bit Register Map Address

    // SETUP_AW register: Setup of Address Widths - (common for all data pipes)  
    SPI_Write_Reg(SETUP_AW, &setup_aw_value); //RX/TX Address field width 5 bytes
    // Configuration register é definido quando entra no modo RX ou TX (ver funções para cada modo)

    // EN_RXADDR register: Enable Pipe0 (only pipe0)
    SPI_Write_Reg(EN_RXADDR, &en_rxaddr_value);    // Enable Pipe0 (only pipe0)

    if(autoAck_enable)
    {
    	en_aa_value = 0x01;			// Enabled Auto Acknowledgment on pipe 0
		setup_retr_value = 0x25;	// Enabled retransmission (5 max)

    	// EN_AA register: Enable Auto Acknowledgment: Pipe 0
    	SPI_Write_Reg(EN_AA, &en_aa_value);

    	// SETUP_RETR register: Time to automatic retransmission selected: 250us, retransmission enabled
		SPI_Write_Reg(SETUP_RETR, &setup_retr_value);
    }
    else
    {
    	en_aa_value = 0x00;			// Disabled Auto Acknowledgment
		setup_retr_value = 0x00;	// Disabled retransmission (5 max)

    	// EN_AA register: Disable Auto Acknowledgment
		SPI_Write_Reg(EN_AA, &en_aa_value);        // Disable Auto Acknowledgment: All pipes

		// SETUP_RETR register: Time to automatic retransmission selected: 250us, retransmission disabled
		SPI_Write_Reg(SETUP_RETR, &setup_retr_value);
    }


    // RF_CH register: Select RF channel
    SPI_Write_Reg(RF_CH, &rf_channel);          // Select RF channel: Fo = 2,490 GHz + rf_channel

    //RF SETUP
    //Ajustar potência de saída em modo TX (bits 2:1)
    //  bit 0 = 1 (setup LNA gain)
    rf_setup_byte = 0x01; //0000 0001
    switch (rf_pwr) 
    {     
        case RF_TX_POWER_NEGATIVE_18dBm: //bits 2:1 = 00
            rf_setup_byte &= 0xF9; //1111 1001
        break;

        case RF_TX_POWER_NEGATIVE_12dBm: //bits 2:1 = 01
            rf_setup_byte |= 0x02;//0000 0010
            rf_setup_byte &= 0xFB;//1111 1011 
        break;

        case RF_TX_POWER_NEGATIVE_6dBm: //bits 2:1 = 10
            rf_setup_byte &= 0xFD;//1111 1101
            rf_setup_byte |= 0x04;//0000 0100
        break;

        case RF_TX_POWER_0dBm: //bits 2:1 = 11
            rf_setup_byte |= 0x06;//0000 0110
        break;

        default: 
        break;      
        }
        //Ajustar Air Data Rate (bit 3)
        switch (rf_data_rate) 
        {
        case RF_DATA_RATE_1Mbps: //bit 3 = 0
            rf_setup_byte &= 0xF7;//1111 0111
        break;
        case RF_DATA_RATE_2Mbps: //bit 3 = 1
            rf_setup_byte |= 0x08;//0000 1000
        break;
        }

    //Bit 4: PLL_LOCK = 0; bits 7:5 = Reserved = 000
    rf_setup_byte &= 0x0F;//0000 1111
    SPI_Write_Reg(RF_SETUP, &rf_setup_byte);     // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR

    //Transmiter Address.
    SPI_Write_Buf_Reg(TX_ADDR, ADDR_HOST, TX_RX_ADDR_WIDTH);
    //Receiver Address - Pipe 0
    SPI_Write_Buf_Reg(RX_ADDR_P0, ADDR_HOST, TX_RX_ADDR_WIDTH);
    // Ativa Payload dinamico em data pipe 0
    SPI_Write_Reg(DYNPD, &dypnd_value);        // Ativa Payload dinâmico em data pipe 0
    // Ativa Payload dinamico, com ACK e comando W_TX_PAY
    SPI_Write_Reg(FEATURE, &feature_value);      // Ativa Payload dinâmico, com ACK e comando W_TX_PAY

    //After the packet is validated, Enhanched ShockBurst™ disassembles the packet and loads the payload into
    //the RX FIFO, and assert the RX_DR IRQ (active low)
    //A interrupção é associada ao handler RF_IRQ (nesta classe), no código principal (rf_shield_Host.cpp).

    // Clears the TX and RX FIFO
    SPI_Write(FLUSH_TX, &zero);
    SPI_Write(FLUSH_RX, &zero);

    // Writes in the STATUS register
    SPI_Write_Reg(NRF_STATUS, &nrf_status_value);

    //Default: Stay in RX Mode waiting for data from MIP
    RX_Mode();

}

/**
 * Writes a value in a register
 * @param reg   Register adress
 * @param value Value to be written
 **/
void SPI_Write_Reg(uint8_t reg, uint8_t * value)
{
    uint16_t size = sizeof(*value);
    reg = (uint8_t) W_REGISTER + reg;
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_RESET);    // CSN low, initiate SPI transaction
    HAL_SPI_Transmit(&_spi, &reg, sizeof(reg), HAL_MAX_DELAY);                    // select register
    HAL_SPI_Transmit(&_spi, value, size, HAL_MAX_DELAY);                   // ..and write value to it..
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);      // CSN high again, ends SPI transaction
}

/**
 * Writes a value in a register buffer
 * @param reg   Register adress
 * @param value Value to be written
 * @param size  Buffer size
 **/
void SPI_Write_Buf_Reg(uint8_t reg, uint8_t *value, uint16_t size)
{
    reg = (uint8_t) W_REGISTER + reg;
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_RESET);    // CSN low, initiate SPI transaction
    HAL_SPI_Transmit(&_spi, &reg, sizeof(reg), HAL_MAX_DELAY);                    // select register
    HAL_SPI_Transmit(&_spi, value, size, HAL_MAX_DELAY);                   // ..and write value to it..
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);      // CSN high again, ends SPI transaction
}

/**
 * Writes a buffer value
 * @param reg   Register adress
 * @param value Value to be written
 * @param size  Buffer size
 **/
void SPI_Write_Buf(uint8_t command, uint8_t *value, uint16_t size)
{
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_RESET);    // CSN low, initiate SPI transaction
    HAL_SPI_Transmit(&_spi, &command, sizeof(command), HAL_MAX_DELAY);                    // select register
    HAL_SPI_Transmit(&_spi, value, size, HAL_MAX_DELAY);                   // ..and write value to it..
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);      // CSN high again, ends SPI transaction
}

/**
 * Send a command that need a value to be written
 * @param command   SPI command
 * @param value     Value to be written
 **/
void SPI_Write(uint8_t command, uint8_t * value)
{
    uint16_t size = sizeof(*value);
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_RESET);    // CSN low, initiate SPI transaction
    HAL_SPI_Transmit(&_spi, &command, sizeof(command), HAL_MAX_DELAY);                    // select register
    HAL_SPI_Transmit(&_spi, value, size, HAL_MAX_DELAY);                   // ..and write value to it..
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);      // CSN high again, ends SPI transaction
}

/**
 * Reads a value returned by a command
 * @param command   SPI command
 * @return The byte returned by the command
 **/
uint8_t SPI_Read(uint8_t command)
{
    uint8_t reading = 0;

    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_RESET);    // CSN low, initiate SPI transaction
    HAL_SPI_Transmit(&_spi, &command, sizeof(command), HAL_MAX_DELAY); 	  // Transmits the command
    HAL_SPI_Receive (&_spi, &reading, sizeof(command), HAL_MAX_DELAY);	  // Saves the response in 'reading'
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);      // CSN high again, ends SPI transaction

    return reading;
}

/**
 * Reads the STATUS register
 * @return the STATUS register content
 */

uint8_t SPI_Read_Status()
{
    uint8_t reg_read = 0;
    uint8_t zeros = 0xFF;

    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_RESET);    // CSN low, initiate SPI transaction
    HAL_SPI_TransmitReceive (&_spi, &zeros, &reg_read, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);      // CSN high again, ends SPI transaction

    return reg_read;
}

/**
 * Reads a register e returns its value
 * @param reg   Register adress
 * @return Value read from the register
 **/
uint8_t SPI_Read_Reg(uint8_t reg)
{
    uint8_t reg_read = 0;
    reg = (uint8_t) R_REGISTER + reg;

    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_RESET);    // CSN low, initiate SPI transaction
    HAL_SPI_Transmit(&_spi, &reg, sizeof(reg), HAL_MAX_DELAY); 
    HAL_SPI_Receive (&_spi, &reg_read, sizeof(reg), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);      // CSN high again, ends SPI transaction

    return reg_read;
}

/**
 * Sends a command and reads a buffer of bytes
 * @param command   SPI command
 * @param dataBuf   Buffer to store the data
 **/
void SPI_Read_Buf(uint8_t command, uint8_t *dataBuf, uint16_t size)
{
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_RESET);  // CSN low, initiate SPI transaction
    HAL_SPI_Transmit(&_spi, &command, 1, HAL_MAX_DELAY);           		// select register
    HAL_SPI_Receive (&_spi, dataBuf, size, HAL_MAX_DELAY);              // read register
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);    // CSN high again, ends SPI transaction
}

/**
 * Reads the payload in RX using the R_RX_PAYLOAD command
 * @param buf	Buffer to store the data received
 **/
void SPI_Read_rx_buf(uint8_t *buf)
{
	uint8_t command = R_RX_PAYLOAD;
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_RESET);  // CSN low, initiate SPI transaction
    HAL_SPI_Transmit(&_spi, &command, 1, HAL_MAX_DELAY);           		// select register
    HAL_SPI_Receive (&_spi, buf, PAYLOAD_WIDTH, HAL_MAX_DELAY);         // read register
    HAL_GPIO_WritePin(_RF_CSN_GPIO_Port, _RF_CSN_Pin, GPIO_PIN_SET);    // CSN high again, ends SPI transaction
}

/**
 * Changes the nRF state to RX, which it awaits a payload to be received
 **/
void RX_Mode(void)
{
    //rx_newPayload = 0;
    //status = 0;
    RX_OK = 0;

    uint8_t config_value = 0x0F;

    //The RX mode is an active mode where the nRF24L01 radio is a receiver. To enter this mode, the
    //nRF24L01 must have the PWR_UP bit set high, PRIM_RX bit set high and the CE pin set high.

    //Make sure you sett CE = 0 first, so the chip is in Standby mode before you change radio mode. 
    //CE (active high and is used to activate the chip in RX or TX mode) - 0: Desativa o transceiver para programação
    HAL_GPIO_WritePin(_RF_CE_GPIO_Port, _RF_CE_Pin, GPIO_PIN_RESET); 

    //Configurar transceiver para recepção de dados  
    // CONFIG register (nRF24LE01):
    // b7. Reserved     = 0;
    // b6. MASK_RX_DR   = 0: Reflect RX_DR as active low on RFIRQ (interrupt)
    // b5. MASK_TX_DS   = 0: Reflect TX_DS as active low interrupt on RFIRQ
    // b4. MASK_MAX_RT  = 1: Disabled - Reflect MAX_RT as active low on RFIRQ
    // b3. EN_CRC       = 1: Enable CRC - Forced high if one of the bits in the EN_AA is high
    // b2. CRCO         = 1: CRC encoding 2 bytes
    // b1. PWR_UP       = 1: POWER UP
    // b0. PRIM_RX      = 1: RX/TX control with RX (sets the nRF24L01 in transmit/receive)
    SPI_Write_Reg(CONFIG, &config_value);

    //CE (active high and is used to activate the chip in RX or TX mode) - a: Ativa o transceiver para RX
    HAL_GPIO_WritePin(_RF_CE_GPIO_Port, _RF_CE_Pin, GPIO_PIN_SET); 
  
}


/**
 * Function called when an IRQ occurs. After verifying the nRF state it saves the paylod (RX mode) or 
 *  flushes the TX FIFO after a successful transmission
 **/
void RF_IRQ(uint8_t *buf, uint8_t *size, uint8_t *newPayload)
{
    RX_Mode();
	//HAL_Delay(1); // Delay to give NRf time to transmit the ACK packet
    //HAL_GPIO_WritePin(_RF_CE_GPIO_Port, _RF_CE_Pin, GPIO_PIN_RESET);
    // Read STATUS register
    status = SPI_Read_Status();

    //HAL_GPIO_TogglePin(_RF_LED_GPIO_Port, _RF_LED_Pin);

    if(status & RX_DR)
    { 
        // if received data ready (RX_DR) interrupt
        RX_OK = 1;
        *size = SPI_Read(R_RX_PLD_WIDTH);  // Retorna o número de bytes no payload recebido
        SPI_Read_Buf(R_RX_PAYLOAD, buf, *size);  // read receive payload from RX_FIFO buffer

        if(*size > 32)  //Não pode conter mais que 32 bytes
        {
            *size = 0;
        }
        SPI_Write(FLUSH_RX, 0x00); //Limpar o buffer RX (os dados recebidos estão em rx_buf).

        *newPayload = 1;
    }

    

    uint8_t sta_val = 0x70;
    SPI_Write_Reg(NRF_STATUS, &sta_val);

    //se o pacote foi reconhecido pelo receptor (funciona com TX-ACK)
    if(status & TX_DS)
    {
        //Completou TX ?
        TX_OK = 1;
        //RX_Mode();
        SPI_Write(FLUSH_TX,0); //limpar o buffer TX
        HAL_GPIO_TogglePin(LED_Port, LED_VERDE);

    }
    
    // If the maximum number of retransmissions was reached
    if(status & MAX_RT)
    {
    	//
    	SPI_Write(FLUSH_TX,0); //limpar o buffer TX
    	HAL_GPIO_TogglePin(LED_Port, LED_VERMELHO);

    }



    //Reset status
    
    //RX_Mode();


}


/**
 * Transmits a payload and returns to rx_mode
 * @param buf The payload to be transmitted
 * @param payloadLength The payload's length
 */

void TX_Mode(uint8_t* buf, uint8_t payloadLength, uint8_t autoAck_enabled)
{
	  TX_OK = 0; //Iniciando transmissão (Na IRQ é setada para 1, indicando fim de transmissão)

	  //The TX mode is an active mode where the nRF24L01 transmits a packet.
	  //To enter this mode, the nRF24L01 must have the PWR_UP bit set high, PRIM_RX bit set low,
	  //a payload in the TX FIFO and, a high pulse on the CE for more than 10μs.

	  //Make sure you sett CE = 0 first, so the chip is in Standby mode before you change radio mode.
	  //CE (active high and is used to activate the chip in RX or TX mode) - 0: Desativa o transceiver para programação
	  HAL_GPIO_WritePin(_RF_CE_GPIO_Port, _RF_CE_Pin, GPIO_PIN_RESET);


	  //Configurar transceiver para transmissão de dados
	  // CONFIG register (nRF24LE01):
	  // b7. Reserved 		= 0;
	  // b6. MASK_RX_DR 	= 0: Reflect RX_DR as active low on RFIRQ (interrupt)
	  // b5. MASK_TX_DS 	= 0: Reflect TX_DS as active low interrupt on RFIRQ
	  // b4. MASK_MAX_RT 	= 0 or 1: 0 Reflect MAX_RT as active low on RFIRQ or 1 to disable this IRQ
	  // b3. EN_CRC 		= 1: Enable CRC - Forced high if one of the bits in the EN_AA is high
	  // b2. CRCO 			= 1: CRC encoding 2 bytes
	  // b1. PWR_UP 		= 1: POWER UP
	  // b0. PRIM_RX 		= 0: RX/TX control with TX (sets the nRF24L01 in transmit/receive)
	  uint8_t config = 0x0E; // 0000 1110

	  // Disables the MAX_RT IRQ if the Auto Acknowledgment is disabled
	  if(autoAck_enabled == 0)
	  {
		  config |= 0x10; // 0000 1110 | 0001 0000 = 0001 1110
	  }

	  SPI_Write_Reg(CONFIG, &config);

	  HAL_Delay(2);

	  //enviar (transmitir) endereço do receptor para o qual a mensagem será enviada (o outro nRF24L01)
	  SPI_Write_Buf_Reg(RX_ADDR_P0, ADDR_HOST, TX_RX_ADDR_WIDTH);

	  //Envia o payload para o transceiver.
	  if(autoAck_enabled)
		  SPI_Write_Buf(W_TX_PAYLOAD, buf, payloadLength); // Writes data to TX payload
	  else
		  SPI_Write_Buf(W_TX_PAYLOAD_NOACK, buf, payloadLength); // Writes data to TX payload without Acknowledgment

	  HAL_Delay(1);

	  //Iniciar transmissão - ativar TX-RF
	  // Set CE pin high to enable TX Mode
	  //	CE (active high and is used to activate the chip in RX or TX mode)
	  // 	- a: Ativa o transceiver para RX
	  HAL_GPIO_WritePin(_RF_CE_GPIO_Port, _RF_CE_Pin, GPIO_PIN_SET);

	  //Aguardar IRQ indicando que concluiu a transmissão..

	  //HAL_Delay(1); //delay suficiente para transmitir o payload maximo de 32 bytes.
	            //Com data rate de 1Mbps ==> 1us por bit;
	            //Pacote transmitido: Preambulo (1 byte) + endereço (5bytes) + controle (9bits) + payload (até 32 bytes)
	            //                    + CRC (2 bytes) ==> Total 329 bits (pacote maximo) ==> ou seja 329useg
	            //    adicionando os tempos de wakeup etc, teríamos +- 1mseg... vou usar 2mseg por segurança aqui...

	  //RX_Mode();

	  /*
	   * COLOCAR UM WHILE PRA MUDAR PRA RX MODE
	   *
	   * INDENTIFICAR SE QUAL INTERRUPÇÃO QUE TÁ SENDO ACIONADA (TX OU MAX_RT) USANDO LED
	   */
}
