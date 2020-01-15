/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define nRF_Canal 92
#define NUM_CHARS 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void rx_task(void);
uint8_t convert2ascii(uint8_t num);
void printAscii(uint8_t byte);
void get_Msg_fromHost(/*uint8_t* buf, uint16_t len*/);
void rfSendBuffer(uint8_t *buffer2send, uint8_t buffer_size);
void tx_task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buf[PAYLOAD_WIDTH];
uint8_t rx_payloadWidth = 0;
uint8_t rx_newPayload = 0;
const char endMsgChar = '\0'; //Caracter finalizador de mensagens
uint8_t rf_tx_buffer[NUM_CHARS] = {0};
int rf_tx_buffer_count = 0;
uint8_t rf_tx_SendMsg = 0;
uint8_t rfBridgeON = 0;
uint8_t rx_newData = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  nRFint_guard = 0;		// Do not execute interruptions until the nRF initalization is complete
  rf_tx_buffer_count = 0;
  rf_tx_SendMsg = 0;
  rfBridgeON = 0;   // Don't transfer data via RF until the Handshake HOST <-> STM is complete

  nRF24L01_STM32(hspi1); // Set the SPI parameters for the nRF library

  // Initiate the nRF with the channel, data rate and tx power parameters
  init(nRF_Canal, RF_DATA_RATE_1Mbps, RF_TX_POWER_0dBm);
  rx_newPayload = 0;
  nRFint_guard = 1; //liberar execução da interrupção externa
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(rx_newData > 0)	// rx_newData é alterado na função callback quando um novo dado chega pela serial (CDC_Receive_FS)
		  get_Msg_fromHost();

	  tx_task();	//Se existir mensagem (corretamente lida em get_Msg_fromHost()), a envia para o MIP (via RF)

	  rx_task();  	//Verifica se chegou algum pacote do MIP (via RF).
	              	  //Se existir mensagem (corretamente lida em rx_task()), a envia para oHOST via porta COM.

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_CE_Pin|RF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_CE_Pin RF_CSN_Pin */
  GPIO_InitStruct.Pin = RF_CE_Pin|RF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_IRQ_Pin */
  GPIO_InitStruct.Pin = RF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 * Função de callback chamada toda vez que ocorre ocorre uma interrupção externa (FALLING) no pino IRQ do nRF.
 * Caso o nRFint_guard permita, ela salva o conteúdo recebido pelo RF, salva no rx_buf e ativa a flag rx_newPayload.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == RF_IRQ_Pin)
  {
	  // As interrupções já podem ser tratadas?
	  if(nRFint_guard > 0)
		  // Salva o conteúdo em rx_buf, a qte de bytes em rx_payloadWidth e ativa a flag rx_newPayload.
		  RF_IRQ(rx_buf, &rx_payloadWidth, &rx_newPayload);
  }
}
/**
 * Verifica se algum pacote foi recebido pela interrupção e envia ao Host.
 */
void rx_task()
{
    //Verificar se chegou (recebeu) um novo pacote pelo canal RF.
    //(O MIP enviou um pacote para o HOST).
    if (rx_newPayload > 0)  //newPayload setada em IRQ de chegada de novo pacote (RX)
    {
        rx_newPayload = 0; //sdinalizar payload recebida

        if(rx_payloadWidth > 0) //Se a interrupção foi gerado por algum ruído etc, não teremos dados no payload
        {
          //Enviar pacote recebido para o código do HOST (Visual Studio) via serial COMM (USB)

        	CDC_Transmit_FS(rx_buf, rx_payloadWidth);
        	HAL_Delay(5);

        }
    }
}

uint8_t convert2ascii(uint8_t num)
{
	if(num <= 0x09)
	{
		num = num + 0x30;
	}
	else
	{
		num = num + 0x37;
	}
	return num;
}
void printAscii(uint8_t byte)
{
	uint8_t sta[3];
  sta[0] = ((byte & 0xF0) >> 4); //MSB
  sta[0] = convert2ascii(sta[0]);
  sta[1] = (byte & 0x0F);
  sta[1] = convert2ascii(sta[1]);//LSB
  sta[2] = '\n';

  CDC_Transmit_FS(sta, 3);
  HAL_Delay(10);
}

/**
 * Esssa função verifica se o STM já fez o handshake com o Host.
 * Caso ele já tenha ocorrido, ela ativa a flag (rf_tx_SendMsg) que permite o redirecionamento das
 * mensagens que chegam ao RF para o Host. Caso o handshake não tenha ocorrido, ele é feito aqui.
 * Lê uma mensagem do HOST (terminada com '\0') via port COM e coloca no Buffer para transmissão RF para o MIP.
 *
 * Essa função é chamada toda vez que um dado chega na porta serial através da função de callback do STM
 *
 * @param buf Buffer de dados que chegou na porta serial
 * @param len Quantidade de bytes que chegaram
 */
void get_Msg_fromHost(/*uint8_t* buf, uint16_t len*/)
{
    int i, rc;

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    rx_newData = 0;


    //Se não estiver enviando mensagem do Buffer TX para o HOST:
    if (rf_tx_SendMsg == 0)
    {

    	//memcpy(rf_tx_buffer, buf, len); // Salva os dados do vetor buf em rf_tx_buffer
    	//rf_tx_buffer_count = len;		// Salva a qte de bytes de len em rf_tx_buffer_count

    	// Pega o último byte de rf_tx_buffer e salva em rc
        rc = rf_tx_buffer[rf_tx_buffer_count - 1];

        //O último caractere (rc) é o indicador de final de mensagem ('\0')?
        if (rc == endMsgChar)
        {

            //A mensagem termina com '\0' -- terminador de mensagens enviadas pelo HOST;

            //Estamos em fase de handshake initial Host <-> STM?
            if(rfBridgeON)
            {
            	//CDC_Transmit_FS(rf_tx_buffer, rf_tx_buffer_count);
				//HAL_Delay(10);
              //NÃO - Arduino é apenas um ponte entre HOST e Transceiver RF.
              //Portanto, o que chega ao uC deve ser retransmitido ao MIP via RF.
              // ==> Sinalizar execução do estado para transmissão desta msg para o MIP via RF.
              rf_tx_SendMsg = 1;
              return;
            }
            else
            {
            	// HANDSHAKE  HOST <-> STM
                //Esta é a primeira mensagem recebido (contém terminador '\0').
                //
                //1 - Todos os dados que chegarem pela COM para o STM antes da chegada de MIPCOM_READY ("RDY") serão ignorados.
                //      2 - Ao receber o primeiro MIPCOM_READY ("RDY") o STM retorna pela COM para o HOST "RDYOK" - O Host irá
                //          ler ou tentar ler esta mensagem (que pode estar corrompida ou nem mesmo chegar ao Host (C#).
                //
                //Neste primeiro momento, a mensagem em rf_tx_buffer pode conter apenas lixo, conter a mensagem com lixo antes de 'RDY'
                //ou conter apenas "RDY".
                if(rf_tx_buffer_count > 4) //Se conter mais que 4 elementos ['R','D','Y','\0'] remover elementos excedentes no inicio.
                {
                    for(i = 0; i < 4; i++)
                      rf_tx_buffer[i] = rf_tx_buffer[(rf_tx_buffer_count-4) + i];
                    rf_tx_buffer_count = 4;
                }
                //A mensagem é "RDY\0" ?
                if(rf_tx_buffer[0] == 'R' && rf_tx_buffer[1] == 'D' && rf_tx_buffer[2] == 'Y')
                {
                  //Ecoar para o Host

                	CDC_Transmit_FS(rf_tx_buffer, rf_tx_buffer_count);

                  rfBridgeON = 1; //De agora em diante, todos os bytes recebidos do Host serão enviados ao MIP por RF.
                }
                //Caso a mensagem tenha sido enviada para o Host (acima) ou não (deve ser ignorada):
                rf_tx_buffer_count = 0; //reiniciar leitura de novas mensagens;
                rf_tx_SendMsg = 0; //a mensagem recebida não deve ser enviada por RF
            }
        }
    }
}

void tx_task()
{
  uint8_t data_size, index_atual;

  //Transmissão/Recepção de dados via RF liberada? E
  //Existe mensagem para ser enviada para o MIP via RF?
  if ((rfBridgeON == 0) || (rf_tx_SendMsg == 0))
    return;

  data_size = rf_tx_buffer_count;
  rf_tx_buffer_count = 0;

  index_atual = 0;
  while (index_atual < data_size) // Verifica se todos os dados contidos já foram enviados
  {
    //Enquanto tiver algum para escrever
	// Caso a mensagem possua menos, de 32 bytes ele envia apenas os bytes necessários
    if ((data_size - index_atual) <= 32)
    {
      //Se existem menos de 32 bytes para serem enviados
      rfSendBuffer(&rf_tx_buffer[index_atual], (data_size - index_atual));
      HAL_Delay(1); //Aguardar transmissão -- max 32 bytes
      index_atual = data_size;
    }
    else
    {
      //Se existem pelo menos 32 bytes para serem escritos, escreve um pacote
      rfSendBuffer(&rf_tx_buffer[index_atual], 32);
      HAL_Delay(1); //Aguardar transmissão -- max 32 bytes
      index_atual += 32;
    }
  }

  //Sinalizar mensagem transmitida
  rf_tx_SendMsg = 0;

}
//Transmitir um pacote de até 32 bytes pela RF para o MIP
void rfSendBuffer(uint8_t *buffer2send, uint8_t buffer_size)
{
  uint8_t send_index = 0;
  // Escreve no buffer de saída (tx_buf) os bytes a serem enviados
  for (int i = 0; i < buffer_size; i++)
  {
    tx_buf[i] = buffer2send[i];
    send_index += 1;
  }
  //Enviar via RF
  TX_Mode_NOACK(tx_buf, send_index);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/