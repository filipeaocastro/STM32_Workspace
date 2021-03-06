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
//uint8_t static nRFint_guard = 0; //não executar ainda interrupções que porventura cheguem em nRF_pinIRQ
uint8_t rf_tx_buffer[NUM_CHARS] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void rx_task(void);
uint8_t convert2ascii(uint8_t num);
void printAscii(uint8_t byte);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_buf[PAYLOAD_WIDTH];
uint8_t rx_payloadWidth = 0;
uint8_t rx_newPayload = 0;
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

  nRF24L01_STM32(hspi1); // Set the SPI parameters for the nRF library

  // ****** debug ****** INICIO
  HAL_Delay(5000);

  uint8_t bbb[] = {"Regitrador STATUS antes do Init:\n"};
  CDC_Transmit_FS(bbb, sizeof(bbb));
  HAL_Delay(10);

  uint8_t sta = SPI_read2(NRF_STATUS);
  uint8_t setup_aw = SPI_read2(SETUP_AW);
  uint8_t en_aa = SPI_read2(EN_AA);
  uint8_t rf_ch = SPI_read2(RF_CH);

  printAscii(sta);
  printAscii(setup_aw);
  printAscii(en_aa);
  printAscii(rf_ch);

  uint8_t sep[] = {"-------------\n"};
  CDC_Transmit_FS(sep, sizeof(sep));

  // ****** debug ****** FIM

  // Initiate the nRF with the channel, data rate and tx power parameters
  uint8_t status = init(nRF_Canal, RF_DATA_RATE_1Mbps, RF_TX_POWER_0dBm);

  // ****** debug ******

  uint8_t ddd[] = {"Regitrador STATUS depois do Init:\n"};
  CDC_Transmit_FS(ddd, sizeof(ddd));
  HAL_Delay(10);

  uint8_t _setup_aw = SPI_read2(SETUP_AW);
  uint8_t _en_aa = SPI_read2(EN_AA);
  uint8_t _rf_ch = SPI_read2(RF_CH);

  printAscii(status);
  printAscii(_setup_aw);
  printAscii(_en_aa);
  printAscii(_rf_ch);

  // ****** debug ****** FIM

  nRFint_guard = 1; //liberar execução da interrupção externa

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  if(HAL_GPIO_ReadPin(RF_IRQ_GPIO_Port, RF_IRQ_Pin) == 0)
	  {
		  uint8_t fff[] = {"Chegou um trem\n"};
		  CDC_Transmit_FS(fff, sizeof(fff));
		  RF_IRQ();
	  }*/
	  rx_task();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
void rx_task()
{
    //Verificar se chegou (recebeu) um novo pacote pelo canal RF.
    //(O MIP enviou um pacote para o HOST).
    if (rx_newPayload > 0)  //newPayload setada em IRQ de chegada de novo pacote (RX)
    {
        rx_newPayload = 0; //sdinalizar payload recebida
        uint8_t rrr[] = {"Entrou no rx task\n"};
        CDC_Transmit_FS(rrr, sizeof(rrr));

        if(rx_payloadWidth > 0) //Se a interrupção foi gerado por algum ruído etc, não teremos dados no payload
        {
          //Enviar pacote recebido para o código do HOST (Visual Studio) via serial COMM (USB)
          //Serial.write(host_nrf.rx_buf, host_nrf.rx_payloadWidth);

        	HAL_Delay(10);
        	CDC_Transmit_FS(rx_buf, rx_payloadWidth);
        	HAL_Delay(10);
        	uint8_t nl = '\n';
        	CDC_Transmit_FS(&nl, 1);
        	HAL_Delay(10);
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == RF_IRQ_Pin)
  {
	  //rx_newPayload = 1;
	  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  //uint8_t fff[] = {"Chegou um trem\n"};
	  //CDC_Transmit_FS(fff, sizeof(fff));
	  //HAL_Delay(10);
	  //printAscii(SPI_read2(NRF_STATUS));
	  //uint8_t sta_val = 0x70;
	  //SPI_Write_Reg(NRF_STATUS, &sta_val);
	  //uint8_t tamanho = SPI_Read(R_RX_PLD_WIDTH);
	  //SPI_Read_Buf(R_RX_PAYLOAD, &rxx_buf, tamanho);
	  //CDC_Transmit_FS(rxx_buf, sizeof(rxx_buf));
	  //uint8_t fifo_sta = SPI_read2(FIFO_STATUS);
	  //printAscii(fifo_sta);

	  if(nRFint_guard > 0)
		  RF_IRQ(rx_buf, &rx_payloadWidth, &rx_newPayload);



  }
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
