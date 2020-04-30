/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

//#include "stm32f1xx_hal.h"

#include "DataTypes.h"
#include "RF24.h"
#include "GPIOStm32f103.h"
#include "STM32Syscalls.h"
#include "SPIStm32f103.h"
#include "STM32Syscalls.h"
#include "Debug.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */


gpio::GPIOInterface *mesh_gpio;
syscalls::SyscallsInterface *mesh_syscalls;
spi::SPIInterface *mesh_spi;
network::RF24 *radio;
debugger::Debug *debug;

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 0;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
/**********************************************************/

uint8_t addresses[][6] = {"1Node","2Node"};

// Used to control whether this node is sending or receiving
bool role = radioNumber;


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
	MX_TIM1_Init();
	MX_USART2_UART_Init();





//	test::log_debug((char*)"s", "s");
//	log_debug("sss", "sss", 1);
//	log_debug((char*)"%s", "TEST");
//
//	int rsp_time = 0;
//	if(rsp_time){
//		rsp_time = 2;
//	}
//
	struct gpio::gpio_pins pins;
	pins.ce_port = RF24_CE_GPIO_Port;
	pins.ce_pin = RF24_CE_Pin;
	pins.csn_port = RF24_CSN_GPIO_Port;
	pins.csn_pin = RF24_CSN_Pin;

	mesh_spi = new spi::SPIStm32f103(&hspi1);
	debug = new debugger::Debug(&huart2);
//	//
//	mesh_gpio = new gpio::GPIOStm32f103();
//	mesh_gpio->init_pins(&pins);
//	//
//	mesh_syscalls = new syscalls::STM32Syscalls(&htim1);
//	mesh_syscalls->set_cpu_speed(syscalls::SPEED_72MHZ);
//	mesh_syscalls->init();
//
//	radio = new network::RF24(mesh_gpio, mesh_syscalls, mesh_spi);
//
//	  radio->begin();
//
//	  // Set the PA Level low to prevent power supply related issues since this is a
//	 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
//	  radio->setPALevel(network::RF24_PA_LOW);
//
//	  // Open a writing and reading pipe on each radio, with opposite addresses
//	  if(radioNumber){
//	    radio->openWritingPipe(addresses[1]);
//	    radio->openReadingPipe(1,addresses[0]);
//	  }else{
//	    radio->openWritingPipe(addresses[0]);
//	    radio->openReadingPipe(1,addresses[1]);
//	  }
//
//	  // Start the radio listening for data
//	  radio->startListening();
//
//
//	  while(!radio->isChipConnected())
//	  {
//		  HAL_Delay(1);
//	  }

	  uint8_t test[] = "UART LIRAR!!!!\n\r";
//	  char uart_msg[] = "THIS IS WORKING\n\r";
	while (1)
	{
//		HAL_StatusTypeDef HAL_USART_Transmit(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size, uint32_t Timeout)







		debug->debug("%s", "FIRST DEBUG OUTPUT");
		debug->warn("%s", "FIRST DEBUG OUTPUT");
		debug->info("%s", "FIRST DEBUG OUTPUT");
		HAL_Delay(100);
		HAL_GPIO_TogglePin(BLINKY_LED_GPIO_Port, BLINKY_LED_Pin);





		//		if(status)
//		HAL_UART_Transmit(&huart2, (uint8_t*)test, string_len(test), 100);


//
//
//		/****************** Ping Out Role ***************************/
//		if (role == 1)  {
//
//			radio->stopListening();                                    // First, stop listening so we can talk.
//
//
////			Serial.println(F("Now sending"));
//
//			unsigned long start_time = 123;                             // Take the time, and send it.  This will block until complete
//			if (!radio->write( &start_time, sizeof(unsigned long) )){
////				Serial.println(F("failed"));
//			}
//
//			radio->startListening();                                    // Now, continue listening
//
//			unsigned long started_waiting_at = 0;               // Set up a timeout period, get the current microseconds
//			bool timeout = false;                                   // Set up a variable to indicate if a response was received or not
//
//			while ( ! radio->available() ){                             // While nothing is received
//				if (started_waiting_at > 2000 ){            // If waited longer than 200ms, indicate timeout and exit while loop
//					timeout = true;
//					break;
//				}
//				started_waiting_at++;
//				HAL_Delay(1);
//			}
//
//			if ( timeout ){                                             // Describe the results
////				Serial.println(F("Failed, response timed out."));
//			}else{
//				unsigned long got_time;                                 // Grab the response, compare, and send to debugging spew
//				radio->read( &got_time, sizeof(unsigned long) );
//				unsigned long end_time = 456;
//
//				// Spew it
////				Serial.print(F("Sent "));
////				Serial.print(start_time);
////				Serial.print(F(", Got response "));
////				Serial.print(got_time);
////				Serial.print(F(", Round-trip delay "));
////				Serial.print(end_time-start_time);
////				Serial.println(F(" microseconds"));
//			}
//
//			// Try again 1s later
//			HAL_Delay(1000);
//		}
//
//
//
//		/****************** Pong Back Role ***************************/
//
//		if ( role == 0 )
//		{
//			unsigned long got_time;
//
//			if( radio->available()){
//				// Variable for the received timestamp
//				while (radio->available()) {                                   // While there is data ready
//					radio->read( &got_time, sizeof(unsigned long) );             // Get the payload
//					HAL_Delay(1);
//				}
//
//				radio->stopListening();                                        // First, stop listening so we can talk
//				radio->write( &got_time, sizeof(unsigned long) );              // Send the final one back.
//				radio->startListening();                                       // Now, resume listening so we catch the next packets.
////				Serial.print(F("Sent response "));
////				Serial.println(got_time);
//			}
//		}




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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 65535;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 18000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(BLINKY_LED_GPIO_Port, BLINKY_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF24_CE_Pin|RF24_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLINKY_LED_Pin */
  GPIO_InitStruct.Pin = BLINKY_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLINKY_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF24_IRQ_Pin */
  GPIO_InitStruct.Pin = RF24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RF24_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF24_CE_Pin RF24_CSN_Pin */
  GPIO_InitStruct.Pin = RF24_CE_Pin|RF24_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
