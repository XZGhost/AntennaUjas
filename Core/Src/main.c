/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "NRF905.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 50);
	for (int i = 0; i < len; i++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
NRF905_hw_t NRF905_hw;
NRF905_t NRF905;

int master;
int ret;

char buffer[64];
char nrf905_payload_buffer[NRF905_MAX_PAYLOAD + 1];

int message;
int message_length;

int number = 1234;
char str [10];


uint32_t my_address;
uint32_t receiver_address;
uint32_t ans_data;

#define ADDRESS_MASTER 0x25D34478
#define ADDRESS_SLAVE  0x6DA0C59B
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
  MX_I2C1_Init();
  MX_USB_PCD_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	uint32_t uid = 0x00;
	for (uint8_t i = 0; i < 3; ++i) {
		uid += *(uint32_t*) (UID_BASE + i * 4);
	}
	srand(uid);

	NRF905_hw.gpio[NRF905_HW_GPIO_TXEN].pin = GPIO_PIN_15;
	NRF905_hw.gpio[NRF905_HW_GPIO_TXEN].port = GPIOB;
	NRF905_hw.gpio[NRF905_HW_GPIO_TRX_EN].pin = GPIO_PIN_13;
	NRF905_hw.gpio[NRF905_HW_GPIO_TRX_EN].port = GPIOB;
	NRF905_hw.gpio[NRF905_HW_GPIO_PWR].pin = GPIO_PIN_14;
	NRF905_hw.gpio[NRF905_HW_GPIO_PWR].port = GPIOB;

	NRF905_hw.gpio[NRF905_HW_GPIO_CD].pin = GPIO_PIN_12;
	NRF905_hw.gpio[NRF905_HW_GPIO_CD].port = GPIOB;
	NRF905_hw.gpio[NRF905_HW_GPIO_AM].pin = 0;
	NRF905_hw.gpio[NRF905_HW_GPIO_AM].port = NULL;
	NRF905_hw.gpio[NRF905_HW_GPIO_DR].pin = 0;
	NRF905_hw.gpio[NRF905_HW_GPIO_DR].port = NULL;

	NRF905_hw.gpio[NRF905_HW_GPIO_CS].pin = GPIO_PIN_3;
	NRF905_hw.gpio[NRF905_HW_GPIO_CS].port = GPIOE;

	NRF905_hw.tim = &htim1;
	NRF905_hw.spi = &hspi3;


    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);


	master = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6);


//	if (master == 1) {
//		my_address = ADDRESS_MASTER;
//		receiver_address = ADDRESS_SLAVE;
//	} else {
//		my_address = ADDRESS_SLAVE;
//		receiver_address = ADDRESS_MASTER;
//	}
//
//	if (master == 1) {
//		printf("Mode: Master, TX, %08lX\r\n", my_address);
//		HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_RESET);
//	} else {
//		printf("Mode: Slave, RX, %08lX\r\n", my_address);
//
//		HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_SET);
//	}
//
//	NRF905_init(&NRF905, &NRF905_hw);
//	NRF905_set_listen_address(&NRF905, receiver_address);
//
//	printf("Reg conf: ");
//	for (int i = 0; i < 10; ++i) {
//		uint8_t d = NRF905_write_config_register(&NRF905, i, i);
////		uint8_t d = NRF905_read_config_register(&NRF905, i);
//		printf("%02X, ", d);
//	}
//	printf("\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	int c = 0;


	  while (1)
	  {
			if (master >= 1) {
				my_address = ADDRESS_MASTER;
				receiver_address = ADDRESS_SLAVE;
			} else {
				my_address = ADDRESS_SLAVE;
				receiver_address = ADDRESS_MASTER;
			}

			if (master >= 1) {
				printf("1234\r\n");
	//	  		printf("Mode: Master, TX, %08lX\r\n", my_address);
				HAL_GPIO_WritePin(GPIOE, LD9_Pin, GPIO_PIN_SET);
			} else {
	//	  		printf("Mode: Slave, RX, %08lX\r\n", my_address);
				printf("1234\r\n");
				HAL_GPIO_WritePin(GPIOE, LD8_Pin, GPIO_PIN_SET);
				//HAL_SPI_Receive_DMA(hspi, pData, Size)
				//HAL_SPI_Receive_IT(&hspi3, address, 1);

			}

		    snprintf(str, sizeof str, "%d", number);
		    printf("Result: %s\n", str);
		    return 0;

			for(j=str-1; j>=0; j--){ printf("%c", size(number[j]));

			}

			NRF905_init(&NRF905, &NRF905_hw);
			NRF905_set_listen_address(&NRF905, receiver_address);

	//	  	printf("Reg conf: ");
	//	  	for (int i = 0; i < 10; ++i) {
	//	  		uint8_t d = NRF905_write_config_register(&NRF905, i, i);
	//	  //		uint8_t d = NRF905_read_config_register(&NRF905, i);
	//	  		printf("%02X, ", d);
	//	  	}
			printf("\r\n");
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
