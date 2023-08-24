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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bootloader_command_app.h"
#include "stdarg.h" // va_lis printf bu kütüphanede bulunuyor.
#include "stdio.h"
#include "string.h"
#include "bootloader_command_code.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_SECTOR2_BASE_ADDRESS  0x08008000
#define FLASH_SECTOR2_BASE_ADDRESS2  0x08040000
#define BL_RX_DATA_LENGTH 266

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t bootloader_rx_data[BL_RX_DATA_LENGTH]; //Pc den gelen komutlar burada toplanacak

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void printMessage(char *format, ...)
{
	char comingMessage[100];

	va_list vaList;
	va_start(vaList, format);
	vsprintf(comingMessage, format, vaList);
	HAL_UART_Transmit(&huart1, (uint8_t*)comingMessage, strlen(comingMessage), HAL_MAX_DELAY);
	va_end(vaList);
}
void bootloader_uart_data_read(void)
{
	uint8_t bl_rx_length = 0; // gelen data uzunluğu alınması için

	while(1)
	{
		memset(bootloader_rx_data, 0, BL_RX_DATA_LENGTH);

		// Pc'den ilk gelen byte length to follow old için size of 1 yazıldı.
		HAL_UART_Receive(&huart3, bootloader_rx_data, 1, HAL_MAX_DELAY);

		bl_rx_length = bootloader_rx_data[0];// gelecek verinin uzunluğu öğrenildi.

		// Pc'den gelen byte da length to follow değeri çıkarıldı. Command code ögrenildi. ...data[1] oldu.
		HAL_UART_Receive(&huart3, &bootloader_rx_data[1], bl_rx_length, HAL_MAX_DELAY);

		switch(bootloader_rx_data[1]) //command koda göre case gir.
		{
			case BL_GET_VER:
				bootloader_get_ver_cmd(bootloader_rx_data);
			break;
			case BL_GET_HELP:
				bootloader_get_help_cmd(bootloader_rx_data);
			case BL_GO_TO_ADDR:
				bootloader_go_to_addr_cmd(bootloader_rx_data);
			case BL_FLASH_ERASE:
				bootloader_flash_erase_cmd(bootloader_rx_data);
			break;
			case BL_MEM_WRITE:
				bootloader_mem_write_cmd(bootloader_rx_data);
				break;
			default:
				break;
		}
	}

}
void bootloader_jump_to_user_application(void)
{
	// 1- Sifirlama isleyicisinin adresini tutan bir fonksiyon göstericisi
		void (*bootloader_application_reset_handler)(void);
		// 2- Kullanici uygulamasina atlama kodu çagrildi
		printMessage("Bootloader : Called bootloader_jump_to_user_application() \n");
		// 3- MSP'nin degerini tut
		uint32_t mspValue = *(volatile uint32_t*) FLASH_SECTOR2_BASE_ADDRESS;
		printMessage("Bootloader : MSP Value: %#x \n", mspValue);
		//__set_MSP(mspValue);
		// 4- Sifirlama isleyicisinin degerini tut, MSP + 4 = reset handler
		uint32_t resetValue = *(volatile uint32_t*) (FLASH_SECTOR2_BASE_ADDRESS + 4);
		printMessage("Bootloader : Reset Value: %#x \n", resetValue);
		// 5- Sifirlama isleyicisi ile bir islev baslat
		bootloader_application_reset_handler = (void*) resetValue;
		// 6- Sifirlama isleyicisini çagir ve kullanici uygulamasina atla
		bootloader_application_reset_handler();

	}

void bootloader_jump_to_user_application2(void)
{
	// 1- Sifirlama isleyicisinin adresini tutan bir fonksiyon göstericisi
		void (*bootloader_application_reset_handler)(void);
		// 2- Kullanici uygulamasina atlama kodu çagrildi
		printMessage("Bootloader : Called bootloader_jump_to_user_application() \n");
		// 3- MSP'nin degerini tut
		uint32_t mspValue = *(volatile uint32_t*) FLASH_SECTOR2_BASE_ADDRESS2;
		printMessage("Bootloader : MSP Value: %#x \n", mspValue);
		//__set_MSP(mspValue);
		// 4- Sifirlama isleyicisinin degerini tut, MSP + 4 = reset handler
		uint32_t resetValue = *(volatile uint32_t*) (FLASH_SECTOR2_BASE_ADDRESS2 + 4);
		printMessage("Bootloader : Reset Value: %#x \n", resetValue);
		// 5- Sifirlama isleyicisi ile bir islev baslat
		bootloader_application_reset_handler = (void*) resetValue;
		// 6- Sifirlama isleyicisini çagir ve kullanici uygulamasina atla
		bootloader_application_reset_handler();

	}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14,1);


	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
	  {
		  printMessage("Bootloader : Button is pressed and going to bootloader mode.\n");

		  bootloader_uart_data_read();
	  }
	  else
	  {
		  printMessage("Bootloader : Button is not pressed and executing user application.\n");
		  HAL_Delay(3000);
		  bootloader_jump_to_user_application();
		  bootloader_jump_to_user_application2();

	  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
static void goto_application(void ){
	printf("Gonna jump to Application 1\n");

	void (*app_reset_handler)(void) = (void*) ( *((volatile uint32_t *) (0x08040000 + 4U)));

	//__set_MSP( *(volatile uint32_t *) 0x08040000 );

	HAL_GPIO_WritePin (GPIOG, GPIO_PIN_13, 0);
	app_reset_handler();
}*/

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

