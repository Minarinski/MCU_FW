/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_CGRAM_BASE_ADDR	0x40
#define CMD_LCD_ON_CURSOR 0x0E
#define CMD_LCD_ON 0x0C
#define CMD_LCD_CLEAR 0x01
#define CMD_LCD_CURSOR_LINE_1 0x80
#define CMD_LCD_CURSOR_LINE_2 0xC0
#define CMD_LCD_CURSOR_LEFT 0x10
#define CMD_LCD_CURSOR_RIGHT 0x14
#define LCD_ADDR (0x27 << 1)
#define LCD_PIN_RS    (1 << 0)
#define LCD_PIN_EN    (1 << 2)
#define LCD_BACKLIGHT (1 << 3)
#define LCD_DELAY_MS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags) {
    HAL_StatusTypeDef res;
    for(;;) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1, HAL_MAX_DELAY);
        if(res == HAL_OK)
            break;
    }

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|LCD_BACKLIGHT|LCD_PIN_EN;
    data_arr[1] = up|flags|LCD_BACKLIGHT;
    data_arr[2] = lo|flags|LCD_BACKLIGHT|LCD_PIN_EN;
    data_arr[3] = lo|flags|LCD_BACKLIGHT;

    res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
    HAL_Delay(LCD_DELAY_MS);
    return res;
}

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
    LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
    LCD_SendInternal(lcd_addr, data, LCD_PIN_RS);
}

void LCD_Init(uint8_t lcd_addr) {
    // 4-bit mode, 2 lines, 5x7 format
    LCD_SendCommand(lcd_addr, 0x30);
    // display & cursor home (keep this!)
    LCD_SendCommand(lcd_addr, 0x02);
    // display on, right shift, underline off, blink off
    LCD_SendCommand(lcd_addr, CMD_LCD_ON);
    // clear display (optional here)
    LCD_SendCommand(lcd_addr, CMD_LCD_CLEAR);
}

void LCD_SendString(uint8_t lcd_addr, char *str) {
    while(*str) {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
        str++;
    }
}

void LCD_SET_CGRAM(uint8_t lcd_addr, uint8_t addr, uint8_t *data) {
	uint8_t start_addr = LCD_CGRAM_BASE_ADDR | (addr << 3);
	LCD_SendCommand(lcd_addr, start_addr);
	for (int i = 0; i < 8; i++) {
		LCD_SendData(lcd_addr, data[i]);
	}
}

uint8_t BNumber[8] = {0x15,0x1d,0x17,0x1d,0x1,0x10,0x1f};
uint8_t BUp[8] = {0x4,0xe,0x1f,0x0,0x4,0xe,0x1f};
uint8_t BDown[8] = {0x4,0xe,0x1f,0x0,0x4,0xe,0x1f};
uint8_t BRight[8] = {0x10,0x18,0x1c,0x1e,0x1c,0x18,0x10};
uint8_t BLeft[8] = {0x10,0x18,0x1c,0x1e,0x1c,0x18,0x10};

unsigned char UART_Print_Port = 0; //0 = USB, 1 = LoRa, 2 = GPS
uint8_t UART1_Rx_Data[1];
uint8_t UART2_Rx_Data[1];
uint8_t UART3_Rx_Data[1];

uint8_t UART1_Rx_Buffer[20];

uint8_t UART1_Len = 0;

unsigned char UART1_Rx_End = 0;


int _write(int file, unsigned char *p, int len) {
	if (UART_Print_Port == 0) {
		HAL_UART_Transmit(&huart1, p, len, 10);
	} else if (UART_Print_Port == 1) {
		HAL_UART_Transmit(&huart2, p, len, 10);
	} else if (UART_Print_Port == 2) {
		HAL_UART_Transmit(&huart3, p, len, 10);
	}
	return len;
}
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
	if (UART_Print_Port == 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
	} else if (UART_Print_Port == 1) {
		HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	} else if (UART_Print_Port == 2) {
		HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);
	}
	return ch;
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, UART1_Rx_Data, 1);
	HAL_UART_Receive_IT(&huart2, UART2_Rx_Data, 1);
	HAL_UART_Receive_IT(&huart3, UART3_Rx_Data, 1);
	setvbuf(stdout, NULL, _IONBF, 0);
	printf("HELL WORLD\r\n");
	LCD_Init(LCD_ADDR);
	LCD_SET_CGRAM(LCD_ADDR, 0x00, BNumber);
	LCD_SET_CGRAM(LCD_ADDR, 0x01, BUp);
	LCD_SET_CGRAM(LCD_ADDR, 0x02, BDown);
	LCD_SET_CGRAM(LCD_ADDR, 0x03, BRight);
	LCD_SET_CGRAM(LCD_ADDR, 0x04, BLeft);
	LCD_SendCommand(LCD_ADDR, CMD_LCD_CLEAR);
	LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_LINE_1);
	LCD_SendString(LCD_ADDR, "604");
	LCD_SendData(LCD_ADDR, 0);
	for (int i = 0; i < 11; i++) {
		LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_RIGHT);
	}
	LCD_SendData(LCD_ADDR, 1);
	LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_LINE_2);
	LCD_SendData(LCD_ADDR, 3);
	LCD_SendString(LCD_ADDR, "43420");
	LCD_SendData(LCD_ADDR, 3);
	LCD_SendData(LCD_ADDR, 3);
	LCD_SendData(LCD_ADDR, 3);
	LCD_SendString(LCD_ADDR, "43080");
	for (int i = 0; i < 1; i++) {
		LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_RIGHT);
	}
	LCD_SendData(LCD_ADDR, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11); //LAMP2
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12); //LAMP1
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8); //BUZZER
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9); //Debug LED
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13); //Stop LED
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //GPS LED

		if (UART1_Rx_End) {
			printf("Echo\r\n");
			HAL_UART_Transmit(&huart1, UART1_Rx_Buffer, UART1_Len, 2);
			UART1_Len = 0;
			UART1_Rx_End = 0;
		}
		HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 230400;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LORA_M0_Pin|LORA_M1_Pin|LAMP2_Pin|LAMP1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STOP_LED_Pin|GPS_LED_Pin|BUZZER_Pin|DBG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LORA_M0_Pin LORA_M1_Pin LAMP2_Pin LAMP1_Pin */
  GPIO_InitStruct.Pin = LORA_M0_Pin|LORA_M1_Pin|LAMP2_Pin|LAMP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_AUX_Pin */
  GPIO_InitStruct.Pin = LORA_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin BTN3_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin|BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN4_Pin BTN5_Pin */
  GPIO_InitStruct.Pin = BTN4_Pin|BTN5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STOP_LED_Pin GPS_LED_Pin BUZZER_Pin DBG_LED_Pin */
  GPIO_InitStruct.Pin = STOP_LED_Pin|GPS_LED_Pin|BUZZER_Pin|DBG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE_SLCT_Pin */
  GPIO_InitStruct.Pin = MODE_SLCT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MODE_SLCT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_5) {
		printf("0x021,10x03\r\n");
	} else if (GPIO_Pin == GPIO_PIN_6) {
		printf("0x022,10x03\r\n");
	} else if (GPIO_Pin == GPIO_PIN_7) {
		printf("0x023,10x03\r\n");
	} else if (GPIO_Pin == GPIO_PIN_0) {
		printf("0x024,10x03\r\n");
	} else if (GPIO_Pin == GPIO_PIN_1) {
		printf("0x025,10x03\r\n ");
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static uint8_t UART1_Chk = 0;
	if (huart->Instance == USART1) {
		UART1_Rx_End = 0;
		switch (UART1_Chk) {
		case 0:
			if (UART1_Rx_Data[0] == 0x02) {
				// Rx_Buffer[USART1_len]=UART1_Rx_Data[0];
				// USART1_len++;
				UART1_Chk = 1;
			} else
				UART1_Chk = 0;
			break;
		case 1:
			if (UART1_Rx_Data[0] == 0x03) {
				UART1_Rx_End = 1;
				UART1_Chk = 0;
			} else {
				UART1_Rx_Buffer[UART1_Len] = UART1_Rx_Data[0];
				UART1_Len++;
			}
			break;
		default:
			UART1_Chk = 0;
			break;
		}
		HAL_UART_Receive_IT(&huart1, UART1_Rx_Data, 1);
	} else if (huart->Instance == USART2) {
		HAL_UART_Transmit(&huart1, UART2_Rx_Data, 1, 2);
		HAL_UART_Receive_IT(&huart2, UART2_Rx_Data, 1);
	} else if (huart->Instance == USART3) {
		HAL_UART_Transmit(&huart1, UART3_Rx_Data, 1, 2);
		HAL_UART_Receive_IT(&huart3, UART3_Rx_Data, 1);
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
	__disable_irq();
	while (1) {
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
