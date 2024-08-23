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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_RF_FREQUENCY 920900000
#define EEPROM_BASE_ADDRESS 0x08080000
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
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
struct DataFlash {
	char busNM[5];
	char busRouteno[5];
	char busStopID[8];
	char lati[16];
	char longi[16];
}data[150];

int nowIdx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//LCD ============================================
HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data,
		uint8_t flags) {
	HAL_StatusTypeDef res;
	for (;;) {
		res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1, HAL_MAX_DELAY);
		if (res == HAL_OK)
			break;
	}

	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	data_arr[0] = up | flags | LCD_BACKLIGHT | LCD_PIN_EN;
	data_arr[1] = up | flags | LCD_BACKLIGHT;
	data_arr[2] = lo | flags | LCD_BACKLIGHT | LCD_PIN_EN;
	data_arr[3] = lo | flags | LCD_BACKLIGHT;

	res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, sizeof(data_arr),
	HAL_MAX_DELAY);
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
	while (*str) {
		LCD_SendData(lcd_addr, (uint8_t) (*str));
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

void LCD_Write_Info(struct DataFlash nowData, struct DataFlash nextData){
	LCD_SendCommand(LCD_ADDR, CMD_LCD_CLEAR); //Clear
	LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_LINE_1);
	LCD_SendString(LCD_ADDR, nowData.busRouteno);
	LCD_SendData(LCD_ADDR, 0);
	for (int i = 0; i < 11; i++) {
		LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_RIGHT);
	}
	LCD_SendData(LCD_ADDR, 1);
	LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_LINE_2);
	LCD_SendData(LCD_ADDR, 3);
	LCD_SendString(LCD_ADDR, nowData.busStopID);
	LCD_SendData(LCD_ADDR, 3);
	LCD_SendData(LCD_ADDR, 3);
	LCD_SendData(LCD_ADDR, 3);
	LCD_SendString(LCD_ADDR, nextData.busStopID);
	for (int i = 0; i < 1; i++) {
		LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_RIGHT);
	}
	LCD_SendData(LCD_ADDR, 1);
}

//Flash===========================================================


int dataIdx = 0;

void Flash_Unlock(void) {
	FLASH->KEYR = 0x45670123;  // Key1
	FLASH->KEYR = 0xCDEF89AB;  // Key2
}

void Flash_Lock(void) {
	FLASH->CR |= FLASH_CR_LOCK;
}

void Flash_Write(uint32_t address, uint8_t data) {
	while (FLASH->SR & FLASH_SR_BSY)
		;  // Busy flag 체크

	FLASH->CR |= FLASH_CR_PG;  // Programming mode ?��?��

	*(__IO uint16_t*) address = data;  // ?��?��?�� 기록

	while (FLASH->SR & FLASH_SR_BSY)
		;  // Busy flag 체크

	FLASH->CR &= ~FLASH_CR_PG;  // Programming mode ?��?��
}

void Flash_Write_StrInt(uint32_t address, uint8_t *StrData) {
	Flash_Unlock();  // ?��?��?�� 메모�?? ?��?��
	uint16_t value = (uint16_t) strtol((const char*) StrData, NULL, 10);
	Flash_Write(address, value);  // ?��?�� 값을 ?��?��?�� 메모리에 ???��
	Flash_Lock();  // ?��?��?�� 메모�?? ?���??
}

uint32_t Flash_Write_Char(uint32_t address, uint8_t CharData){
	Flash_Unlock();
	Flash_Write(address, CharData);
	Flash_Lock();
	return address+0x02;
}

uint32_t Flash_Write_Str(uint32_t address, uint8_t *StrData){
	for(int i = 0;i<strlen((char *)StrData);i++){
		address = Flash_Write_Char(address, StrData[i]);
	}
	//printf("%x\r\n", address);
	return address;
}

uint32_t Flash_Write_Data(uint32_t address, uint8_t *StrData) {
	char *token;

	token = strtok(StrData, ",");
	if (token[0] == 'D') {
		token = strtok(NULL, ",");
		if (token != NULL) {
			strncpy(data[dataIdx].busNM, token, sizeof(data[dataIdx].busNM) - 1);
			data[dataIdx].busNM[sizeof(data[dataIdx].busNM) - 1] = '\0';
			address = Flash_Write_Str(address, data[dataIdx].busNM);
			address = Flash_Write_Char(address, ',');
		}

		token = strtok(NULL, ",");
		if (token != NULL) {
			strncpy(data[dataIdx].busRouteno, token, sizeof(data[dataIdx].busRouteno) - 1);
			data[dataIdx].busRouteno[sizeof(data[dataIdx].busRouteno) - 1] = '\0';
			address = Flash_Write_Str(address, data[dataIdx].busRouteno);
			address = Flash_Write_Char(address, ',');
		}

		token = strtok(NULL, ",");
		if (token != NULL) {
			strncpy(data[dataIdx].busStopID, token, sizeof(data[dataIdx].busStopID) - 1);
			data[dataIdx].busStopID[sizeof(data[dataIdx].busStopID) - 1] = '\0';
			address = Flash_Write_Str(address, data[dataIdx].busStopID);
			address = Flash_Write_Char(address, ',');
		}
	} else {
		token = strtok(NULL, ",");
		if (token != NULL) {
			strncpy(data[dataIdx].lati, token, sizeof(data[dataIdx].lati) - 1);
			data[dataIdx].lati[sizeof(data[dataIdx].lati) - 1] = '\0';
			address = Flash_Write_Str(address, data[dataIdx].lati);
			address = Flash_Write_Char(address, ',');
		}

		token = strtok(NULL, ",");
		if (token != NULL) {
			strncpy(data[dataIdx].longi, token, sizeof(data[dataIdx].longi) - 1);
			data[dataIdx].longi[sizeof(data[dataIdx].longi) - 1] = '\0';
			address = Flash_Write_Str(address, data[dataIdx].longi);
			address = Flash_Write_Char(address, '!');
		}
		dataIdx += 1;
		if (dataIdx ==2){
			LCD_Write_Info(data[nowIdx], data[nowIdx+1]);
		}
	}
//	printf("busNM:%s, busRouteNo:%s, BusStopID:%s, lati:%s, longi:%s\r\n",
//			data.busNM, data.busRouteno, data.busStopID, data.lati, data.longi);
	return address;
}

uint16_t Flash_Read(uint32_t address) {
	return *(__IO uint16_t*) address; // �???��?�� ?��?��?�� 메모�?? 주소?��?�� ?��?��?�� ?���??
}

void Flash_Erase_Page(uint32_t address) {
	Flash_Unlock();  // ?��?��?�� 메모�?? ?��?��

	FLASH->CR |= FLASH_CR_PER;   // Page Erase 비트 ?��?��
	FLASH->AR = address;         // �???�� ?��?���???�� 주소 ?��?��
	FLASH->CR |= FLASH_CR_STRT;  // Erase ?��?��

	while (FLASH->SR & FLASH_SR_BSY)
		;  // ?��?��?�� ?��료될 ?��까�? ??�??

	FLASH->CR &= ~FLASH_CR_PER;  // Page Erase 비트 ?��?��

	Flash_Lock();  // ?��?��?�� 메모�?? ?���??
}

void splitData(char* strData){
	char *token;

	token = strtok(strData, ","); // CarNM
	strncpy(data[dataIdx].busNM, token, sizeof(data[dataIdx].busNM) - 1);

	token = strtok(NULL, ","); // RouteNo
	strncpy(data[dataIdx].busRouteno, token, sizeof(data[dataIdx].busRouteno) - 1);

	token = strtok(NULL, ","); // StopID
	strncpy(data[dataIdx].busStopID, token, sizeof(data[dataIdx].busStopID) - 1);

	token = strtok(NULL, ","); // lati
	strncpy(data[dataIdx].lati, token, sizeof(data[dataIdx].lati) - 1);

	token = strtok(NULL, "!"); // longi
	strncpy(data[dataIdx].longi, token, sizeof(data[dataIdx].longi) - 1);
}

uint32_t CallData(uint32_t address){
	char a[70] = {0,};
	int i = 0;
	while(1){
		a[i] = (char)Flash_Read(address);
		address += 0x02;
		if(a[i] == 0xFF){
			return address;
		}
		if(a[i] == '!'){
			splitData(a);
			dataIdx+=1;
			i = -1;
			memset(a, 0, 60);
		}
		i += 1;
	}
}

// GPS=======================================================
char latitude[16];
char longitude[16];

double convertToDecimalDegrees(const char* coordinate, char type) {
    int degrees;
    double minutes;
    double decimalDegrees;

    if (type == 'L') { // Latitude
        // 첫 두 자리 (도)
        degrees = (coordinate[0] - '0') * 10 + (coordinate[1] - '0'); // dd
        // 나머지 부분 (분)
        minutes = atof(coordinate + 2); // mm.mmmm
    } else if (type == 'G') { // Longitude
        // 첫 세 자리 (도)
        degrees = (coordinate[0] - '0') * 100 + (coordinate[1] - '0') * 10 + (coordinate[2] - '0'); // ddd
        // 나머지 부분 (분)
        minutes = atof(coordinate + 3); // mm.mmmm
    } else {
        printf("Invalid type\n");
        return;
    }

    // 소수점 부분 계산
    decimalDegrees = degrees + (minutes / 60.0);

    return decimalDegrees;
}

void parseGPSData(uint8_t *buffer, uint16_t size) {
	char *nmeaGGA = NULL;
	double la, lo;
	//printf("%s", (char*)buffer);
	// DMA 버퍼?��?�� $GPGGA 문자?��?�� �??��
	nmeaGGA = strstr((char*) buffer, "GLL");
	if (nmeaGGA != NULL) {
		char *token;

		// NMEA 메시�? ?��?��?��
		token = strtok(nmeaGGA, ",");

//        // UTC ?���? (무시)
//        token = strtok(NULL, ",");

		// ?��?��
		token = strtok(NULL, ",");
		if (token != NULL) {
			strncpy(latitude, token, sizeof(latitude) - 1);
			latitude[sizeof(latitude) - 1] = '\0';
			la = convertToDecimalDegrees(latitude, 'L');
		}

		// N/S ?��?��
		token = strtok(NULL, ",");

		// 경도
		token = strtok(NULL, ",");
		if (token != NULL) {
			strncpy(longitude, token, sizeof(longitude) - 1);
			longitude[sizeof(longitude) - 1] = '\0';
			lo = convertToDecimalDegrees(longitude, 'G');
		}

		// E/W ?��?��
		token = strtok(NULL, ",");

		// ?��?��?�� 결과�? ?��버그 출력
		printf("\r\nLatitude: %.6f, Longitude: %.6f\r\n", la, lo);
	}
}

//==============================================================================
//LoRa
#define LoRa_RX_BUFFER_SIZE 64

uint8_t LoRaRxBuffer[LoRa_RX_BUFFER_SIZE]; // 수신 데이터를 저장할 버퍼
volatile uint8_t rxCompleteFlag = 0; // 데이터 수신 완료 플래그

void SetMode(uint8_t mode) {
	switch (mode) {
	case 0:
		HAL_GPIO_WritePin(GPIOA, LORA_M0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, LORA_M1_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOA, LORA_M0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, LORA_M1_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOA, LORA_M0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, LORA_M1_Pin, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOA, LORA_M0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, LORA_M1_Pin, GPIO_PIN_SET);
		break;
	}
}

void LoRa_SendData(uint8_t *data, uint16_t length) {
	// AUX 핀이 HIGH 상태인지 확인하여 모듈이 준비되었는지 확인
	while (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET)
		;

	// 데이터 송신
	HAL_UART_Transmit(&huart2, data, length, HAL_MAX_DELAY);
}

uint8_t BNumber[8] = { 0x15, 0x1d, 0x17, 0x1d, 0x1, 0x10, 0x1f };
uint8_t BUp[8] = { 0x4, 0xe, 0x1f, 0x0, 0x4, 0xe, 0x1f };
uint8_t BDown[8] = { 0x4, 0xe, 0x1f, 0x0, 0x4, 0xe, 0x1f };
uint8_t BRight[8] = { 0x10, 0x18, 0x1c, 0x1e, 0x1c, 0x18, 0x10 };
uint8_t BLeft[8] = { 0x10, 0x18, 0x1c, 0x1e, 0x1c, 0x18, 0x10 };

unsigned char UART_Print_Port = 0; //0 = USB, 1 = LoRa, 2 = GPS
uint8_t UART1_Rx_Data[1];

uint8_t flagUart3 = 0;

uint8_t UART1_Rx_Buffer[50];
uint8_t UART1_Len = 0;

unsigned char UART1_Rx_End = 0;

#define RX3_BUFFER_SIZE 256
uint8_t rxBuffer[RX3_BUFFER_SIZE];
uint8_t nmeaBuffer[RX3_BUFFER_SIZE];
uint8_t dataReceived = 0;

uint8_t modeFlag = 0;



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
int main(void) {

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
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, UART1_Rx_Data, 1);
	HAL_UART_Receive_IT(&huart2, LoRaRxBuffer, 1);
	HAL_UART_Receive_IT(&huart3, rxBuffer, 1);
	setvbuf(stdout, NULL, _IONBF, 0);
	//	printf("HELL WORLD\r\n");
	LCD_Init(LCD_ADDR);
	LCD_SET_CGRAM(LCD_ADDR, 0x00, BNumber);
	LCD_SET_CGRAM(LCD_ADDR, 0x01, BUp);
	LCD_SET_CGRAM(LCD_ADDR, 0x02, BDown);
	LCD_SET_CGRAM(LCD_ADDR, 0x03, BRight);
	LCD_SET_CGRAM(LCD_ADDR, 0x04, BLeft);
//	LCD_SendCommand(LCD_ADDR, CMD_LCD_CLEAR); //Clear
//	LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_LINE_1);
//	LCD_SendString(LCD_ADDR, "604");
//	LCD_SendData(LCD_ADDR, 0);
//	for (int i = 0; i < 11; i++) {
//		LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_RIGHT);
//	}
//	LCD_SendData(LCD_ADDR, 1);
//	LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_LINE_2);
//	LCD_SendData(LCD_ADDR, 3);
//	LCD_SendString(LCD_ADDR, "43420");
//	LCD_SendData(LCD_ADDR, 3);
//	LCD_SendData(LCD_ADDR, 3);
//	LCD_SendData(LCD_ADDR, 3);
//	LCD_SendString(LCD_ADDR, "43080");
//	for (int i = 0; i < 1; i++) {
//		LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_RIGHT);
//	}
//	LCD_SendData(LCD_ADDR, 1);

	//flash
	uint32_t ModeFlashAddress = 0x0800CB00;  // ???��?�� ?��?��?�� 메모�?? 주소
	uint32_t DataFlashAddress = 0x0800CC00; // ???��?�� ?��?��?�� 메모�?? 주소
	uint16_t InfoModeFlag = Flash_Read(ModeFlashAddress);
//	Flash_Erase_Page(DataFlashAddress);
//	Flash_Erase_Page(0x0800D000);
//	Flash_Erase_Page(0x0800D400);
//	Flash_Erase_Page(0x0800D800);
//	Flash_Erase_Page(0x0800DC00);
//	Flash_Erase_Page(0x0800E000);
//	Flash_Erase_Page(0x0800E400);

	//printf("ModeFlag:%d", InfoModeFlag);
	if (InfoModeFlag == 1) {
		DataFlashAddress = CallData(DataFlashAddress);
		strncpy(data[0].busStopID, "44444", sizeof(data[0].busStopID) - 1);
		strncpy(data[0].lati, "127.362724", sizeof(data[0].lati) - 1);
		strncpy(data[0].longi, "36.391382", sizeof(data[0].longi) - 1);

		LCD_Write_Info(data[nowIdx], data[nowIdx+1]);
	} else if (InfoModeFlag == 0) {
		LCD_SendCommand(LCD_ADDR, CMD_LCD_CLEAR); //Clear
		LCD_SendCommand(LCD_ADDR, CMD_LCD_CURSOR_LINE_1);
		LCD_SendString(LCD_ADDR, "InputMode");
	} else {
		InfoModeFlag = 0;
		Flash_Unlock();
		Flash_Write(ModeFlashAddress, 0);
		Flash_Lock();
	}

	HAL_UART_Receive_DMA(&huart3, rxBuffer, RX3_BUFFER_SIZE);

	//LoRa ================================================================
	SetMode(0);

	uint8_t data[] = "Hello LoRa!";

	//FW===================================================================
	modeFlag = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11); //LAMP2
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12); //LAMP1
//		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8); //BUZZER
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9); //Debug LED
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13); //Stop LED
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //GPS LED

		if (!modeFlag) { //Local Mode
			while(1){
				if (UART1_Rx_End) {
					//printf("Echo\r\n");
					if (!strcmp(UART1_Rx_Buffer, "Input")) {
						Flash_Erase_Page(ModeFlashAddress);
						Flash_Unlock();
						Flash_Write(ModeFlashAddress, (uint8_t)0);
						Flash_Lock();
					} else if (!strcmp(UART1_Rx_Buffer, "OutPut")) {
						Flash_Erase_Page(ModeFlashAddress);
						Flash_Unlock();
						Flash_Write(ModeFlashAddress, 1);
						Flash_Lock();
					} else if ((!strncmp(UART1_Rx_Buffer, "Data", 4)
							|| !strncmp(UART1_Rx_Buffer, "data", 4))
							&& InfoModeFlag == 0) {
						DataFlashAddress = Flash_Write_Data(DataFlashAddress, UART1_Rx_Buffer);
						//printf("Data\r\n");
						printf("N\r\n");
					}
					//HAL_UART_Transmit(&huart1, UART1_Rx_Buffer, UART1_Len, 2);
					for (int i = 0; i < 50; i++) {
						UART1_Rx_Buffer[i] = '\0';
					}
					UART1_Len = 0;
					UART1_Rx_End = 0;
				}

				if (InfoModeFlag){
					if (dataReceived) {
						parseGPSData(rxBuffer, RX3_BUFFER_SIZE);
						dataReceived = 0;
					}
				}
			}
		}


//
//		LoRa_SendData(data, sizeof(data) - 1);
//
//		if (rxCompleteFlag) {
//			rxCompleteFlag = 0; // 수신 완료 플래그 리셋
//			printf("LoRa : %s\r\n", LoRaRxBuffer);
//		}
//
//		HAL_Delay(100);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LORA_M0_Pin | LORA_M1_Pin | LAMP2_Pin | LAMP1_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	STOP_LED_Pin | GPS_LED_Pin | BUZZER_Pin | DBG_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LORA_M0_Pin LORA_M1_Pin LAMP2_Pin LAMP1_Pin */
	GPIO_InitStruct.Pin = LORA_M0_Pin | LORA_M1_Pin | LAMP2_Pin | LAMP1_Pin;
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
	GPIO_InitStruct.Pin = BTN1_Pin | BTN2_Pin | BTN3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN4_Pin BTN5_Pin */
	GPIO_InitStruct.Pin = BTN4_Pin | BTN5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : STOP_LED_Pin GPS_LED_Pin BUZZER_Pin DBG_LED_Pin */
	GPIO_InitStruct.Pin = STOP_LED_Pin | GPS_LED_Pin | BUZZER_Pin | DBG_LED_Pin;
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
		printf("0x020,10x03\r\n");
	} else if (GPIO_Pin == GPIO_PIN_6) {
		printf("0x021,10x03\r\n");
	} else if (GPIO_Pin == GPIO_PIN_7) {
		printf("0x022,10x03\r\n");
	} else if (GPIO_Pin == GPIO_PIN_0) {
		printf("0x023,10x03\r\n");
	} else if (GPIO_Pin == GPIO_PIN_1) {
		printf("0x024,10x03\r\n ");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static uint8_t UART1_Chk = 0;
	static uint16_t index = 0;
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
		//HAL_UART_Transmit(&huart1, UART1_Rx_Data, 1, 10);
		HAL_UART_Receive_IT(&huart1, UART1_Rx_Data, 1);
	} else if (huart->Instance == USART2) {
		rxCompleteFlag = 1;
		HAL_UART_Receive_IT(&huart2, LoRaRxBuffer, 1);
	} else if (huart->Instance == USART3) {
		dataReceived = 1;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
