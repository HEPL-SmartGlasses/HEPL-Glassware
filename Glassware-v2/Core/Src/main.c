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
#include "fatfs.h"
#include "path.h"
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "fonts.h"
#include "ssd1306.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HSPI_SDCARD &hspi2
#define SD_CS_PORT GPIOB
#define SD_CS_PIN GPIO_PIN_0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */
enum menuState {dir = 0, sel_start = 1, sel_dest = 2};
enum selectedLocation {eecs1311 = 0, wbathroom, mbathroom, vending, stairs, none};
enum menuState menu;
enum selectedLocation location = none;
double ** map;     // map storage
double ** destMap; // map storage
Graph * graph; // storage for shortest path algorithm
double curPosX;
double curPosY;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool hasArrived(){
	// check if current position is close to the destination
	return true;
}

double computeNextStep(){

	int startIdx = findNode(graph, curPosX, curPosY);
	int destinationIdx = findNode(graph, destMap[location][0], destMap[location][1]);

	// find shortest path from current position to destination
	int * path = findShortestPath(graph, startIdx, destinationIdx);

	// find immediate route for user
	// compute arrow heading (based on user orientation)
	double theta = heading(graph, path);
	return theta;
}

void getCurrentPosition(){
// request data from Foot Board
	curPosX = destMap[0][0];
	curPosY = destMap[0][1];
	return;
}

void displayArrow(double theta){
	  SSD1306_GotoXY (0,0);
	  SSD1306_DrawArrow(1, 1, theta, 0);
	  SSD1306_UpdateScreen(); //display
}

void Destination_init(){
	int locNum = 5;
    destMap = malloc(locNum * sizeof(double *));
	for (int i = 0; i < locNum; i++){
		// allocate x/y coordinates
		destMap[i] = malloc( 2 * sizeof(double));
	}

	// eecs 1311
	destMap[0][0] = 16.96;
	destMap[0][1] = 78.64;

	// womem's bathroom
	destMap[1][0] = 41.12;
	destMap[1][1] = 82.48;

	// men's bathroom
	destMap[0][0] = 48.48;
	destMap[0][1] = 82.80;

	// vending machine
	destMap[0][0] = 39.36;
	destMap[0][1] = 62.48;

	// stairs
	destMap[0][0] = 48.32;
	destMap[0][1] = 78.48;
}

int Map_init_SD(){
	// open map file from sd card

	// Initialize SD card
	// some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
    char line[100]; // Line buffer
	FRESULT fres;   //Result after operations
	char* filename = "map.txt";

	fres = f_mount(&FatFs, "0:", 1); // 1 = mount now

	fres = f_open(&fil, filename, FA_READ);
    if (fres) return (int)fres;

    int count = 0;
    while (f_gets(line, sizeof line, &fil)) {
    	count++;
    }

    /* Close the file */
    f_close(&fil);

    map = malloc(count * sizeof(double *));
	for (int i = 0; i < count; i++){
		// allocate x/y coordinates
		map[i] = malloc( 4 * sizeof(double));
	}

    // Reopen file and read content
	fres = f_open(&fil, filename, FA_READ);
    if (fres) return (int)fres;

    /* Read every line and display it */
    char * token;
    const char* del = " ";

    int i = 0;
    while (f_gets(line, sizeof(line), &fil)) {
    	int j = 0;
        // printf(line);
        token = strtok(line, del);
        double num = atof(token);
        map[i][j] = num;

        while(token != NULL){
        	if (j >= 3){ // file is badly formatted
        		break;
        	}
        	j++;
        	token = strtok(NULL, del);
        	num = atof(token);
        	map[i][j] = num;
        }
        i++;
    }

    /* Close the file */
    f_close(&fil);

    buildGraphFromMap(graph, map, count);

    return 0;

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
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI3_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  graph = createGraph();
  Map_init_SD();
  Destination_init();

  // Initialize screen
  menu = dir;
  SSD1306_Init();

//  //DRESULT temp = SD_disk_initialize(0);
//
//  // Initialize SD card
//  // some variables for FatFs
//  FATFS FatFs; 	//Fatfs handle
//  FIL fil; 		//File handle
//  FRESULT fres; //Result after operations
//  char* filename = "map.txt";
//
//
////  uint8_t buf_tx[1] = {0xFF};
////  uint8_t buf_rx[1] = {0x00};
////  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
////  HAL_SPI_TransmitReceive(&hspi2, buf_tx, buf_rx, 1, 2);
////  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
//
//  fres = f_mount(&FatFs, "0:", 1); // 1 = mount now
//  if (fres != FR_OK)
//  {
//      #ifdef DEBUG
//	  SSD1306_GotoXY (0,0);
//	  SSD1306_Puts ("ErrSD-Mnt", &Font_11x18, 1); // error mounting
//	  SSD1306_UpdateScreen(); //display
//      #endif
////	  while(1);
//  }
//
//  #ifdef DEBUG
//  DWORD free_clusters, free_sectors, total_sectors;
//  FATFS* getFreeFs;
//  fres = f_getfree("", &free_clusters, &getFreeFs);
//  if (fres != FR_OK)
//  {
//	  SSD1306_GotoXY (0,0);
//	  SSD1306_Puts ("ErrSD-GFr", &Font_11x18, 1); // error getting free
//	  SSD1306_UpdateScreen(); //display
////	  while(1);
//  }
//  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
//  free_sectors = free_clusters * getFreeFs->csize;
//  #endif
//
//  fres = f_open(&fil, filename, FA_READ);
//  if (fres != FR_OK) {
//      #ifdef DEBUG
//	  SSD1306_GotoXY (0,0);
//	  SSD1306_Puts ("ErrSD-OpF", &Font_11x18, 1); // error opening file
//	  SSD1306_UpdateScreen();
// 	  #endif
////	  while(1);
//  }
//
//  BYTE readBuf[50];
//  TCHAR* rres = f_gets((TCHAR*)readBuf, 50, &fil);
//  if (rres == 0)
//  {
//      #ifdef DEBUG
//	  SSD1306_GotoXY (0,0);
//	  SSD1306_Puts ("ErrSD-RdF", &Font_11x18, 1); // error reading file
//	  SSD1306_UpdateScreen();
//      #endif
////	  while(1);
//  }
//  f_close(&fil);
//  #ifdef DEBUG
//  SSD1306_GotoXY (0,0);
//  SSD1306_Puts(strcat("File: ", filename), &Font_11x18, 1);
//  SSD1306_GotoXY (11,0);
//  SSD1306_Puts(readBuf, &Font_11x18, 1);
//  SSD1306_UpdateScreen();
//  HAL_Delay(2000);
//  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  SSD1306_GotoXY (0,0);
	  SSD1306_Puts ("HEPL WORLD :)", &Font_11x18, 1);
	  SSD1306_UpdateScreen(); //display
//
//	  HAL_Delay (2000);
	   if (location == none){
		   // UI
		   // set up interrupts for buttons
		   location = wbathroom;
	   } else {
		   getCurrentPosition(); // find user position
		   double heading = computeNextStep();
		   displayArrow(heading);
		   HAL_Delay (2000);

		   if (hasArrived()){
			 location = none;
			 // display congrats message
		   }
	   }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
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
  hi2c1.Init.Timing = 0x0010061A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_SD_CS_GPIO_Port, SPI2_SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_SD_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

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
