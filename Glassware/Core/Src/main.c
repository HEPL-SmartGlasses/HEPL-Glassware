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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "fatfs_sd.h"
#include <string.h>
#include "graph.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG

#define SD_SPI_HANDLE hspi2
#define SD_CS_GPIO_Port 'B'
#define SD_CS_GPIO_Pin 0

#define OLED_BAUD_RATE 9600
#define OLED_ADDR 0x3C

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
enum menuState {dir = 0, sel_start = 1, sel_dest = 2};
enum selectedLocation {eecs1311 = 0, wbathroom, mbathroom, vending, stairs};
menuState state;
selectedLocation location;
uint16_t numLocations = 5; // number of supported locations
uint16_t polygonSize = 4;  // standard polygon coordinates number
uint16_t *** map = malloc(numLocations * sizeof(uint16_t **)); // map storage
uint16_t polygonsRequired[5] = {1, 2, 1, 1, 1}; // buildings required polygon number
Graph * graph;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int Map_init_SD(){
	// open map file from sd card

	// Initialize SD card
	// some variables for FatFs
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
    char line[100]; /* Line buffer */
	FRESULT fres; //Result after operations
	string filename = "map.txt";

	f_mount(&FatFs, "", 1); // 1 = mount now

	fres = f_open(&fil, filename, FA_READ);
    if (fr) return (int)fr;

    int count = 0;
    while (f_gets(line, sizeof line, &fil)) {
    	count++;
    }

    /* Close the file */
    f_close(&fil);

	for (size_t i = 0; i < count){
		// allocate x/y coordinates
		map[i] = malloc( 4 * sizeof(uint16_t *));
	}

    // Reopen file and read content
	fres = f_open(&fil, filename, FA_READ);
    if (fr) return (int)fr;

    /* Read every line and display it */
    char * token;
    const char del = " ";

    int i = 0;
    while (f_gets(line, sizeof line, &fil)) {
    	int j = 0;
        // printf(line);
        token = strtok(line, del);
        int num = atoi(token);
        map[i][j] = num;

        while(token != NULL){
        	if (j > 3){ // file is badly formatted
        		return 1;
        	}
        	j++;
        	token = strtok(NULL, del);
        	num = atoi(token);
        	map[i][j] = num;
        }
        i++;
    }

    /* Close the file */
    f_close(&fil);

    buildGraphFromMap(graph, map, count);

    return 0;

}

void Map_init() {
	// allocate location storage
	// coordinates are acceses by map[loc][axis][polygonNum]
	// i.e EECS 1311 x coordination of 2nd polygon is map[0][0][1]
	for (size_t i = 0; i < numLocations){
		// allocate x/y coordinates
		map[i] = malloc( 2 * sizeof(uint16_t *));

		// allocate polygon size for each
		map[i][0] = malloc( (polygonSize * polygonsRequired[i]) * sizeof(uint16_t *));
		map[i][1] = malloc( (polygonSize * polygonsRequired[i]) * sizeof(uint16_t *));
	}

	// TODO: put actual coordinates stored in CCW fashion from bottom left

	// EECS 1311 coordinates
	map[0][0][0] = 1; map[0][0][1] = 1; map[0][0][2] = 1; map[0][0][3] = 1; // x
	map[0][1][0] = 1; map[0][1][1] = 1; map[0][1][2] = 1; map[0][1][3] = 1; // y

	// Women's bathroom coordinates
	map[1][0][0] = 1; map[1][0][1] = 1; map[1][0][2] = 1; map[1][0][3] = 1; // x
	map[1][0][4] = 1; map[1][0][5] = 1; map[1][0][6] = 1; map[1][0][7] = 1; // x

	map[1][1][0] = 1; map[1][1][1] = 1; map[1][1][2] = 1; map[1][1][3] = 1; // y
	map[1][1][4] = 1; map[1][1][5] = 1; map[1][1][6] = 1; map[1][1][7] = 1; // y

	// Men's bathroom coordinates
	map[2][0][0] = 1; map[2][0][1] = 1; map[2][0][2] = 1; map[2][0][3] = 1; // x
	map[2][1][0] = 1; map[2][1][1] = 1; map[2][1][2] = 1; map[2][1][3] = 1; // y

	// Vending machine coordinates
	map[3][0][0] = 1; map[3][0][1] = 1; map[3][0][2] = 1; map[3][0][3] = 1; // x
	map[3][1][0] = 1; map[3][1][1] = 1; map[3][1][2] = 1; map[3][1][3] = 1; // y

	// Stairs coordinates
	map[4][0][0] = 1; map[4][0][1] = 1; map[4][0][2] = 1; map[4][0][3] = 1; // x
	map[4][1][0] = 1; map[4][1][1] = 1; map[4][1][2] = 1; map[4][1][3] = 1; // y
}

bool is_inside_polygon(uint16_t x, uint16_t y){
	uint16_t numPol = polygonsRequired[location];
	bool is_inside = false;

	for(size_t i = 0; i < numPol; i++){
		// retrieve position data
		uint16_t x0 = map[location][0][4*i]    , x1 = map[location][0][4*i + 1],
				 x2 = map[location][0][4*i + 2], x3 = map[location][0][4*i + 3],
				 y0 = map[location][1][4*i]    , y1 = map[location][1][4*i + 1],
				 y2 = map[location][1][4*i + 2], y3 = map[location][1][4*i + 3];

		// check bounds
		is_inside |= ((x > x0) && (x > x3) && (x < x2) && (x < x1)) &&
				     ((y > y0) && (y > y1) && (y < y2) && (y < y3));
	}

	return is_inside;
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
  graph = createGraph();
  Map_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize screen
  enum menuState menu = dir;
  SSD1306_Init();
  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("HEPL", &Font_11x18, 1);
  SSD1306_UpdateScreen(); //display
  HAL_Delay(4000);

  // Initialize SD card
  // some variables for FatFs
  FATFS FatFs; 	//Fatfs handle
  FIL fil; 		//File handle
  FRESULT fres; //Result after operations
  string filename = "map.txt";

  fres = f_mount(&FatFs, "", 1); // 1 = mount now
  if (fres != FR_OK)
  {
      #ifdef DEBUG
	  SSD1306_GotoXY (0,0);
	  SSD1306_Puts ("ErrSD-Mnt", &Font_11x18, 1); // error mounting
	  SSD1306_UpdateScreen(); //display
      #endif
	  while(1);
  }

  #ifdef DEBUG
  DWORD free_clusters, free_sectors, total_sectors;
  FATFS* getFreeFs;
  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK)
  {
	  SSD1306_GotoXY (0,0);
	  SSD1306_Puts ("ErrSD-GFr", &Font_11x18, 1); // error getting free
	  SSD1306_UpdateScreen(); //display
	  while(1);
  }
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;
  #endif

  fres = f_open(&fil, filename, FA_READ);
  if (fres != FR_OK) {
      #ifdef DEBUG
	  SSD1306_GotoXY (0,0);
	  SSD1306_Puts ("ErrSD-OpF", &Font_11x18, 1); // error opening file
	  SSD1306_UpdateScreen();
 	  #endif
	  while(1);
  }

  BYTE readBuf[30];
  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
  if (rres == 0)
  {
      #ifdef DEBUG
	  SSD1306_GotoXY (0,0);
	  SSD1306_Puts ("ErrSD-RdF", &Font_11x18, 1); // error reading file
	  SSD1306_UpdateScreen();
      #endif
	  while(1);
  }
  f_close(&fil);
  #ifdef DEBUG
  SSD1306_GotoXY (0,0);
  SSD1306_Puts(strcat("File: ", filename), &Font_11x18, 1);
  SSD1306_GotoXY (11,0);
  SSD1306_Puts(readBuf, &Font_11x18, 1);
  SSD1306_UpdateScreen();
  HAL_Delay(2000);
  #endif

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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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
  hi2c1.Init.Timing = 0x00000E14;
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
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_EN_GPIO_Port, BT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CAM_CS_Pin|FLASH_WP_Pin|SWO_Pin|SPI3_XBEE_ATTN_Pin
                          |SPI3_XBEE_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_SD_CS_Pin|SPI2_FLASH_CS_Pin|FLASH_HOLD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_EN_Pin */
  GPIO_InitStruct.Pin = BT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CAM_CS_Pin FLASH_WP_Pin SWO_Pin SPI3_XBEE_ATTN_Pin
                           SPI3_XBEE_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CAM_CS_Pin|FLASH_WP_Pin|SWO_Pin|SPI3_XBEE_ATTN_Pin
                          |SPI3_XBEE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_SD_CS_Pin SPI2_FLASH_CS_Pin FLASH_HOLD_Pin */
  GPIO_InitStruct.Pin = SPI2_SD_CS_Pin|SPI2_FLASH_CS_Pin|FLASH_HOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SWDIO_Pin */
  GPIO_InitStruct.Pin = SWDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWDIO_GPIO_Port, &GPIO_InitStruct);

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
