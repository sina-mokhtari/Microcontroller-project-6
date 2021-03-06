/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
#include "LiquidCrystal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned char byte;

enum logType {
	lightLow, lightHigh, motion, temperatureRaise
};

typedef struct {
	RTC_DateTypeDef date;
	RTC_TimeTypeDef time;
	enum logType type;

} log;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_D8 GPIO_PIN_8
#define LCD_D9 GPIO_PIN_9
#define LCD_D10 GPIO_PIN_10
#define LCD_D11 GPIO_PIN_11
#define LCD_D12 GPIO_PIN_12
#define LCD_D13 GPIO_PIN_13
#define LCD_D14 GPIO_PIN_14

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LOG_BUFFER_SIZE 100
#define TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM 128
#define ADC_DELAY 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

uint_fast16_t temperatureSamplingArr[TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM];
uint_fast16_t lightSamplingArr[TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM];

RTC_TimeTypeDef rtcTime;
RTC_DateTypeDef rtcDate;

uint_fast32_t temperatureSamplingIdx = 0;
uint_fast32_t sum = 25 * TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM;
uint_fast32_t temperatueRawValue, lightRawValue;
uint_fast32_t lastTemperature = 0;
uint_fast32_t newLight = 0;
uint_fast32_t lastLight = 0;
uint_fast32_t newTemperature;
uint_fast32_t normalTemperature;
uint_fast32_t normalLight;
uint_fast32_t temperatureSamplingSum = 25 * TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM,
		lightSamplingSum = 30 * TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM;
uint_fast32_t lightSamplingIdx = 0;

bool buzzerOn = false;
bool warning = false;

log logBuffer[LOG_BUFFER_SIZE];
uint_fast32_t logIdx = 0;

TIM_HandleTypeDef *buzzerPwmTimer = &htim8;
uint32_t buzzerPwmChannel = TIM_CHANNEL_1;

byte black[8] = { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
byte barEmpty[8] = { 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F };
byte barStart[8] = { 0x1F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F };
byte barEnd[8] = { 0x1F, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x1F };

byte sunCenter[8] = { 0B00100, 0B01110, 0B10001, 0B10001, 0B10001, 0B01110, 0B00100, 0B00000 };
byte sunShineRight[8] = { 0x08, 0x10, 0x00, 0x1E, 0x00, 0x10, 0x08, 0x00 };
byte sunShineLeft[8] = { 0x02, 0x01, 0x00, 0x0F, 0x00, 0x01, 0x02, 0x00 };

byte thermometer[8] = { 0x00, 0x06, 0x06, 0x06, 0x06, 0x09, 0x09, 0x06 };
byte degree[8] = { 0x00, 0x07, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void buzzerChangeTone(uint16_t freq, uint16_t volume);
void printDateTime();
void updateLight();
void updateTemperature();
void warningOn();
void warningOff();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_USB_PCD_Init();
	MX_RTC_Init();
	MX_USART2_UART_Init();
	MX_TIM6_Init();
	MX_TIM8_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM7_Init();
	MX_TIM4_Init();
	MX_TIM16_Init();
	/* USER CODE BEGIN 2 */

	//__NVIC_DisableIRQ(EXTI0_IRQn);
	rtcTime.Seconds = 0;
	rtcTime.Minutes = 0;
	rtcTime.Hours = 0;

	rtcDate.Year = 0;
	rtcDate.Month = 0;
	rtcDate.Date = 0;

	HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

	LiquidCrystal(GPIOD, LCD_D8, LCD_D9, LCD_D10, LCD_D11, LCD_D12, LCD_D13,
	LCD_D14);

	createChar(0, sunShineLeft);
	createChar(1, sunCenter);
	createChar(2, sunShineRight);
	createChar(3, black);
	createChar(4, barEmpty);
	createChar(5, barEnd);
	createChar(6, thermometer);
	createChar(7, degree);

	begin(20, 4);

	write(0);
	write(1);
	write(2);
	setCursor(1, 2);
	write(6);
	setCursor(5, 2);
	write(7);
	write('C');

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_PWM_Start(buzzerPwmTimer, buzzerPwmChannel);
	HAL_ADC_Start_IT(&hadc1);

	for (int i = 0; i < TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM; i++) {
		temperatureSamplingArr[i] = 25; //((float) HAL_ADC_GetValue(&hadc1) * 22 / 273);
	}

	for (int i = 0; i < TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM; i++) {
		lightSamplingArr[i] = 30; //((float) HAL_ADC_GetValue(&hadc1) * 22 / 273);
	}

	/*char tmpp[10];
	 sprintf(tmpp, "%d\n", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0));
	 HAL_UART_Transmit(&huart2, (uint8_t*) tmpp, strlen(tmpp), HAL_MAX_DELAY);
	 HAL_Delay(100);
	 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/*char tmpp[10];
		 sprintf(tmpp, "%d\n", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0));
		 HAL_UART_Transmit(&huart2, (uint8_t*) tmpp, strlen(tmpp), HAL_MAX_DELAY);
		 HAL_Delay(100);*/
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_TIM8;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
	PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

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
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 39;
	hrtc.Init.SynchPrediv = 999;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 9999;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 14399;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */
	htim4.Instance->SR = 0;				// for timer to work properly form first time triggered
	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 47999;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 9999;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 47999;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 999;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 65535;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 14399;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 9999;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

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
	huart2.Init.BaudRate = 230400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void) {

	/* USER CODE BEGIN USB_Init 0 */

	/* USER CODE END USB_Init 0 */

	/* USER CODE BEGIN USB_Init 1 */

	/* USER CODE END USB_Init 1 */
	hpcd_USB_FS.Instance = USB;
	hpcd_USB_FS.Init.dev_endpoints = 8;
	hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_Init 2 */

	/* USER CODE END USB_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
	CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
	GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin | MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
	 LD7_Pin LD9_Pin LD10_Pin LD8_Pin
	 LD6_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD7_Pin | LD9_Pin
			| LD10_Pin | LD8_Pin | LD6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PD8 PD9 PD10 PD11
	 PD12 PD13 PD14 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12
			| GPIO_PIN_13 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PD0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		if (warning)
			return;

		HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

		if (logIdx < LOG_BUFFER_SIZE)
			logBuffer[logIdx++] = (log ) { rtcDate, rtcTime, motion };

		if (!warning) {
			HAL_ADC_Stop_IT(&hadc1);
			HAL_ADC_Stop_IT(&hadc2);
			warningOn();
			warning = true;
			HAL_TIM_Base_Start_IT(&htim4);
		}
	}
}

log logCache;
char tmpStrUart[56];
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		if (logIdx > 0) {
			for (int i = 0; i < logIdx; i++) {
				logCache = logBuffer[i];
				switch (logCache.type) {
				case motion:
					sprintf(tmpStrUart,
							"Motion detected! --- %02d\\%02d\\%02d - %02d:%02d:%02d      \n- - - -\n",
							logCache.date.Year, logCache.date.Month, logCache.date.Date,
							logCache.time.Hours, logCache.time.Minutes, logCache.time.Seconds);
					break;
				case temperatureRaise:
					sprintf(tmpStrUart,
							"Temperature Increased! --- %02d\\%02d\\%02d - %02d:%02d:%02d\n- - - -\n",
							logCache.date.Year, logCache.date.Month, logCache.date.Date,
							logCache.time.Hours, logCache.time.Minutes, logCache.time.Seconds);
					break;
				case lightHigh:
					sprintf(tmpStrUart,
							"Light is over 80%%! --- %02d\\%02d\\%02d - %02d:%02d:%02d    \n- - - -\n",
							logCache.date.Year, logCache.date.Month, logCache.date.Date,
							logCache.time.Hours, logCache.time.Minutes, logCache.time.Seconds);
					break;
				case lightLow:
					sprintf(tmpStrUart,
							"Light is under 20%%! --- %02d\\%02d\\%02d - %02d:%02d:%02d   \n- - - -\n",
							logCache.date.Year, logCache.date.Month, logCache.date.Date,
							logCache.time.Hours, logCache.time.Minutes, logCache.time.Seconds);
					break;
				}
				HAL_UART_Transmit(&huart2, (uint8_t*) tmpStrUart, 55, HAL_MAX_DELAY);
			}
			logIdx = 0;
		}
	} else if (htim->Instance == TIM7) {
		printDateTime();
	} else if (htim->Instance == TIM4) {
		warningOff();
		warning = false;
		HAL_TIM_Base_Stop_IT(&htim4);
		if (!buzzerOn)
			HAL_ADC_Start_IT(&hadc1);
	} else if (htim->Instance == TIM16) {
		buzzerChangeTone(1000, 0);
		HAL_Delay(50);
		buzzerOn = false;
		HAL_TIM_Base_Stop_IT(&htim16);
		if (!warning)
			HAL_ADC_Start_IT(&hadc1); // only this line for fixing ADC not starting again after temperature raise
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) { // lastTemperature
		if (buzzerOn)
		return;

		/*char tmpp[50];
		 sprintf(tmpp, "temperature ADC begin : %lu\n", HAL_GetTick());
		 HAL_UART_Transmit(&huart2, (uint8_t*) tmpp, strlen(tmpp), HAL_MAX_DELAY);
		 */
		temperatueRawValue = HAL_ADC_GetValue(hadc);

		char tmpp[50];
		sprintf(tmpp, "%d\n", temperatueRawValue);
		HAL_UART_Transmit(&huart2, (uint8_t*) tmpp, strlen(tmpp), HAL_MAX_DELAY);

		normalTemperature = ((float) temperatueRawValue * 22 / 273);

		temperatureSamplingSum += (normalTemperature
				- temperatureSamplingArr[temperatureSamplingIdx]);

		temperatureSamplingArr[temperatureSamplingIdx] = normalTemperature;

		//if (temperatureSamplingIdx == 0) {
			newTemperature = temperatureSamplingSum >> 7;

			if (newTemperature > lastTemperature) {
				HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
				if (logIdx < LOG_BUFFER_SIZE)
					logBuffer[logIdx++] = (log ) { rtcDate, rtcTime, temperatureRaise };
				if (!buzzerOn) {
					buzzerChangeTone(1000, 1000);
					buzzerOn = true;
					HAL_TIM_Base_Start_IT(&htim16);
				}
			}
			updateTemperature();
		//}

		temperatureSamplingIdx = (temperatureSamplingIdx + 1)
				% TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM;

		if (!buzzerOn)
		HAL_ADC_Start_IT(&hadc2);

	} else if (hadc->Instance == ADC2) { // light

		lightRawValue = HAL_ADC_GetValue(hadc);

		normalLight = (float) (lightRawValue / 5);	// simplified of (x - 0) * 100 / (600 - 0)

		lightSamplingSum += normalLight - lightSamplingArr[lightSamplingIdx];

		lightSamplingArr[lightSamplingIdx] = normalLight;

		//if (lightSamplingIdx == 0) {
			newLight = lightSamplingSum >> 7;
			newLight = newLight > 100 ? 100 : newLight;
			if (newLight > 80) {
				HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
				if (logIdx < LOG_BUFFER_SIZE)
					logBuffer[logIdx++] = (log ) { rtcDate, rtcTime, lightHigh };
			} else if (newLight < 20) {
				HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
				if (logIdx < LOG_BUFFER_SIZE)
					logBuffer[logIdx++] = (log ) { rtcDate, rtcTime, lightLow };
			}
			updateLight();
		//}
		/*
		 if (lightSamplingIdx == TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM) {
		 newLight = lightSamplingSum / TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM;
		 newLight = newLight > 100 ? 100 : newLight;
		 lightSamplingIdx = 0;
		 lightSamplingSum = 0;
		 updateLight();
		 if (newLight > 80) {
		 HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
		 HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
		 if (logIdx < LOG_BUFFER_SIZE)
		 logBuffer[logIdx++] = (log ) { rtcDate, rtcTime, lightHigh };
		 } else if (newLight < 20) {
		 HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
		 HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
		 if (logIdx < LOG_BUFFER_SIZE)
		 logBuffer[logIdx++] = (log ) { rtcDate, rtcTime, lightLow };
		 }
		 }
		 */
		lightSamplingIdx = (lightSamplingIdx + 1) % TEMPERATUER_LIGHT_FILTERING_SAMPLE_NUM;

		HAL_ADC_Start_IT(&hadc1);

		/*sprintf(tmpp, "light ADC end : %lu\n", HAL_GetTick());
		 HAL_UART_Transmit(&huart2, (uint8_t*) tmpp, strlen(tmpp), HAL_MAX_DELAY);*/
	}
}

char tmpStrDateTime[17];
void printDateTime() {
	HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);

	sprintf(tmpStrDateTime, "%d/%d/%d - %02d:%02d:%02d", rtcDate.Year, rtcDate.Month, rtcDate.Date,
			rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds);
	setCursor(2, 3);
	print(tmpStrDateTime);

}

char tmpStrLight[3];
void updateLight() {
	if (newLight == lastLight)
		return;

	sprintf(tmpStrLight, "%d%%", newLight < 100 ? newLight : 100);
	setCursor(15, 0);
	print(tmpStrLight);

	if (newLight / 10 == lastLight / 10)
		return;

	if (newLight < 10) {
		setCursor(17, 0);
		print(" ");
		createChar(3, barStart);
		setCursor(3, 0);
		write(3);
	} else {
		createChar(3, black);
		setCursor(3, 0);
		write(3);
	}

	if (lastLight == 100) {
		setCursor(18, 0);
		print(" ");
	}

	for (int i = 4; i < 12; i++) {
		if (i - 2 <= newLight / 10) {
			setCursor(i, 0);
			write(3);
		} else {
			setCursor(i, 0);
			write(4);
		}
	}

	if (newLight >= 100) {
		setCursor(12, 0);
		write(3);
	} else {
		setCursor(12, 0);
		write(5);
	}

	lastLight = newLight;
	HAL_Delay(50);
}

char tmpStrTemp[3];
void updateTemperature() {
	if (newTemperature == lastTemperature)
		return;

	sprintf(tmpStrTemp, "%02d", newTemperature);
	setCursor(3, 2);
	print(tmpStrTemp);

	lastTemperature = newTemperature;
}

void warningOn() {
	setCursor(9, 2);
	print("MOTION!!!");
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	//HAL_Delay(50);
}
void warningOff() {
	setCursor(9, 2);
	print("         ");
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	//HAL_Delay(50);
}

void buzzerChangeTone(uint16_t freq, uint16_t volume) {
	if (freq == 0 || freq > 20000) {
		__HAL_TIM_SET_COMPARE(buzzerPwmTimer, buzzerPwmChannel, 0);
	} else {
		const uint32_t internalClockFreq = HAL_RCC_GetSysClockFreq();
		const uint32_t prescaler = 1 + internalClockFreq / freq / 60000;
		const uint32_t timerClock = internalClockFreq / prescaler;
		const uint32_t periodCycles = timerClock / freq;
		const uint32_t pulseWidth = volume * periodCycles / 1000 / 2;

		buzzerPwmTimer->Instance->PSC = prescaler - 1;
		buzzerPwmTimer->Instance->ARR = periodCycles - 1;
		buzzerPwmTimer->Instance->EGR = TIM_EGR_UG;

		__HAL_TIM_SET_COMPARE(buzzerPwmTimer, buzzerPwmChannel, pulseWidth);
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
