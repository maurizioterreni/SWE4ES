/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fatfs_sd.h"
#include <I2CReader.h>
#include <SensorReaderFactory.h>
#include <WeatherData.h>

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

RTC_HandleTypeDef hrtc;

osThreadId sdTaskHandle;
osThreadId t25TaskHandle;
osThreadId httpTaskHandle;
osThreadId t30TaskHandle;
osThreadId temperatureTaskHandle;
osThreadId t3TaskHandle;
osThreadId t6TaskHandle;
osThreadId humidityTaskHandle;
osThreadId t11TaskHandle;
osThreadId t14TaskHandle;
osThreadId pressureTaskHandle;
osThreadId t18TaskHandle;
osThreadId t20TaskHandle;
/* USER CODE BEGIN PV */

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

SensorReaderFactory *sensorFactory;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

void startSdTask(void const * argument);
void startT25Task(void const * argument);
void startHttpTask(void const * argument);
void startT30Task(void const * argument);
void startTemperatureTask(void const * argument);
void startT3Task(void const * argument);
void startT6Task(void const * argument);
void startHumidityTask(void const * argument);
void startT11Task(void const * argument);
void startT14Task(void const * argument);
void startPressureTask(void const * argument);
void startT18Task(void const * argument);
void startT20Task(void const * argument);



int getDataString(char *buf, int size);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_FATFS_Init();
	//	MX_RTC_Init();
	/* USER CODE BEGIN 2 */




	sensorFactory = new SensorReaderFactory();

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */

	osThreadDef(presTask, startPressureTask, osPriorityRealtime 0, 256);
	pressureTaskHandle = osThreadCreate(osThread(pressureTask), NULL);

	osThreadDef(t18Task, startT18Task, osPriorityAboveNormal, 0, 256);
	t18TaskHandle = osThreadCreate(osThread(t18Task), NULL);

	osThreadDef(t20Task, startT20Task, osPriorityNormal, 0, 256);
	t20TaskHandle = osThreadCreate(osThread(t20Task), NULL);


	osThreadDef(humTask, startHumidityTask, osPriorityRealtime, 0, 256);
	humidityTaskHandle = osThreadCreate(osThread(humidtyTask), NULL);

	osThreadDef(t11Task, startT11Task, osPriorityAboveNormal, 0, 256);
	t11TaskHandle = osThreadCreate(osThread(t11Task), NULL);

	osThreadDef(t14Task, startT14Task, osPriorityNormal, 0, 256);
	t14TaskHandle = osThreadCreate(osThread(t14Task), NULL);

	osThreadDef(tempTask, startTemperatureTask, osPriorityRealtime, 0, 256);
	temperatureTaskHandle = osThreadCreate(osThread(temperatureTask), NULL);

	osThreadDef(t3Task, startT3Task, osPriorityAboveNormal, 0, 256);
	t3TaskHandle = osThreadCreate(osThread(t3Task), NULL);

	osThreadDef(t6Task, startT6Task, osPriorityNormal, 0, 256);
	t6TaskHandle = osThreadCreate(osThread(t6Task), NULL);


	osThreadDef(sdTask, startSdTask, osPriorityRealtime, 0, 256);
	sdTaskHandle = osThreadCreate(osThread(sdTask), NULL);

	osThreadDef(t25Task, startT25Task, osPriorityNormal, 0, 256);
	t25TaskHandle = osThreadCreate(osThread(t25Task), NULL);


	osThreadDef(httpTask, startHttpTask, osPriorityRealtime, 0, 256);
	httpTaskHandle = osThreadCreate(osThread(httpTask), NULL);

	osThreadDef(t30Task, startT30Task, osPriorityNormal, 0, 256);
	t30TaskHandle = osThreadCreate(osThread(t30Task), NULL);
	//
	//

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
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
	/** Initializes the CPU, AHB and APB buses clocks
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	hi2c1.Init.ClockSpeed = 400000;
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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void startTemperatureTask(void const * argument) {
	/* USER CODE BEGIN 5 */
	while(1) {
		xTaskNotify(t3TaskHandle, 0x0, eNoAction);
		xTaskNotify(t6TaskHandle, 0x0, eNoAction);
		osDelay(1000);
	}
	/* USER CODE END 5 */
}

void startT3Task(void const * argument) {
	TemperatureReader * reader = sensorFactory->createTemperatureReader();
	I2CReader::getInstance()->init(&hi2c1, reader);
	while(1) {
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE ) {
			Send_Uart("T0:",0);
			Send_Uart("T1:",0);
			Send_Uart("T2:",0);
			long old = HAL_GetTick();
			float value = I2CReader::getInstance()->getData(&hi2c1, reader);
			WeatherData::getInstance()->updateTemperature(value);
			Send_Uart("T3:",HAL_GetTick() - old);
		}
	}
}

void startT6Task(void const * argument) {
	while(1) {
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE ) {
			long old = HAL_GetTick();
			WeatherData::getInstance()->calculateData();
		}
	}
}


void startHumidityTask(void const * argument) {
	/* USER CODE BEGIN 5 */
	while(1) {
		xTaskNotify(t14TaskHandle, 0x0, eNoAction);
		osDelay(1000);
	}
	/* USER CODE END 5 */
}

void startT11Task(void const * argument) {
	HumidityReader *reader = sensorFactory->createHumidityReader();
	I2CReader::getInstance()->init(&hi2c1, reader);
	while(1) {
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE ) {
			long old = HAL_GetTick();
			float value = I2CReader::getInstance()->getData(&hi2c1, reader);
			WeatherData::getInstance()->updateHumidity(value);
		}
	}
}

void startT14Task(void const * argument) {
	while(1) {
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE ) {
			long old = HAL_GetTick();
			WeatherData::getInstance()->calculateData();
		}
	}
}

void startPressureTask(void const * argument) {
	/* USER CODE BEGIN 5 */
	while(1) {
		xTaskNotify(t18TaskHandle, 0x0, eNoAction);
		xTaskNotify(t20TaskHandle, 0x0, eNoAction);
		osDelay(1000);
	}
	/* USER CODE END 5 */
}

void startT18Task(void const * argument) {
	PressureReader *reader = sensorFactory->createPressureReader();
	I2CReader::getInstance()->init(&hi2c1, reader);
	while(1) {
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE ) {
			long old = HAL_GetTick();
			float value = I2CReader::getInstance()->getData(&hi2c1, reader);
			WeatherData::getInstance()->updatePressure(value);
		}
	}
}

void startT20Task(void const * argument) {
	while(1) {
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE ) {
			long old = HAL_GetTick();
			WeatherData::getInstance()->calculateData();
		}
	}
}

void startSdTask(void const * argument) {
	while(1) {
		Send_Uart("task04\n");
		xTaskNotify(t25TaskHandle, 0x0, eNoAction);
		osDelay(60000);
	}
}


void startT25Task(void const * argument) {
	char buf[100];
	fresult = f_mount(&fs, "/", 1);
	while(1) {
		fresult = f_open(&fil, "wth.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

		fresult = f_lseek(&fil, f_size(&fil));

		if (fresult == FR_OK) {
			WeatherData::getInstance()->getDataString(buf, sizeof(buf));
			f_puts(buf, &fil);
		}


		f_close (&fil);
	}
}

void startHttpTask(void const * argument) {
	while(1) {
		xTaskNotify(t30TaskHandle, 0x0, eNoAction);
		osDelay(60000);
	}
}


void startT30Task(void const * argument) {
	while(1) {
		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE ) {
			char buf[100];
			long old = HAL_GetTick();
			int len = WeatherData::getInstance()->getDataString(buf, sizeof(buf));
			HAL_UART_Transmit(&huart3, (uint8_t *) buf, len, 100);
		}
	}
}

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


int getDataString(char *buf, int size) {
	return snprintf(buf, size, "10.0,1000,50");
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
