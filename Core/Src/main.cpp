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

osThreadId tempTaskHandle;
osThreadId humTaskHandle;
osThreadId pressTaskHandle;
osThreadId sdTaskHandle;
osThreadId httpTaskHandle;


//TEMP TASK
osThreadId t1TaskHandle;
osThreadId t2TaskHandle;
osThreadId t3TaskHandle;
osThreadId t4TaskHandle;
osThreadId t5TaskHandle;
osThreadId t6TaskHandle;




//HUM TASK
osThreadId t8TaskHandle;
osThreadId t9TaskHandle;
osThreadId t10TaskHandle;
osThreadId t11TaskHandle;
osThreadId t12TaskHandle;
osThreadId t13TaskHandle;

//Press TASK
osThreadId t15TaskHandle;
osThreadId t16TaskHandle;
osThreadId t17TaskHandle;
osThreadId t18TaskHandle;


//SD TASK
osThreadId t20TaskHandle;
osThreadId t21TaskHandle;
osThreadId t22TaskHandle;


//HttpTask
osThreadId t24TaskHandle;
osThreadId t25TaskHandle;
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
void startHttpTask(void const * argument);
void startTemperatureTask(void const * argument);
void startHumidityTask(void const * argument);
void startPressureTask(void const * argument);


//TEMP TASK
void startT1Task(void const * argument);
void startT2Task(void const * argument);
void startT3Task(void const * argument);
void startT4Task(void const * argument);
void startT5Task(void const * argument);
void startT6Task(void const * argument);


//HUM TASK
void startT8Task(void const * argument);
void startT9Task(void const * argument);
void startT10Task(void const * argument);
void startT11Task(void const * argument);
void startT12Task(void const * argument);
void startT13Task(void const * argument);


//Press TASK
void startT15Task(void const * argument);
void startT16Task(void const * argument);
void startT17Task(void const * argument);
void startT18Task(void const * argument);



//SD Task
void startT20Task(void const * argument);
void startT21Task(void const * argument);
void startT22Task(void const * argument);


//Http Task


void startT24Task(void const * argument);
void startT25Task(void const * argument);


int getDataString(char *buf, int size);


void Send_Uart (char *string);
void Send_Uart (char *string, long val);
void Send_Uart (int id, long val);

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
	osThreadDef(tempTask, startTemperatureTask, osPriorityRealtime, 0, 128);
	tempTaskHandle = osThreadCreate(osThread(tempTask), NULL);


	osThreadDef(t1Task, startT1Task, osPriorityLow, 0, 128); //P1
	t1TaskHandle = osThreadCreate(osThread(t1Task), NULL);
	vTaskSuspend(t1TaskHandle);


	osThreadDef(t2Task, startT2Task, osPriorityNormal, 0, 128);  //P3
	t2TaskHandle = osThreadCreate(osThread(t2Task), NULL);
	vTaskSuspend(t2TaskHandle);


	osThreadDef(t3Task, startT3Task, osPriorityNormal, 0, 128); //P3
	t3TaskHandle = osThreadCreate(osThread(t3Task), NULL);
	vTaskSuspend(t3TaskHandle);


	osThreadDef(t4Task, startT4Task, osPriorityLow, 0, 128); //P1
	t4TaskHandle = osThreadCreate(osThread(t4Task), NULL);
	vTaskSuspend(t4TaskHandle);


	osThreadDef(t5Task, startT5Task, osPriorityHigh, 0, 128); //5
	t5TaskHandle = osThreadCreate(osThread(t5Task), NULL);
	vTaskSuspend(t5TaskHandle);


	osThreadDef(t6Task, startT6Task, osPriorityHigh, 0, 128); //P5
	t6TaskHandle = osThreadCreate(osThread(t6Task), NULL);
	vTaskSuspend(t6TaskHandle);


	//	/* definition and creation of humTask */
	osThreadDef(humTask, startHumidityTask, osPriorityRealtime, 0, 128);
	humTaskHandle = osThreadCreate(osThread(humTask), NULL);


	osThreadDef(t8Task, startT8Task, osPriorityBelowNormal, 0, 128); //P2
	t8TaskHandle = osThreadCreate(osThread(t8Task), NULL);
	vTaskSuspend(t8TaskHandle);


	osThreadDef(t9Task, startT9Task, osPriorityNormal, 0, 128); //P3
	t9TaskHandle = osThreadCreate(osThread(t9Task), NULL);
	vTaskSuspend(t9TaskHandle);


	osThreadDef(t10Task, startT10Task, osPriorityNormal, 0, 128); //P3
	t10TaskHandle = osThreadCreate(osThread(t10Task), NULL);
	vTaskSuspend(t10TaskHandle);


	osThreadDef(t11Task, startT11Task, osPriorityBelowNormal, 0, 128); //P2
	t11TaskHandle = osThreadCreate(osThread(t11Task), NULL);
	vTaskSuspend(t11TaskHandle);


	osThreadDef(t12Task, startT12Task, osPriorityHigh, 0, 128); //P5
	t12TaskHandle = osThreadCreate(osThread(t12Task), NULL);
	vTaskSuspend(t12TaskHandle);


	osThreadDef(t13Task, startT13Task, osPriorityHigh, 0, 128); //P5
	t13TaskHandle = osThreadCreate(osThread(t13Task), NULL);
	vTaskSuspend(t13TaskHandle);
	//
	//	//
	//	//	/* definition and creation of pressTask */
	osThreadDef(pressTask, startPressureTask, osPriorityRealtime, 0, 128);
	pressTaskHandle = osThreadCreate(osThread(pressTask), NULL);


	osThreadDef(t15Task, startT15Task, osPriorityNormal, 0, 128); //P3
	t15TaskHandle = osThreadCreate(osThread(t15Task), NULL);
	vTaskSuspend(t15TaskHandle);


	osThreadDef(t16Task, startT16Task, osPriorityNormal, 0, 128); //P3
	t16TaskHandle = osThreadCreate(osThread(t16Task), NULL);
	vTaskSuspend(t16TaskHandle);


	osThreadDef(t17Task, startT17Task, osPriorityHigh, 0, 128); //P5
	t17TaskHandle = osThreadCreate(osThread(t17Task), NULL);
	vTaskSuspend(t17TaskHandle);


	osThreadDef(t18Task, startT18Task, osPriorityHigh, 0, 128);  //P6
	t18TaskHandle = osThreadCreate(osThread(t18Task), NULL);
	vTaskSuspend(t18TaskHandle);

	//
	//	//
	//	//	/* definition and creation of sdTask */
	osThreadDef(sdTask, startSdTask, osPriorityRealtime, 0, 128);
	sdTaskHandle = osThreadCreate(osThread(sdTask), NULL);


	osThreadDef(t20Task, startT20Task, osPriorityAboveNormal, 0, 128); //P4
	t20TaskHandle = osThreadCreate(osThread(t20Task), NULL);
	vTaskSuspend(t20TaskHandle);


	osThreadDef(t21Task, startT21Task, osPriorityHigh, 0, 128); //P5
	t21TaskHandle = osThreadCreate(osThread(t21Task), NULL);
	vTaskSuspend(t21TaskHandle);


	osThreadDef(t22Task, startT22Task, osPriorityHigh, 0, 128); //P5
	t22TaskHandle = osThreadCreate(osThread(t22Task), NULL);
	vTaskSuspend(t22TaskHandle);





	//
	//	//
	osThreadDef(httpTask, startHttpTask, osPriorityRealtime, 0, 128);
	httpTaskHandle = osThreadCreate(osThread(httpTask), NULL);


	osThreadDef(t24Task, startT24Task, osPriorityHigh, 0, 128);
	t24TaskHandle = osThreadCreate(osThread(t24Task), NULL);
	vTaskSuspend(t24TaskHandle);


	osThreadDef(t25Task, startT25Task, osPriorityHigh, 0, 128);
	t25TaskHandle = osThreadCreate(osThread(t25Task), NULL);
	vTaskSuspend(t25TaskHandle);

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

void Send_Uart (char *string) {
	HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen (string), 100);
}

void Send_Uart (char *string, long val) {
	char buf[100];
	snprintf(buf, 100, "%s%lu\n\r",
			string,
			val);
	Send_Uart(buf);
}


void Send_Uart (int id, long val) {
	char buf[100];
	snprintf(buf, 100, "t%d:%lu\n\r",
			id,
			val);
	Send_Uart(buf);
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
		osDelay(1000);
		Send_Uart(0,0);
		vTaskResume(t1TaskHandle);
	}
	/* USER CODE END 5 */
}


void startT1Task(void const * argument) {
	while(1) {
		Send_Uart(1, 0);
		vTaskResume(t2TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT2Task(void const * argument) {
	while(1) {
		Send_Uart(2, 0);
		I2CReader::getInstance()->wait();
		vTaskResume(t3TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT3Task(void const * argument) {
	TemperatureReader * reader = sensorFactory->createTemperatureReader();
	I2CReader::getInstance()->init(&hi2c1, reader);
	while(1) {
		long old = HAL_GetTick();
		float value = I2CReader::getInstance()->getData(&hi2c1, reader);
		WeatherData::getInstance()->updateTemperature(value);
		I2CReader::getInstance()->release();
		Send_Uart(3, HAL_GetTick() - old);
		vTaskResume(t4TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT4Task(void const * argument) {
	while(1) {
		Send_Uart(4, 0);
		vTaskResume(t5TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT5Task(void const * argument) {
	while(1) {
		Send_Uart(5, 0);
		WeatherData::getInstance()->semaphoreWait();
		vTaskResume(t6TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT6Task(void const * argument) {
	while(1) {
		long old = HAL_GetTick();
		WeatherData::getInstance()->calculateData();
		WeatherData::getInstance()->semaphoreRelease();
		Send_Uart(6, HAL_GetTick() - old);
		vTaskSuspend(NULL);
	}
}

void startHumidityTask(void const * argument) {
	while(1) {
		osDelay(1000);
		Send_Uart(7, 0);
		vTaskResume(t8TaskHandle);
	}
}


void startT8Task(void const * argument) {
	while(1) {
		Send_Uart(8, 0);
		vTaskResume(t9TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT9Task(void const * argument) {
	while(1) {
		Send_Uart(9, 0);
		I2CReader::getInstance()->wait();
		vTaskResume(t10TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT10Task(void const * argument) {
	HumidityReader *reader = sensorFactory->createHumidityReader();
	I2CReader::getInstance()->init(&hi2c1, reader);
	while(1) {
		long old = HAL_GetTick();
		float value = I2CReader::getInstance()->getData(&hi2c1, reader);
		WeatherData::getInstance()->updateHumidity(value);
		I2CReader::getInstance()->release();
		Send_Uart(10, HAL_GetTick() - old);
		vTaskResume(t11TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT11Task(void const * argument) {
	while(1) {
		Send_Uart(11, 0);
		vTaskResume(t12TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT12Task(void const * argument) {
	while(1) {
		Send_Uart(12, 0);
		WeatherData::getInstance()->semaphoreWait();
		vTaskResume(t13TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT13Task(void const * argument) {
	while(1) {
		long old = HAL_GetTick();
		WeatherData::getInstance()->calculateData();
		WeatherData::getInstance()->semaphoreRelease();
		Send_Uart(10, HAL_GetTick() - old);
		vTaskSuspend(NULL);
	}
}


void startPressureTask(void const * argument) {
	while(1) {
		osDelay(1000);
		Send_Uart(14, 0);
		vTaskResume(t15TaskHandle);
	}
}


void startT15Task(void const * argument) {
	while(1) {
		Send_Uart(15, 0);
		I2CReader::getInstance()->wait();
		vTaskResume(t16TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT16Task(void const * argument) {
	PressureReader *reader = sensorFactory->createPressureReader();
	I2CReader::getInstance()->init(&hi2c1, reader);
	while(1) {
		long old = HAL_GetTick();
		float value = I2CReader::getInstance()->getData(&hi2c1, reader);
		WeatherData::getInstance()->updatePressure(value);
		I2CReader::getInstance()->release();
		Send_Uart(16, HAL_GetTick() - old);
		vTaskResume(t17TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT17Task(void const * argument) {
	while(1) {
		Send_Uart(17, 0);
		WeatherData::getInstance()->semaphoreWait();
		vTaskResume(t18TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT18Task(void const * argument) {
	while(1) {
		long old = HAL_GetTick();
		WeatherData::getInstance()->calculateData();
		WeatherData::getInstance()->semaphoreRelease();
		Send_Uart(18, HAL_GetTick());
		vTaskSuspend(NULL);
	}
}


void startSdTask(void const * argument) {
	while(1) {
		osDelay(60000);
		Send_Uart(19,0);
		vTaskResume(t20TaskHandle);
	}
}


void startT20Task(void const * argument) {
	while(1) {
		Send_Uart(20, 0);
		vTaskResume(t21TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT21Task(void const * argument) {
	while(1) {
		Send_Uart(21, 0);
		WeatherData::getInstance()->semaphoreWait();
		vTaskResume(t22TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT22Task(void const * argument) {
	char buf[100];
	fresult = f_mount(&fs, "/", 1);
	while(1) {
		long old = HAL_GetTick();
		fresult = f_open(&fil, "wth.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

		fresult = f_lseek(&fil, f_size(&fil));

		if (fresult == FR_OK) {
			WeatherData::getInstance()->getDataString(buf, sizeof(buf));
			f_puts(buf, &fil);
		}


		f_close (&fil);
		WeatherData::getInstance()->semaphoreRelease();
		Send_Uart(22, HAL_GetTick() - old);
		vTaskSuspend(NULL);
	}
}


void startHttpTask(void const * argument) {
	while(1) {
		osDelay(60000);
		Send_Uart(23,0);
		vTaskResume(t24TaskHandle);
	}
}

void startT24Task(void const * argument) {
	while(1) {
		Send_Uart(24, 0);
		WeatherData::getInstance()->semaphoreWait();
		vTaskResume(t25TaskHandle);
		vTaskSuspend(NULL);
	}
}

void startT25Task(void const * argument) {
	while(1) {
		long old = HAL_GetTick();
		char buf[100];
		int len = WeatherData::getInstance()->getDataString(buf, sizeof(buf));
		HAL_UART_Transmit(&huart3, (uint8_t *) buf, len, 100);
		WeatherData::getInstance()->semaphoreRelease();
		Send_Uart(25, HAL_GetTick() - old);
		vTaskSuspend(NULL);
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
