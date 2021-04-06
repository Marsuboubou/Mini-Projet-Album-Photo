/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main3.c
 * @brief          : Main du TP3
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "time.h"
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
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

SDRAM_HandleTypeDef hsdram1;

osThreadId defaultTaskHandle;
osThreadId task_clockHandle;
osThreadId task_Bip_BipHandle;
osThreadId task_Vil_CoyoteHandle;
osThreadId task_Enclume_1Handle;
osThreadId task_Enclume_2Handle;
osThreadId task_Vil_WinHandle;
osThreadId task_BipBip_WinHandle;
osThreadId task_Gest_EncluHandle;
osMessageQId QueueU2BHandle;
osMessageQId QueueVWHandle;
osMessageQId QueueBWHandle;
osMessageQId QueueGE2EHandle;
osMutexId Mutex_LCDHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_UART7_Init(void);
static void MX_FMC_Init(void);
static void MX_DMA2D_Init(void);
void StartDefaultTask(void const *argument);
void clock_LCD(void const *argument);
void Bip_Bip(void const *argument);
void Vil_Coyote(void const *argument);
void Enclume_1(void const *argument);
void Enclume_2(void const *argument);
void Vil_Winner(void const *argument);
void BipBip_Winner(void const *argument);
void Gestion_Enclume(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rxbuffer[10]; //Buffer pour récupérer les variables venant de l'USART
int etat_led12 = 0; //Etat de la LED12 pour temoin interruption sur le bouton du joystick
int Position_Enclume_1[2] = { 0, 20 }; //Blackboard pour la position de l'enclume 1
int Position_Enclume_2[2] = { 0, 20 }; //Blackboard pour la position de l'enclume 2
int etat_enclumes[2] = { 0, 0 }; //Blackboard pour avoir l'état des enclumes (1 si enclume utilisée)
int Position_Bip_Bip[2] = { 250, 242 }; //Blackboard pour la position de Bip-Bip
int Position_Vil_Coyote[2] = { 0, 20 }; //Blackboard pour la position de Vil_Coyote
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
	MX_ADC3_Init();
	MX_I2C1_Init();
	MX_I2C3_Init();
	MX_LTDC_Init();
	MX_RTC_Init();
	MX_SPI2_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_USART1_UART_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	MX_DAC_Init();
	MX_UART7_Init();
	MX_FMC_Init();
	MX_DMA2D_Init();
	/* USER CODE BEGIN 2 */
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_LayerDefaultInit(1,
	LCD_FB_START_ADDRESS + BSP_LCD_GetXSize() * BSP_LCD_GetYSize() * 4);
	BSP_LCD_DisplayOn();
	BSP_LCD_SelectLayer(1);
	BSP_LCD_Clear(LCD_COLOR_ORANGE);
	BSP_LCD_SetFont(&Font12);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_SetBackColor(LCD_COLOR_ORANGE);

	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

	HAL_UART_Receive_IT(&huart1, rxbuffer, 1);
	HAL_GPIO_WritePin(LED18_GPIO_Port, LED18_Pin, 0); //LED18 pour état du jeu

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of Mutex_LCD */
	osMutexDef(Mutex_LCD);
	Mutex_LCDHandle = osMutexCreate(osMutex(Mutex_LCD));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of QueueU2B */
	osMessageQDef(QueueU2B, 2, uint8_t);
	QueueU2BHandle = osMessageCreate(osMessageQ(QueueU2B), NULL);

	/* definition and creation of QueueVW */
	osMessageQDef(QueueVW, 2, uint8_t);
	QueueVWHandle = osMessageCreate(osMessageQ(QueueVW), NULL);

	/* definition and creation of QueueBW */
	osMessageQDef(QueueBW, 2, uint8_t);
	QueueBWHandle = osMessageCreate(osMessageQ(QueueBW), NULL);

	/* definition and creation of QueueGE2E */
	osMessageQDef(QueueGE2E, 2, uint8_t);
	QueueGE2EHandle = osMessageCreate(osMessageQ(QueueGE2E), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 512);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of task_clock */
	osThreadDef(task_clock, clock_LCD, osPriorityBelowNormal, 0, 1024);
	task_clockHandle = osThreadCreate(osThread(task_clock), NULL);

	/* definition and creation of task_Bip_Bip */
	osThreadDef(task_Bip_Bip, Bip_Bip, osPriorityHigh, 0, 1024);
	task_Bip_BipHandle = osThreadCreate(osThread(task_Bip_Bip), NULL);

	/* definition and creation of task_Vil_Coyote */
	osThreadDef(task_Vil_Coyote, Vil_Coyote, osPriorityNormal, 0, 1024);
	task_Vil_CoyoteHandle = osThreadCreate(osThread(task_Vil_Coyote), NULL);

	/* definition and creation of task_Enclume_1 */
	osThreadDef(task_Enclume_1, Enclume_1, osPriorityLow, 0, 1024);
	task_Enclume_1Handle = osThreadCreate(osThread(task_Enclume_1), NULL);

	/* definition and creation of task_Enclume_2 */
	osThreadDef(task_Enclume_2, Enclume_2, osPriorityLow, 0, 1024);
	task_Enclume_2Handle = osThreadCreate(osThread(task_Enclume_2), NULL);

	/* definition and creation of task_Vil_Win */
	osThreadDef(task_Vil_Win, Vil_Winner, osPriorityHigh, 0, 512);
	task_Vil_WinHandle = osThreadCreate(osThread(task_Vil_Win), NULL);

	/* definition and creation of task_BipBip_Win */
	osThreadDef(task_BipBip_Win, BipBip_Winner, osPriorityHigh, 0, 512);
	task_BipBip_WinHandle = osThreadCreate(osThread(task_BipBip_Win), NULL);

	/* definition and creation of task_Gest_Enclu */
	osThreadDef(task_Gest_Enclu, Gestion_Enclume, osPriorityIdle, 0, 128);
	task_Gest_EncluHandle = osThreadCreate(osThread(task_Gest_Enclu), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC
			| RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART6
			| RCC_PERIPHCLK_UART7 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C3;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
	PeriphClkInitStruct.PLLSAIDivQ = 1;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
	PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
	PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
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

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void) {

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void) {

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */
	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		Error_Handler();
	}
	/** DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

}

/**
 * @brief DMA2D Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA2D_Init(void) {

	/* USER CODE BEGIN DMA2D_Init 0 */

	/* USER CODE END DMA2D_Init 0 */

	/* USER CODE BEGIN DMA2D_Init 1 */

	/* USER CODE END DMA2D_Init 1 */
	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0;
	if (HAL_DMA2D_Init(&hdma2d) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DMA2D_Init 2 */

	/* USER CODE END DMA2D_Init 2 */

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
	hi2c1.Init.Timing = 0x00C0EAFF;
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
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
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
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00C0EAFF;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void) {

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerCfgTypeDef pLayerCfg = { 0 };

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 40;
	hltdc.Init.VerticalSync = 9;
	hltdc.Init.AccumulatedHBP = 53;
	hltdc.Init.AccumulatedVBP = 11;
	hltdc.Init.AccumulatedActiveW = 533;
	hltdc.Init.AccumulatedActiveH = 283;
	hltdc.Init.TotalWidth = 565;
	hltdc.Init.TotalHeigh = 285;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
		Error_Handler();
	}
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 480;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 272;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
	pLayerCfg.Alpha = 255;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
	pLayerCfg.FBStartAdress = 0xC0000000;
	pLayerCfg.ImageWidth = 480;
	pLayerCfg.ImageHeight = 272;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */

	/* USER CODE END LTDC_Init 2 */

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
	RTC_AlarmTypeDef sAlarm = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
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
	/** Enable the Alarm A
	 */
	sAlarm.AlarmTime.Hours = 0x0;
	sAlarm.AlarmTime.Minutes = 0x0;
	sAlarm.AlarmTime.Seconds = 0x0;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/** Enable the Alarm B
	 */
	sAlarm.Alarm = RTC_ALARM_B;
	if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/** Enable the TimeStamp
	 */
	if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING,
	RTC_TIMESTAMPPIN_POS1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

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
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART7_Init(void) {

	/* USER CODE BEGIN UART7_Init 0 */

	/* USER CODE END UART7_Init 0 */

	/* USER CODE BEGIN UART7_Init 1 */

	/* USER CODE END UART7_Init 1 */
	huart7.Instance = UART7;
	huart7.Init.BaudRate = 115200;
	huart7.Init.WordLength = UART_WORDLENGTH_8B;
	huart7.Init.StopBits = UART_STOPBITS_1;
	huart7.Init.Parity = UART_PARITY_NONE;
	huart7.Init.Mode = UART_MODE_TX_RX;
	huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart7.Init.OverSampling = UART_OVERSAMPLING_16;
	huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart7) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART7_Init 2 */

	/* USER CODE END UART7_Init 2 */

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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void) {

	/* USER CODE BEGIN FMC_Init 0 */

	/* USER CODE END FMC_Init 0 */

	FMC_SDRAM_TimingTypeDef SdramTiming = { 0 };

	/* USER CODE BEGIN FMC_Init 1 */

	/* USER CODE END FMC_Init 1 */

	/** Perform the SDRAM1 memory initialization sequence
	 */
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 16;
	SdramTiming.ExitSelfRefreshDelay = 16;
	SdramTiming.SelfRefreshTime = 16;
	SdramTiming.RowCycleDelay = 16;
	SdramTiming.WriteRecoveryTime = 16;
	SdramTiming.RPDelay = 16;
	SdramTiming.RCDDelay = 16;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN FMC_Init 2 */

	/* USER CODE END FMC_Init 2 */
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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, LED14_Pin | LED15_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED16_GPIO_Port, LED16_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_DISP_GPIO_Port, LCD_DISP_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH,
	LED13_Pin | LED17_Pin | LED11_Pin | LED12_Pin | LED2_Pin | LED18_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(EXT_RST_GPIO_Port, EXT_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PE3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D2_Pin
	 ULPI_D1_Pin ULPI_D4_Pin */
	GPIO_InitStruct.Pin = ULPI_D7_Pin | ULPI_D6_Pin | ULPI_D5_Pin | ULPI_D2_Pin
			| ULPI_D1_Pin | ULPI_D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : BP2_Pin BP1_Pin */
	GPIO_InitStruct.Pin = BP2_Pin | BP1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LED14_Pin LED15_Pin */
	GPIO_InitStruct.Pin = LED14_Pin | LED15_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_VBUS_Pin */
	GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Audio_INT_Pin */
	GPIO_InitStruct.Pin = Audio_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin LED16_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin | LED16_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : LED3_Pin LCD_DISP_Pin */
	GPIO_InitStruct.Pin = LED3_Pin | LCD_DISP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pin : uSD_Detect_Pin */
	GPIO_InitStruct.Pin = uSD_Detect_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_BL_CTRL_Pin */
	GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : TP3_Pin NC2_Pin */
	GPIO_InitStruct.Pin = TP3_Pin | NC2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : LED13_Pin LED17_Pin LED11_Pin LED12_Pin
	 LED2_Pin LED18_Pin */
	GPIO_InitStruct.Pin = LED13_Pin | LED17_Pin | LED11_Pin | LED12_Pin
			| LED2_Pin | LED18_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_INT_Pin */
	GPIO_InitStruct.Pin = LCD_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ULPI_NXT_Pin */
	GPIO_InitStruct.Pin = ULPI_NXT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BP_JOYSTICK_Pin */
	GPIO_InitStruct.Pin = BP_JOYSTICK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BP_JOYSTICK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
	GPIO_InitStruct.Pin = ULPI_STP_Pin | ULPI_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : EXT_RST_Pin */
	GPIO_InitStruct.Pin = EXT_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(EXT_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_RXER_Pin */
	GPIO_InitStruct.Pin = RMII_RXER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
	GPIO_InitStruct.Pin = ULPI_CLK_Pin | ULPI_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 8, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uint8_t Message[2];
	if (rxbuffer[0] == 'a')
		HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 1);
	if (rxbuffer[0] == 'e')
		HAL_GPIO_WritePin(LED11_GPIO_Port, LED11_Pin, 0);
	if ((rxbuffer[0] == 'q') || (rxbuffer[0] == 'd') || (rxbuffer[0] == 'Q')
			|| (rxbuffer[0] == 'D')) {
		Message[0] = rxbuffer[0];
		xQueueSendFromISR(QueueU2BHandle, &Message, 0);
	}
	HAL_UART_Receive_IT(&huart1, rxbuffer, 1);
}

//Fonction d'interruption sur le BP_Joystick
void EXTI9_5_IRQHandler(void) {
	uint8_t Message2Enclumes[2];
	etat_led12 != etat_led12;
	HAL_GPIO_WritePin(LED12_GPIO_Port, LED12_Pin, etat_led12); //On vérifie que l'interruption marche avec témoin led
	Message2Enclumes[0] = 1;
	//Envoie du message d'activation
	xQueueSendFromISR(QueueGE2EHandle, &Message2Enclumes, 0); //Envoie un message à la tâche de gestion des enclumes
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_clock_LCD */
/**
 * @brief Function implementing the task_clock thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_clock_LCD */
void clock_LCD(void const *argument) {
	/* USER CODE BEGIN clock_LCD */
	//Variables pour l'affichage de l'heure
	char text_heure[25] = { };
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	char MessageBipBipWin[2];

	//Variables pour la période de la tâche
	TickType_t zLastWakeTime;
	const TickType_t zTimeincrement = 100; //Définition de la période
	zLastWakeTime = xTaskGetTickCount(); //On initialise le dernier réveil

	/* Infinite loop */
	for (;;) {
		//Période de 100ms
		vTaskDelayUntil(&zLastWakeTime, zTimeincrement);
		//Affichage du temps
		HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
		sprintf(text_heure, "Temps de jeu : %2d:%2d", time.Minutes,
				time.Seconds);
		//Partage des ressources avec un MUTEX
		if ( xSemaphoreTake(Mutex_LCDHandle,
				(TickType_t ) 10) == pdTRUE) {
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			BSP_LCD_DisplayStringAt(0, 5, (uint8_t*) text_heure, RIGHT_MODE);
			xSemaphoreGive(Mutex_LCDHandle);
		}
		taskENTER_CRITICAL();
		if (time.Minutes >= 2) {
			MessageBipBipWin[0] = 'W'; //BipBip gagne automatiquement au bout de 2 minutes
			xQueueSend(QueueBWHandle, &MessageBipBipWin, 0);
		}
		taskEXIT_CRITICAL();
	}
	/* USER CODE END clock_LCD */
}

/* USER CODE BEGIN Header_Bip_Bip */
/**
 * @brief Function implementing the task_Bip_Bip thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Bip_Bip */
void Bip_Bip(void const *argument) {
	/* USER CODE BEGIN Bip_Bip */
	//Position de Bip-Bip
	uint8_t Message[2];
	int X_Bip_Bip = Position_Bip_Bip[0];
	int Y_Bip_Bip = Position_Bip_Bip[1];
	int X_Bip_Bip_old = X_Bip_Bip;
	int Y_Bip_Bip_old = Y_Bip_Bip;
	int change = 0;
	//Taille de Bip-Bip
	uint16_t hauteur = 30;
	uint16_t largeur = 10;
	//Vitesse de déplacement de Bip-Bip
	int v_bip_bip = 2;

	//Variables pour la collision
	uint8_t MessageCollision[2];

	//Initialisation de Bip-Bip
	if ( xSemaphoreTake(Mutex_LCDHandle,
			(TickType_t ) 10) == pdTRUE) {
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE); //Dessine un rectangle bleu pour Bip-Bip
		BSP_LCD_FillRect(X_Bip_Bip, Y_Bip_Bip, largeur, hauteur);
		xSemaphoreGive(Mutex_LCDHandle);
	}

	/* Infinite loop */
	for (;;) {
		//Récupération du message de la queue
		xQueueReceive(QueueU2BHandle, &Message, portMAX_DELAY);
		if ((Message[0] == 'q'))
			X_Bip_Bip -= v_bip_bip;
		else if ((Message[0] == 'd'))
			X_Bip_Bip += v_bip_bip;
		else if ((Message[0] == 'D'))
			X_Bip_Bip += v_bip_bip * 2;
		else if ((Message[0] == 'Q'))
			X_Bip_Bip -= v_bip_bip * 2;

		//Prise en compte des bords de l'écran
		if (X_Bip_Bip < 0)
			X_Bip_Bip = 0;
		if ((X_Bip_Bip + largeur) > 480)
			X_Bip_Bip = (480 - largeur);

		//On regarde si Bip-Bip a changé de position
		if ((X_Bip_Bip != X_Bip_Bip_old) || (Y_Bip_Bip != Y_Bip_Bip_old))
			change = 1;
		else
			change = 0;

		//On écrit dans le blackboard
		taskENTER_CRITICAL();
		Position_Bip_Bip[0] = X_Bip_Bip;
		Position_Bip_Bip[1] = Y_Bip_Bip;
		taskEXIT_CRITICAL();

		//On vérifie s'il y a collision
		if (((X_Bip_Bip < (Position_Enclume_1[0] + 20))
				&& ((X_Bip_Bip + largeur) > Position_Enclume_1[0])
				&& (Y_Bip_Bip < (Position_Enclume_1[1] + 20))
				&& ((Y_Bip_Bip + hauteur) > Position_Enclume_1[1]))
				|| ((X_Bip_Bip < (Position_Enclume_2[0] + 20))
						&& ((X_Bip_Bip + largeur) > Position_Enclume_2[0])
						&& (Y_Bip_Bip < (Position_Enclume_2[1] + 20))
						&& ((Y_Bip_Bip + hauteur) > Position_Enclume_2[1]))) {
			MessageCollision[0] = 'C';
			xQueueSend(QueueVWHandle, &MessageCollision, 0);
		}

		//Affichage de Bip-Bip

		if ( xSemaphoreTake(Mutex_LCDHandle,
				(TickType_t ) 10) == pdTRUE) {
			//Efface l'ancien Bip_Bip
			if (change == 1) {
				BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
				BSP_LCD_FillRect(X_Bip_Bip_old, Y_Bip_Bip_old, largeur,
						hauteur);
			}
			X_Bip_Bip_old = X_Bip_Bip;
			Y_Bip_Bip_old = Y_Bip_Bip;
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE); //Dessine Bip-Bip
			BSP_LCD_FillRect(X_Bip_Bip, Y_Bip_Bip, largeur, hauteur);
			xSemaphoreGive(Mutex_LCDHandle);
		}
		osDelay(50); //On a une période de 50 ms
	}
	/* USER CODE END Bip_Bip */
}

/* USER CODE BEGIN Header_Vil_Coyote */
/**
 * @brief Function implementing the task_Vil_Coyote thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Vil_Coyote */
void Vil_Coyote(void const *argument) {
	/* USER CODE BEGIN Vil_Coyote */
	//Position de Vil Coyote
	int X_Coyote = 20;
	int Y_Coyote = 20; //Coyote reste sur la même ligne
	int X_Coyote_old = X_Coyote;
	int change = 0;
	//Taille de Vil Coyote
	uint16_t hauteur = 10;
	uint16_t largeur = 30;
	//Déplacement de Vil Coyote
	int v_Coyote = 5;
	//Configuration ADC pour le joystick horizontal
	uint32_t joystick_h;
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	sConfig.Channel = ADC_CHANNEL_8;
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);

	/* Infinite loop */
	for (;;) {
		//Récupération des informations sur le joystick
		HAL_ADC_Start(&hadc1);
		while (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
			;
		joystick_h = HAL_ADC_GetValue(&hadc1);

		//Modification de la position de Vil Coyote en fonction du joystick
		if (joystick_h <= 2000)
			X_Coyote += v_Coyote;
		if (joystick_h >= 2096)
			X_Coyote -= v_Coyote;

		//Prise en compte des bords de l'écran
		if (X_Coyote < 0) {
			X_Coyote = 0;
		}
		if ((X_Coyote + largeur) > 480) {
			X_Coyote = (480 - largeur);
		}

		//On regarde si Vil Coyote a changé de position
		if (X_Coyote != X_Coyote_old)
			change = 1;
		else
			change = 0;

		//On écrit dans le blackboard les positions
		taskENTER_CRITICAL();
		Position_Vil_Coyote[0] = X_Coyote;
		taskEXIT_CRITICAL();

		//Affichage de Vil Coyote en fonction de sa position

		if ( xSemaphoreTake(Mutex_LCDHandle,
				(TickType_t ) 10) == pdTRUE) {
			//Efface l'ancien Coyote
			if (change == 1) {
				BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
				BSP_LCD_FillRect(X_Coyote_old, Y_Coyote, largeur, hauteur);
			}
			X_Coyote_old = X_Coyote;
			BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY); //Dessine Vil Coyote
			BSP_LCD_FillRect(X_Coyote, Y_Coyote, largeur, hauteur);
			xSemaphoreGive(Mutex_LCDHandle);
		}
		osDelay(50); //On a une période de 50 ms
	}
	/* USER CODE END Vil_Coyote */
}

/* USER CODE BEGIN Header_Enclume_1 */
/**
 * @brief Function implementing the task_Enclume_1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Enclume_1 */
void Enclume_1(void const *argument) {
	/* USER CODE BEGIN Enclume_1 */
	//Positions de l'enclume 1
	int X_E1 = Position_Enclume_1[0];
	int Y_E1 = Position_Enclume_1[1];
	int X_E1_old = X_E1;
	int Y_E1_old = Y_E1;
	int etat_initialisation_E1 = 0; //Temoin de l'initialisation des positions au lancement de la tâche
	int change = 0;
	uint8_t MessageCollision[2];
	//Taille de l'enclume
	int largeurE1 = 20;
	int hauteurE1 = 20;
	//Vitesse de descente
	int v_enclume1 = 2; //Descente rapide
	//Période de la tâchede 250ms
	TickType_t zLastWakeTime1;
	const TickType_t zTimeincrement1 = 25; //Définition de la période
	zLastWakeTime1 = xTaskGetTickCount(); //On initialise le dernier réveil

	/* Infinite loop */
	for (;;) {
		vTaskDelayUntil(&zLastWakeTime1, zTimeincrement1); //Période de 25ms
		if (etat_enclumes[0] == 1) { //La descente est activée

			//On récupère les positions de largage, on passe une fois seulement par début de chute
			if (etat_initialisation_E1 == 0) {
				taskENTER_CRITICAL();
				X_E1 = Position_Vil_Coyote[0];
				Y_E1 = Position_Vil_Coyote[1];
				taskEXIT_CRITICAL();
				if ( xSemaphoreTake(Mutex_LCDHandle,
						(TickType_t ) 10) == pdTRUE) {
					X_E1_old = X_E1;
					Y_E1_old = Y_E1;
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //Dessine l'enclume
					BSP_LCD_FillRect(X_E1, Y_E1, largeurE1, hauteurE1);
					xSemaphoreGive(Mutex_LCDHandle);
				}
				etat_initialisation_E1 = 1;
			}
			//Descente de l'enclume
			Y_E1 += v_enclume1;

			//Fin de chute de l'enclume
			if (Y_E1 >= 272) {
				taskENTER_CRITICAL();
				etat_enclumes[0] = 0;
				taskEXIT_CRITICAL();
				etat_initialisation_E1 = 0; //On devra initialiser à nouveau la position de largage
			}

			//On regarde si l'enclume a changée de position
			if (Y_E1 != Y_E1_old)
				change = 1;
			else
				change = 0;

			//On écrit dans le blackboard les positions
			taskENTER_CRITICAL();
			Position_Enclume_1[0] = X_E1;
			Position_Enclume_1[1] = Y_E1;
			taskEXIT_CRITICAL();

			//On vérifie s'il y a collision
			if (((X_E1 < (Position_Bip_Bip[0] + 10))
					&& ((X_E1 + largeurE1) > Position_Bip_Bip[0])
					&& (Y_E1 < (Position_Bip_Bip[1] + 30))
					&& ((Y_E1 + hauteurE1) > Position_Bip_Bip[1]))) {
				MessageCollision[0] = 'C';
				xQueueSend(QueueVWHandle, &MessageCollision, 0);
			}

			//Affichage de l'enclume en fonction de sa position
			if ( xSemaphoreTake(Mutex_LCDHandle,
					(TickType_t ) 10) == pdTRUE) {
				//Efface l'ancienne enclume
				if (change == 1) {
					BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
					BSP_LCD_FillRect(X_E1_old, Y_E1_old, largeurE1, hauteurE1);
				}
				X_E1_old = X_E1;
				Y_E1_old = Y_E1;
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_FillRect(X_E1, Y_E1, largeurE1, hauteurE1);
				xSemaphoreGive(Mutex_LCDHandle);
			}

		}
	}
	/* USER CODE END Enclume_1 */
}

/* USER CODE BEGIN Header_Enclume_2 */
/**
 * @brief Function implementing the task_Enclume_2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Enclume_2 */
void Enclume_2(void const *argument) {
	/* USER CODE BEGIN Enclume_2 */
	//Positions de l'enclume 1
	int X_E2 = Position_Enclume_2[0];
	int Y_E2 = Position_Enclume_2[1];
	int X_E2_old = X_E2;
	int Y_E2_old = Y_E2;
	int etat_initialisation_E2 = 0; //Temoin de l'initialisation des positions au lancement de la tâche
	int change = 0;
	uint8_t MessageCollision[2];
	//Taille de l'enclume
	int largeurE2 = 20;
	int hauteurE2 = 20;
	//Vitesse de descente
	int v_enclume2 = 2; //Descente rapide
	//Période de la tâchede 250ms
	TickType_t zLastWakeTime2;
	const TickType_t zTimeincrement2 = 25; //Définition de la période
	zLastWakeTime2 = xTaskGetTickCount(); //On initialise le dernier réveil

	/* Infinite loop */
	for (;;) {
		vTaskDelayUntil(&zLastWakeTime2, zTimeincrement2); //Période de 25ms
		if (etat_enclumes[1] == 1) { //La descente est activée

			//On récupère les positions de largage, on passe une fois seulement par début de chute
			if (etat_initialisation_E2 == 0) {
				taskENTER_CRITICAL();
				X_E2 = Position_Vil_Coyote[0];
				Y_E2 = Position_Vil_Coyote[1];
				taskEXIT_CRITICAL();
				if ( xSemaphoreTake(Mutex_LCDHandle,
						(TickType_t ) 10) == pdTRUE) {
					X_E2_old = X_E2;
					Y_E2_old = Y_E2;
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //Dessine l'enclume
					BSP_LCD_FillRect(X_E2, Y_E2, largeurE2, hauteurE2);
					xSemaphoreGive(Mutex_LCDHandle);
				}
				etat_initialisation_E2 = 1;
			}
			//Descente de l'enclume
			Y_E2 += v_enclume2;

			//Fin de chute de l'enclume
			if (Y_E2 >= 272) {
				taskENTER_CRITICAL();
				etat_enclumes[1] = 0;
				taskEXIT_CRITICAL();
				etat_initialisation_E2 = 0; //On devra initialiser à nouveau la position de largage
			}

			//On regarde si l'enclume a changée de position
			if (Y_E2 != Y_E2_old)
				change = 1;
			else
				change = 0;

			//On écrit dans le blackboard les positions
			taskENTER_CRITICAL();
			Position_Enclume_1[0] = X_E2;
			Position_Enclume_1[1] = Y_E2;
			taskEXIT_CRITICAL();

			//On vérifie s'il y a collision
			if (((X_E2 < (Position_Bip_Bip[0] + 10))
					&& ((X_E2 + largeurE2) > Position_Bip_Bip[0])
					&& (Y_E2 < (Position_Bip_Bip[1] + 30))
					&& ((Y_E2 + hauteurE2) > Position_Bip_Bip[1]))) {
				MessageCollision[0] = 'C';
				xQueueSend(QueueVWHandle, &MessageCollision, 0);
			}

			//Affichage de l'enclume en fonction de sa position
			if ( xSemaphoreTake(Mutex_LCDHandle,
					(TickType_t ) 10) == pdTRUE) {
				//Efface l'ancienne enclume
				if (change == 1) {
					BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
					BSP_LCD_FillRect(X_E2_old, Y_E2_old, largeurE2, hauteurE2);
				}
				X_E2_old = X_E2;
				Y_E2_old = Y_E2;
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_FillRect(X_E2, Y_E2, largeurE2, hauteurE2);
				xSemaphoreGive(Mutex_LCDHandle);
			}

		}
	}
	/* USER CODE END Enclume_2 */
}

/* USER CODE BEGIN Header_Vil_Winner */
/**
 * @brief Function implementing the task_Vil_Win thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Vil_Winner */
void Vil_Winner(void const *argument) {
	/* USER CODE BEGIN Vil_Winner */
	uint8_t MessageCollision[2];
	int activation = 0;
	int etat = 0;
	char texte_vil_win[23] = "VIL COYOTE IS A WINNER";
	//Période de 500ms
	TickType_t zLastWakeTime;
	const TickType_t zTimeincrement = 500; //Définition de la période
	zLastWakeTime = xTaskGetTickCount(); //On initialise le dernier réveil
	/* Infinite loop */
	for (;;) {
		vTaskDelayUntil(&zLastWakeTime, zTimeincrement);
		switch (activation) {
		case 0:
			xQueueReceive(QueueVWHandle, &MessageCollision, portMAX_DELAY);
			if (MessageCollision[0] == 'C') {
				//On supprime les autres tâches
				vTaskDelete(task_Vil_CoyoteHandle);
				vTaskDelete(task_Bip_BipHandle);
				vTaskDelete(task_Enclume_1Handle);
				vTaskDelete(task_Enclume_2Handle);
				vTaskDelete(task_BipBip_WinHandle);
				vTaskDelete(task_Gest_EncluHandle);
				vTaskDelete(task_clockHandle);
				activation = 1;
			}
			break;
		case 1:
			if (etat == 0) {
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_DisplayStringAt(180, 135, (uint8_t*) texte_vil_win,
						LEFT_MODE);
				etat = 1;
			} else if (etat == 1) {
				BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
				BSP_LCD_DisplayStringAt(180, 135, (uint8_t*) texte_vil_win,
						LEFT_MODE);
				etat = 0;
			}
		}
	}
	/* USER CODE END Vil_Winner */
}

/* USER CODE BEGIN Header_BipBip_Winner */
/**
 * @brief Function implementing the task_BipBip_Win thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_BipBip_Winner */
void BipBip_Winner(void const *argument) {
	/* USER CODE BEGIN BipBip_Winner */
	uint8_t MessageBipBip[2];
	int activation = 0;
	int etat = 0;
	char texte_BipBip_win[23] = "BIP BIP IS A WINNER";
	//Période de 500ms
	TickType_t zLastWakeTime;
	TickType_t zTimeincrement = 500; //Définition de la période
	zLastWakeTime = xTaskGetTickCount(); //On initialise le dernier réveil
	/* Infinite loop */
	for (;;) {
		vTaskDelayUntil(&zLastWakeTime, zTimeincrement);
		switch (activation) {
		case 0:
			xQueueReceive(QueueBWHandle, &MessageBipBip, portMAX_DELAY);
			if (MessageBipBip[0] == 'W') {
				//On supprime les autres tâches
				vTaskDelete(task_Vil_CoyoteHandle);
				vTaskDelete(task_Bip_BipHandle);
				vTaskDelete(task_Enclume_1Handle);
				vTaskDelete(task_Enclume_2Handle);
				vTaskDelete(task_Vil_WinHandle);
				vTaskDelete(task_Gest_EncluHandle);
				vTaskDelete(task_clockHandle);
				activation = 1;
			}
			break;
		case 1:
			if (etat == 0) {
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_DisplayStringAt(180, 135, (uint8_t*) texte_BipBip_win,
						LEFT_MODE);
				etat = 1;
			} else if (etat == 1) {
				BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
				BSP_LCD_DisplayStringAt(180, 135, (uint8_t*) texte_BipBip_win,
						LEFT_MODE);
				etat = 0;
			}
			break;
		}

	}
	/* USER CODE END BipBip_Winner */
}

/* USER CODE BEGIN Header_Gestion_Enclume */
/**
 * @brief Function implementing the task_Gest_Enclu thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Gestion_Enclume */
void Gestion_Enclume(void const *argument) {
	/* USER CODE BEGIN Gestion_Enclume */
	uint8_t Message4Enclumes[2];
	int test_etat_enclumes;
	/* Infinite loop */
	for (;;) {
		xQueueReceive(QueueGE2EHandle, &Message4Enclumes, portMAX_DELAY);
		if (Message4Enclumes[0] == 1) {
			taskENTER_CRITICAL();
			test_etat_enclumes = etat_enclumes[0] + etat_enclumes[1];
			switch (test_etat_enclumes) {
			case 0: //Aucune enclume lâchée, donc on largue la numéro 1
				etat_enclumes[0] = 1;
				break;
			case 1: //Une enclule lâchée
				if (etat_enclumes[0] == 1)
					etat_enclumes[1] = 1; //On largue la numéro 2 car la 1 déjà lancée
				else
					etat_enclumes[0] = 1; //On largue la 1 car la 2 toujours en descente
				break;
			case 2: //Les deux enclume sont larguées, on ne fait rien
				break;
			}
			taskEXIT_CRITICAL();
		}
		osDelay(70); //Période de 70ms souple
	}
	/* USER CODE END Gestion_Enclume */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
