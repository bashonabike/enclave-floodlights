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
#include "math.h"
#include <stdlib.h>
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

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
unsigned short trimDuty = 150;
unsigned long width = 0;
unsigned long avgWidth = 0;
_Bool set = 0;
_Bool ready = 0;
_Bool flip = 0;
_Bool blip = 0;
unsigned long start = 0;
unsigned short place = 0;

unsigned short curMains = 0;
unsigned short curDuty = 0;
unsigned char incrDuty = 0;
int curShift = 0;
uint32_t curStartTilNow = 0;
unsigned short effDuty = 0;
unsigned short rise1 = 0;
unsigned short fall1 = 0;
unsigned short rise2 = 0;
unsigned short fall2 = 0;

volatile uint32_t *DWT_CONTROL = (uint32_t *) 0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *) 0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *) 0xE000EDFC;
volatile uint32_t *LAR  = (uint32_t *) 0xE0001FB0;   // <-- added lock access register

//PLACE SHIFT HERE!!  FACTOR FOR DELAY IN MAINS DETECTION
int shift = 1440;

const uint32_t startReset= (uint32_t)(pow(2, sizeof(uint32_t)*8));
const uint32_t startTilNowReset= (uint32_t)(pow(2, sizeof(uint32_t)*8)/64);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define micros() (*DWT_CYCCNT/64)
#define starttilnow() ((micros() - start) % startTilNowReset)

#define ADC_WIDTH 4092
#define HALF_ADC_WIDTH 2048

#define MIN_TRIM 0
#define MAX_TRIM 4096
//#define MIN_TRIM 100
//#define MAX_TRIM 3900
#define WIDTH_TRIM (MAX_TRIM - MIN_TRIM)

typedef enum  {
	TRIM_SHIFT = ADC_CHANNEL_1,
	TRIM_DUTY = ADC_CHANNEL_2,
	MAINS = ADC_CHANNEL_4
} ChannelDef;

uint16_t readADCChannel(ChannelDef channel) {
	  ADC_ChannelConfTypeDef sConfig = {0};

	/** Configure Regular Channel
	  */
	  sConfig.Channel = channel;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1);
	  return HAL_ADC_GetValue(&hadc1);
}
void resetRiseFall() {
  //Divide by 2 since we cycle duty per half-wave, not full wave
  effDuty = (short)((((unsigned long)width
		  *(unsigned long)(trimDuty - MIN_TRIM))) / (2*WIDTH_TRIM));
  rise1 = ((width/2) - effDuty)/2;
  fall1 = rise1 + effDuty;
  rise2 = (width/2) + rise1;
  fall2 = (width/2) + fall1;
}

void mainsDetect() {
  //Why does width occasionally jump to ~20k micros for a bit??  Then returns back to ~61Hz
  //Filtered out by blip discard method.  May be artifact of low frequency operation
	curMains = readADCChannel(MAINS);
  if (flip && curMains > HALF_ADC_WIDTH) {
    if (set) {
    	incrDuty++;
    	if(incrDuty%16 == 0)        trimDuty = (trimDuty + 1)%MAX_TRIM;
    	curStartTilNow = starttilnow();
      //Slowly track to actual width if permanent shift
      avgWidth = ((19*(unsigned long)width + (curStartTilNow))/20);
      //Try to filter out blips, is this an actual frequency shift on mains?
      //If average closer to calculated than historical, this is the new frequency
      //Explicit cast to (long) since stdlib has no overload of abs for unsigned long
      if (width == 0 || abs((unsigned long)((curStartTilNow) - width)) < 1000
       || abs((unsigned long)((curStartTilNow) - avgWidth)) < abs((unsigned long)(width - avgWidth))) {
        width = curStartTilNow;
        //Reset avg
        avgWidth = width;
        ready = 1;
      //reset applicable variables
resetRiseFall();
      }
    //Factor for blip, back start up to estimated actual point
    if (abs((long)((curStartTilNow) - width)) > 1000) {
    start = (start + width)%startReset;
}
else {
    start = micros();
  }

    }
    else {
      set = 1;
    start = micros();
    }
    flip = 0;
//    curDuty = readADCChannel(TRIM_DUTY);
//    if (trimDuty != curDuty) {
//      trimDuty = (unsigned short) ((9*trimDuty + curDuty)/10);
   // trimDuty = (trimDuty + 1)%MAX_TRIM;
      //reset applicable variables
resetRiseFall();
//  }
  } else if (!flip && curMains <= HALF_ADC_WIDTH) {
    flip = 1;
  }
}

void pulseFloodlight() {
	if (ready) {
	    //Modulo by width in case miss a cycle
	    place = (starttilnow() + shift) % width;
	    if (place >= fall2 || place < rise1) {
	    	HAL_GPIO_WritePin(GPIOB, IndicLED_Pin, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOB, Floodlight_Pin, GPIO_PIN_RESET);
	    } else if (place >= rise1 && place < fall1) {
	    	HAL_GPIO_WritePin(GPIOB, IndicLED_Pin, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOB, Floodlight_Pin, GPIO_PIN_SET);
	    } else if (place >= fall1 && place < rise2) {
	    	HAL_GPIO_WritePin(GPIOB, IndicLED_Pin, GPIO_PIN_RESET);
	    	HAL_GPIO_WritePin(GPIOB, Floodlight_Pin, GPIO_PIN_RESET);
	    } else if (place >= rise2 && place < fall2) {
	    	HAL_GPIO_WritePin(GPIOB, IndicLED_Pin, GPIO_PIN_SET);
	    	HAL_GPIO_WritePin(GPIOB, Floodlight_Pin, GPIO_PIN_SET);
	    }
	  }

//	curShift = 9*(readADCChannel(TRIM_SHIFT) - 2400);
//	  if (shift != curShift) {
//	    shift = (9*shift + curShift)/10;
//	  }
}
// EXTI Mains External Interrupt ISR Handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == Mains_Pin) {
		mainsDetect();

	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	pulseFloodlight();
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //Start interrupts & ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_TIM_Base_Start_IT(&htim2);

  *DEMCR = *DEMCR | 0x01000000;     // enable trace
  *LAR = 0xC5ACCE55;                // <-- added unlock access to DWT (ITM, etc.)registers
  *DWT_CYCCNT = 0;                  // clear DWT cycle counter
  *DWT_CONTROL = *DWT_CONTROL | 1;  // enable DWT cycle counter

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//  mainsDetect();
//	  pulseFloodlight();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, Floodlight_Pin|IndicLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Mains_Pin */
  GPIO_InitStruct.Pin = Mains_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Mains_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Floodlight_Pin IndicLED_Pin */
  GPIO_InitStruct.Pin = Floodlight_Pin|IndicLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
