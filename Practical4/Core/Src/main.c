/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx.h"
#include "lcd_stm32f4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS  128       // Number of samples in LUT
#define TIM2CLK 16e6 // STM Clock frequency: Hint You might want to check the ioc file
#define F_SIGNAL  125e3	// Frequency of output analog signal

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs
uint32_t Sine_LUT[] = {
	    2047, 2147, 2248, 2347, 2446, 2545, 2641, 2737,
	    2831, 2922, 3012, 3100, 3185, 3267, 3346, 3422,
	    3495, 3564, 3630, 3692, 3749, 3803, 3853, 3898,
	    3939, 3975, 4006, 4033, 4055, 4072, 4085, 4092,
	    4095, 4092, 4085, 4072, 4055, 4033, 4006, 3975,
	    3939, 3898, 3853, 3803, 3749, 3692, 3630, 3564,
	    3495, 3422, 3346, 3267, 3185, 3100, 3012, 2922,
	    2831, 2737, 2641, 2545, 2446, 2347, 2248, 2147,
	    2047, 1947, 1846, 1747, 1648, 1549, 1453, 1357,
	    1263, 1172, 1082,  994,  909,  827,  748,  672,
	     599,  530,  464,  402,  345,  291,  241,  196,
	     155,  119,   88,   61,   39,   22,    9,    2,
	       0,    2,    9,   22,   39,   61,   88,  119,
	     155,  196,  241,  291,  345,  402,  464,  530,
	     599,  672,  748,  827,  909,  994, 1082, 1172,
	    1263, 1357, 1453, 1549, 1648, 1747, 1846, 1947
	};


uint32_t Saw_LUT[] = {
	       0,   31,   63,   95,  127,  159,  191,  223,
	     255,  287,  319,  351,  383,  415,  447,  479,
	     511,  543,  575,  607,  639,  671,  703,  735,
	     767,  799,  831,  863,  895,  927,  959,  991,
	    1023, 1055, 1087, 1119, 1151, 1183, 1215, 1247,
	    1279, 1311, 1343, 1375, 1407, 1439, 1471, 1503,
	    1535, 1567, 1599, 1631, 1663, 1695, 1727, 1759,
	    1791, 1823, 1855, 1887, 1919, 1951, 1983, 2015,
	    2047, 2079, 2111, 2143, 2175, 2207, 2239, 2271,
	    2303, 2335, 2367, 2399, 2431, 2463, 2495, 2527,
	    2559, 2591, 2623, 2655, 2687, 2719, 2751, 2783,
	    2815, 2847, 2879, 2911, 2943, 2975, 3007, 3039,
	    3071, 3103, 3135, 3167, 3199, 3231, 3263, 3295,
	    3327, 3359, 3391, 3423, 3455, 3487, 3519, 3551,
	    3583, 3615, 3647, 3679, 3711, 3743, 3775, 3807,
	    3839, 3871, 3903, 3935, 3967, 3999, 4031, 4063
	};


uint32_t Triangle_LUT[] = {
	       0,  127,  255,  383,  511,  639,  767,  895,
	    1023, 1151, 1279, 1407, 1535, 1663, 1791, 1919,
	    2047, 2175, 2303, 2431, 2559, 2687, 2815, 2943,
	    3071, 3199, 3327, 3455, 3583, 3711, 3839, 3967,
	    4095, 4222, 4350, 4478, 4606, 4734, 4862, 4990,
	    5118, 5246, 5374, 5502, 5630, 5758, 5886, 6014,
	    6142, 6270, 6398, 6526, 6654, 6782, 6910, 7038,
	    7166, 7294, 7422, 7550, 7678, 7806, 7934, 8062,
	    8190, 8062, 7934, 7806, 7678, 7550, 7422, 7294,
	    7166, 7038, 6910, 6782, 6654, 6526, 6398, 6270,
	    6142, 6014, 5886, 5758, 5630, 5502, 5374, 5246,
	    5118, 4990, 4862, 4734, 4606, 4478, 4350, 4222,
	    4095, 3967, 3839, 3711, 3583, 3455, 3327, 3199,
	    3071, 2943, 2815, 2687, 2559, 2431, 2303, 2175,
	    2047, 1919, 1791, 1663, 1535, 1407, 1279, 1151,
	    1023,  895,  767,  639,  511,  383,  255,  127
	};


uint32_t Piano_LUT[] = {
	    2047, 2176, 1988, 2095, 2061, 2086, 2038, 2153,
	    2094, 2148, 2082, 2054, 2017, 2318, 2058, 1780,
	    2028, 2046, 1974, 2054, 2062, 2050, 1607, 2023,
	    2319, 2044, 2841, 1711, 1774, 1854, 2232, 2187,
	    1557, 2157, 1973, 2060, 1991, 2086, 1750, 1957,
	    1940, 2098, 2241, 1930, 2242, 1935, 2586, 2031,
	    2094, 2063, 1989, 2062, 2025, 2037, 2778, 1949,
	    2375, 1877, 2002, 1954, 1502, 1918, 1709, 2035,
	    1792, 2069, 2138, 2047, 2016, 2023, 1735, 2147,
	    1909, 2084, 1807, 2435, 2193, 2033, 2344, 2168,
	    2028, 2044, 2007, 2071, 2034, 2041, 2180, 2053,
	    2087, 1934, 1853, 2034, 2013, 2217, 2011, 2103,
	    2331, 2052, 2075, 2039, 2069, 2024, 1770, 1928,
	    1995, 1998, 2156, 2082, 2036, 2001, 1848, 2145,
	    2007, 2077, 1999, 2071, 2032, 2059, 1947, 1918,
	    1627, 1794, 2075, 2055, 2004, 1943, 2092, 2047
	};


uint32_t Guitar_LUT[] = {
	    2047, 2187, 2186, 2081, 2069, 1965, 1952, 2033,
	    1819, 2067, 2062, 2049, 2065, 2060, 1979, 2042,
	    2045, 2038, 1895, 2249, 1989, 1980, 2233, 2129,
	    1926, 2049, 2033, 2054, 2042, 2023, 2010, 2050,
	    2049, 2070, 1962, 2092, 1963, 2083, 2027, 1915,
	    2137, 2037, 2051, 2049, 2054, 1996, 2090, 2044,
	    1971, 2029, 2107, 2030, 2111, 2090, 1978, 2028,
	    2091, 2088, 2068, 1949, 2099, 2069, 2364, 2128,
	    2073, 2128, 2146, 2136, 2088, 2079, 1844, 1887,
	    2055, 2044, 2061, 2054, 2145, 2046, 2040, 2051,
	    2099, 2106, 1938, 2084, 2061, 1909, 2082, 1973,
	    2065, 2071, 2046, 2041, 2008, 1977, 2077, 2046,
	    2154, 2013, 2134, 2019, 2039, 2028, 2257, 2215,
	    2031, 2049, 2041, 2045, 2078, 1998, 2046, 2049,
	    1996, 2038, 2096, 1980, 2159, 2001, 2121, 1872,
	    2073, 1961, 2198, 1984, 1866, 2300, 1997, 2047
	};



uint32_t Drum_LUT[] = {
	    2047, 3396, 1864, 2110, 2063, 3400, 1893, 1933,
	    2086, 2144, 2036, 2030, 2039, 1949, 1414, 1848,
	    2091, 2032, 2044, 2059, 2048, 2093, 2044, 2020,
	    2094, 1803, 2072, 2045, 2173, 2017, 3235, 1276,
	    1671, 2193, 2436, 2104, 1931, 2062, 2028, 2032,
	    4094, 1922, 2035, 2042, 2502, 2372, 1820, 1996,
	    2284, 2072, 2018, 2051,  671, 2425, 2075, 2068,
	      17, 2239, 2042, 2023, 2192, 2052, 2902, 2386,
	    4073, 2009, 2285, 2038, 2518, 2711, 2048, 2096,
	    1629, 2001, 2086, 2043, 2549, 1894, 2061, 2076,
	    2001, 2055, 2043, 2049, 2098, 2047, 2065, 2047,
	    2923, 2071, 2017, 2080, 2018, 2049, 1831, 2014,
	     837, 2295, 1978, 2040, 1980, 2036, 2106, 2042,
	    1643, 2055, 2066, 2032, 2050, 2442, 2145, 2218,
	    2126, 2032, 2038, 1781, 2154, 2441, 2132, 4094,
	    2150, 2012, 2036, 1615, 1981,  323, 2525, 2047
	};


// THIS is for Task 5, where we switch between the waves
typedef enum{
	SIN,
	SAW,
	TRI,
	PIA,
	GUI,
	DRU,
	COUNT
} Waveform_t;


volatile Waveform_t currentWave = SIN;

// Task 5 debouncing stuffs
static uint32_t lastPress = 0;
#define DEBOUNCE_MS 150   // debounce threshold (ms)





// TODO: Equation to calculate TIM2_Ticks
uint32_t TIM2_Ticks = TIM2CLK / (F_SIGNAL * NS); // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_IRQHandler(void);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  init_LCD();
  /* USER CODE BEGIN 2 */
  // TODO: Start TIM3 in PWM mode on channel 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  // TODO: Start DMA in IT mode on TIM2->CH1. Source is LUT and Dest is TIM3->CCR3; start with Sine LUT
  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t) Sine_LUT, (uint32_t) &(TIM3->CCR3), NS);
  // TODO: Write current waveform to LCD(Sine is the first waveform)
  lcd_putstring("cracker");
  // TODO: Enable DMA (start transfer from LUT to CCR)
  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_Ticks - 1;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* TIM2_CH1 DMA Init */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_tim2_ch1.Instance = DMA1_Stream5;
  hdma_tim2_ch1.Init.Channel = DMA_CHANNEL_3;         // TIM2_CH1 is on channel 3
  hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH; // Memory -> TIM3->CCR3
  hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;    // Peripheral address fixed
  hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;        // Memory address increments
  hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_tim2_ch1.Init.Mode = DMA_CIRCULAR;            // Repeat LUT automatically
  hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Link DMA handle to TIM2 handle */
  __HAL_LINKDMA(&htim2, hdma[TIM_DMA_ID_CC1], hdma_tim2_ch1);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // -------------------------------
  // LCD pins configuration
  // -------------------------------
  // Configure PC14 (RS) and PC15 (E) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure PB8 (D4) and PB9 (D5) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure PA12 (D6) and PA15 (D7) as output push-pull
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Set all LCD pins LOW initially
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);


  // -------------------------------
  // Button0 configuration (PA0)
  // -------------------------------
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;         // Use pull-up resistor
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Enable and set EXTI line 0 interrupt priority
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_IRQHandler(void){

	// TODO: Debounce using HAL_GetTick()

	uint32_t now = HAL_GetTick();


	// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
	// HINT: Consider using C's "switch" function to handle LUT changes

	if ((now - lastPress) > DEBOUNCE_MS){
		lastPress = now;

		// Stops the DMA transfer
		__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
		HAL_DMA_Abort_IT(&hdma_tim2_ch1);

		// Increment to next waveform
        currentWave = (currentWave + 1) % WAVE_COUNT;

        uint32_t *lutPtr = NULL;
        const char *name = "";

        switch (currentWave) {
			case SIN:
				lutPtr = Sine_LUT;
				name = "Sine";
				break;
			case SAW:
				lutPtr = Saw_LUT;
				name = "Sawtooth";
				break;
			case TRI:
				lutPtr = Triangle_LUT;
				name = "Triangle";
				break;
			case PIA:
				lutPtr = Piano_LUT;
				name = "Piano";
				break;
			case GUI:
				lutPtr = Guitar_LUT;
				name = "Guitar";
				break;
			case DRU:
				lutPtr = Drum_LUT;
				name = "Drum";
				break;
		}

        // Restart DMA with new lut
        HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)lutPtr, (uint32_t)&htim2.Instance->CCR1, NS);

        __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

        // 5. Update LCD
        lcd_command(CLEAR);
        lcd_putstring(name);

	}

	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
