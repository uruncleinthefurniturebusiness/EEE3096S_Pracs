/* SMTJOS022 X BHYEBR002 EEE3096S Practical 1A*/
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
#include <stdint.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c> 
#include <stdlib.h> 


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
  MODE_OFF = 0,
  MODE_1,
  MODE_2,
  MODE_3,
} led_mode_t;

typedef enum{
	SPARKLE_PATTERN,
	SPARKLE_TRANS,
	SPARKLE_OFF
} led_state_t;

led_state_t led_state = SPARKLE_PATTERN;
uint32_t led_timer = 0;
uint8_t led_counter = 0;
uint8_t led_array[8];
uint8_t led_init = 0;

typedef struct{
  led_mode_t current_mode;
  uint8_t current_led_position;
  uint8_t direction; //1 means forward, 0 means reverse

} led_system_t;
	

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define DELAY 150

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
// TODO: Define input variables
led_system_t leds;
uint32_t last_button_time = 0;
uint8_t previous_button_state[4] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void TIM16_IRQHandler(void);

// This is our shit functions,
void update_leds(uint8_t pattern);
void init_led_system(void);

void mode1(void);
void mode2(void);
void mode3(void);
uint8_t debounce_button(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t button_index);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void update_leds(uint8_t pattern) {
    // Update each LED based on the pattern
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, (pattern & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, (pattern & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (pattern & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (pattern & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (pattern & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, (pattern & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, (pattern & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, (pattern & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void init_led_system(void){

  leds.current_mode = MODE_OFF;
  leds.current_led_position = 0;
  leds.direction = 1;
    
  // Initialize random seed
  srand(HAL_GetTick());
    
  // Turn off all LEDs
  update_leds(0x00);
}

void mode1(void){
  uint8_t pattern = (1 <<leds.current_led_position);
  update_leds(pattern);

  lcd_command(CLEAR);
  lcd_putstring("Mode 1");

  if (leds.direction == 1){
    if (leds.current_led_position == 7){
      leds.direction = 0; 					// Turn around if end reached
      leds.current_led_position = 6;
    }
    else{
      leds.current_led_position++;   // left movement
    }
  }
  else {
    if (leds.current_led_position == 0){  // Turn around if end reached
      leds.direction = 1;
      leds.current_led_position = 1;
    }
    else{
      leds.current_led_position--;						// right movements
    }

  }

}

void mode2(void){


  uint8_t pattern = 0xFF & ~(1<<leds.current_led_position);
  update_leds(pattern);

  lcd_command(CLEAR);

  lcd_putstring("Mode 2");

  if (leds.direction == 1){
    if (leds.current_led_position == 7){
      leds.direction = 0;
      leds.current_led_position = 6;
    }
    else{
      leds.current_led_position++;
    }
  }
  else {
    if (leds.current_led_position == 0){
      leds.direction = 1;
      leds.current_led_position = 1;
    }
    else{
      leds.current_led_position--;
    }

  }

}

void mode3(void){
	uint32_t time = HAL_GetTick();

	lcd_command(CLEAR);
	lcd_putstring("Mode 3"); // initializes display screen with mode number
	switch (led_state){ //switch statement checks which state the leds are in

	case SPARKLE_PATTERN: //initializes random led pattern
		led_init = rand() % 256;
		update_leds(led_init); //displays random number on leds

		led_counter = 0;
		int i = 0;

		while (i<8){
			if (((1<<i) & led_init) == 1){ //bitwise and operation which is used to add leds which are on to the led array
				led_array[led_counter++] = 1;
			}
			i++;
		}

		led_timer = time + (100 + (rand() % 1401));
		led_state = SPARKLE_TRANS;
		break;

	case SPARKLE_TRANS:
		if ((int32_t)(time - led_timer) >= 0) {
			if (led_counter > 0) {
				int j = rand() % led_counter;
				led_init &= ~(1 << led_array[j]);
				update_leds(led_init);


			}else{
				led_state = SPARKLE_PATTERN;
			}
		}
		break;

	case SPARKLE_OFF:

		break;

	}


}

uint8_t debounce_button(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t button_index) {
  uint8_t current_state = (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) ? 1 : 0;  // Active low
  uint8_t button_pressed = 0;

  if (current_state && !previous_button_state[button_index]) {
        // Button just pressed
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_button_time > 250) {  // 200ms debounce
      button_pressed = 1;
      last_button_time = current_time;
    }
  }
    
  previous_button_state[button_index] = current_state;
  return button_pressed;


}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  init_LCD();
  lcd_command(CLEAR);
  lcd_putstring("Group 9");
  lcd_command(LINE_TWO);
  lcd_putstring("Prac 1");

  init_led_system();


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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // TODO: Start timer TIM16
  // Start the timer interrupt
  HAL_TIM_Base_Start_IT(&htim16);
  

 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* Simple heartbeat - blink LED0 every 500ms to show code is running
	      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	      HAL_Delay(250);
	      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
	      HAL_Delay(250);*/

    // TODO: Check pushbuttons to change timer delay
    if (debounce_button(Button0_GPIO_Port, Button0_Pin, 0)) {
      // Toggle between 1s and 0.5s delays
      static uint8_t delay_mode = 0;  // 0 = 1s, 1 = 0.5s
      delay_mode = !delay_mode;
    
      // Stop timer before changing period
      HAL_TIM_Base_Stop_IT(&htim16);
    
      if (delay_mode == 1) {
        // 0.5 second delay
        htim16.Init.Period = 500 - 1;
      } else {
        // 1.0 second delay  
        htim16.Init.Period = 1000 - 1;
      }
    
      // Reinitialize and restart timer
      HAL_TIM_Base_Init(&htim16);
      HAL_TIM_Base_Start_IT(&htim16);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TIM16_IRQHandler(void)
{
	// Acknowledge interrupt
	HAL_TIM_IRQHandler(&htim16);

  if (debounce_button(Button1_GPIO_Port, Button1_Pin, 1)){
    leds.current_mode = MODE_1;
    leds.current_led_position = 0;
    leds.direction = 1;
  }
  else if (debounce_button(Button2_GPIO_Port, Button2_Pin, 2)){
    leds.current_mode = MODE_2;
    leds.current_led_position = 0;
    leds.direction = 1;
  }
  else if (debounce_button(Button3_GPIO_Port, Button3_Pin, 3)){
      leds.current_mode = MODE_3;
      led_state = SPARKLE_PATTERN;
  }
//s
  switch (leds.current_mode){
    case MODE_1:
      mode1();
      break;
    case MODE_2:
      mode2();
      break;
    case MODE_3:
      mode3();
      break;
    default:
      update_leds(0); // Turns the fucking LEDS off
      break;
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
