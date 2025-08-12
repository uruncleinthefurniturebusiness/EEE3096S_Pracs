

/* SMTJOS022 X BHYEBR002 EEE3096S Practical 1B*/

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
#include <stdint.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 100
#define SCALE 1000000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define and initialise the global varibales required

 int dim[] = {128, 160, 192, 224, 256};
 uint32_t  start_time=0, end_time=0, execution_time=0;
 uint64_t check_sum=0;

/*
  start_time
  end_time
  execution_time 
  checksum: should be uint64_t
  initial width and height maybe or you might opt for an array??
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);


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
  /* USER CODE BEGIN 2 */
  //TODO: Turn on LED 0 to signify the start of the operation
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);


  //TODO: Record the start time
  //start_time = HAL_Get_Tick();
  
  
  //TODO: Call the Mandelbrot Function and store the output in the checksum variable defined initially
  //check_sum = calculate_mandelbrot_fixed_point_arithmetic(dim[0], dim[0], MAX_ITER);

  //TODO: Record the end time
  //end_time = HAL_Get_Tick();

  //TODO: Calculate the execution time
  //execution_time = end_time - start_time;

  const int num_sizes = sizeof(dim) / sizeof(dim[0]);

      // Arrays to hold results for viewing in debugger
      uint64_t checksums[num_sizes];
      uint32_t exec_times[num_sizes];

      for (int i = 0; i < num_sizes; i++) {
          // Record start time
          start_time = HAL_GetTick();

          // Run fixed-point Mandelbrot with square image dim[i] x dim[i]
          check_sum = calculate_mandelbrot_fixed_point_arithmetic(dim[i], dim[i], MAX_ITER);

          // Record end time
          end_time = HAL_GetTick();

          execution_time = end_time - start_time;

          // Store results
          checksums[i] = check_sum;
          exec_times[i] = execution_time;

          // Toggle LED1 on to signal completion of this run
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

          // Hold LEDs on for 1 second
          HAL_Delay(1000);

          // Turn off LED1 before next run
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      }

      // Turn off LED0 after all runs done
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  

  //TODO: Turn on LED 1 to signify the end of the operation


  //TODO: Hold the LEDs on for a 1s delay
  

  //TODO: Turn off the LEDs
  

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Mandelbroat using variable type integers and fixed point arithmetic
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation


  for (int y = 0; y < height-1; y++){
	  for (int x = 0; x < width-1; x++){
		  // Convert to fixed-point coordinates
		  // x0 = (x/width) * 3.5 - 2.5
		  int64_t x0 = ((int64_t)x * 3500000 / width) - 2500000;
		  // y0 = (y/height) * 2.0 - 1.0
		  int64_t y0 = ((int64_t)y * 2000000 / height) - 1000000;
		  int64_t xi = 0, yi =0;
		  int iter = 0;

		  while (iter< max_iterations){
			  int64_t xi2 = (xi*xi)/SCALE;
			  int64_t yi2 = (yi*yi)/SCALE;

			  if (xi2 + yi2 > 4 * SCALE) {
                  break;
              }


			  int64_t temp = xi2-yi2;
			  yi = (2*xi*yi)/SCALE+y0;
			  xi = temp + x0;
			  iter++;
		  }
		  mandelbrot_sum = mandelbrot_sum + iter;

	  }
  }
    
    return mandelbrot_sum;

}

//TODO: Mandelbroat using variable type double
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation
    for (int y = 0; y < height-1; y++){
    	  for (int x = 0; x < width-1; x++){
    		  // Convert to fixed-point coordinates
    		  // x0 = (x/width) * 3.5 - 2.5
    		  double x0 = ((double)x * 3.5 / width) - 2.5;
    		  // y0 = (y/height) * 2.0 - 1.0
    		  double y0 = ((double)y * 2.0 / height) - 1.0;
    		  double xi = 0, yi =0;
    		  int iter = 0;

    		  while (iter< max_iterations && (xi*xi+yi*yi)<=4){

    			  double temp = xi*xi-yi*yi;
    			  yi = (2*xi*yi)+y0;
    			  xi = temp + x0;
    			  iter++;
    		  }
    		  mandelbrot_sum = mandelbrot_sum + iter;

    	  }
      }


    
    return mandelbrot_sum;
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
