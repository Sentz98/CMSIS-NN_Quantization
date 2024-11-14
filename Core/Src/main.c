/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "arm_math.h"

#include "arm_nnfunctions.h"
#include "arm_nnsupportfunctions.h"

#include "biases_data.h"
#include "config_data.h"
#include "input_data.h"
#include "output_mult_data.h"
#include "output_ref_data.h"
#include "output_shift_data.h"
#include "weights_data.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//#define CMSIS_NN_USE_SINGLE_ROUNDING 1
// 9x9x3 image data filled with incremental values
#define BASIC_OUT_CH 1
#define BASIC_IN_CH 1
#define BASIC_INPUT_W 3
#define BASIC_INPUT_H 3
#define BASIC_DST_SIZE 1
#define BASIC_INPUT_SIZE 9
#define BASIC_OUT_ACTIVATION_MIN -128
#define BASIC_OUT_ACTIVATION_MAX 127
#define BASIC_INPUT_BATCHES 1
#define BASIC_FILTER_X 3
#define BASIC_FILTER_Y 3
#define BASIC_STRIDE_X 1
#define BASIC_STRIDE_Y 1
#define BASIC_PAD_X 0
#define BASIC_PAD_Y 0
#define BASIC_OUTPUT_W 1
#define BASIC_OUTPUT_H 1
#define BASIC_INPUT_OFFSET 128
#define BASIC_OUTPUT_OFFSET 127
#define BASIC_DILATION_X 1
#define BASIC_DILATION_Y 1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
	  	uint8_t start[] = "STAR EXECUTION\r\n"; //Data to send
		HAL_UART_Transmit(&huart2,start,sizeof(start),10);// Sending in normal mode

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); //SET Led to measure latency

	    // const arm_cmsis_nn_status expected = ARM_CMSIS_NN_SUCCESS;
	    int8_t output[BASIC_DST_SIZE] = {0};

	    cmsis_nn_context ctx;
	    cmsis_nn_conv_params conv_params;
	    cmsis_nn_per_channel_quant_params quant_params;
	    cmsis_nn_dims input_dims;
	    cmsis_nn_dims filter_dims;
	    cmsis_nn_dims bias_dims;
	    cmsis_nn_dims output_dims;

	    const int32_t *bias_data = basic_biases;
	    for(int i=0; i < 9; i++) { basic_weights[i] = 1; }
	    const int8_t *kernel_data = basic_weights;
	    for(int i=0; i < 9; i++) { basic_input[i] = 1; }
	    const int8_t *input_data = basic_input;
	    const int8_t *output_ref = basic_output_ref;
	    const int32_t output_ref_size = BASIC_DST_SIZE;

	    input_dims.n = BASIC_INPUT_BATCHES;
	    input_dims.w = BASIC_INPUT_W;
	    input_dims.h = BASIC_INPUT_H;
	    input_dims.c = BASIC_IN_CH;
	    filter_dims.w = BASIC_FILTER_X;
	    filter_dims.h = BASIC_FILTER_Y;
	    filter_dims.c = BASIC_IN_CH;
	    output_dims.w = BASIC_OUTPUT_W;
	    output_dims.h = BASIC_OUTPUT_H;
	    output_dims.c = BASIC_OUT_CH;

	    conv_params.padding.w = BASIC_PAD_X;
	    conv_params.padding.h = BASIC_PAD_Y;
	    conv_params.stride.w = BASIC_STRIDE_X;
	    conv_params.stride.h = BASIC_STRIDE_Y;
	    conv_params.dilation.w = BASIC_DILATION_X;
	    conv_params.dilation.h = BASIC_DILATION_Y;

	    conv_params.input_offset = 0; // BASIC_INPUT_OFFSET;
	    conv_params.output_offset = 0; // BASIC_OUTPUT_OFFSET;
	    conv_params.activation.min = BASIC_OUT_ACTIVATION_MIN;
	    conv_params.activation.max = BASIC_OUT_ACTIVATION_MAX;
	    quant_params.multiplier = (int32_t *)basic_output_mult;
	    quant_params.shift = (int32_t *)basic_output_shift;

	    int32_t buf_size = arm_convolve_s8_get_buffer_size(&input_dims, &filter_dims);
	    ctx.buf = malloc(buf_size);
	    ctx.size = 0;

	    arm_convolve_s8(&ctx,
					 &conv_params,
					 &quant_params,
					 &input_dims,
					 input_data,
					 &filter_dims,
					 kernel_data,
					 &bias_dims,
					 bias_data,
					 &output_dims,
					 output
		);

	    if (ctx.buf)
	    {
	        // The caller is responsible to clear the scratch buffers for security reasons if applicable.
	        memset(ctx.buf, 0, buf_size);
	        free(ctx.buf);
	    }

		// Check convolution status
//		if (status != ARM_CMSIS_NN_SUCCESS) {
//			char res[100];
//			sprintf(res, "Failed: %d\n", status);
//			HAL_UART_Transmit(&huart2, (uint8_t*)res, strlen(res), 10);
//		} else {
//			char *res = "Successful\n";
//			HAL_UART_Transmit(&huart2, (uint8_t*)res, strlen(res), 10);
//		}
//
//		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

//		for (int i = 0; i < 10; i++) {
//			uint8_t res[20];
//			snprintf((char *)res, sizeof(res), "%d: %d\n\r", i, output_data[i]);
//
//			// Transmit only the length of the formatted string
//			HAL_UART_Transmit(&huart2, res, strlen((char *)res), 10);
//		}


		HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
