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
#define CMSIS_NN_USE_SINGLE_ROUNDING 1
// 9x9x3 image data filled with incremental values
#define IMG_DATA { \
    1, 1, 1, 1, 1, 1, 1, 1, 1,   2, 2, 2, 2, 2, 2, 2, 2, 2,   3, 3, 3, 3, 3, 3, 3, 3, 3, \
    4, 4, 4, 4, 4, 4, 4, 4, 4,   5, 5, 5, 5, 5, 5, 5, 5, 5,   6, 6, 6, 6, 6, 6, 6, 6, 6, \
    7, 7, 7, 7, 7, 7, 7, 7, 7,   8, 8, 8, 8, 8, 8, 8, 8, 8,   9, 9, 9, 9, 9, 9, 9, 9, 9 \
}

// 3x3x3x3 convolution weights filled with alternating small values
#define CONV1_WT { \
    1, -1, 1,   -1, 1, -1,   1, -1, 1, \
    1, -1, 1,   -1, 1, -1,   1, -1, 1, \
    1, -1, 1,   -1, 1, -1,   1, -1, 1, \
    \
    -1, 1, -1,   1, -1, 1,   -1, 1, -1, \
    -1, 1, -1,   1, -1, 1,   -1, 1, -1, \
    -1, 1, -1,   1, -1, 1,   -1, 1, -1, \
    \
    1, 1, 1,   1, 1, 1,   1, 1, 1, \
    -1, -1, -1,   -1, -1, -1,   -1, -1, -1, \
    1, 1, 1,   1, 1, 1,   1, 1, 1 \
}

// FC weights for 10 output neurons and 27 input features, filled with 1 for simplicity
#define FC_WEIGHTS { \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, \
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 \
}

// FC biases for each of the 10 output neurons
#define FC_BIAS { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }

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

		// Define the weight and bias arrays based on constants
//		static int8_t conv1_wt[CONV1_IM_CH * CONV1_KER_DIM * CONV1_KER_DIM * CONV1_OUT_CH] = CONV1_WT;
//		static int32_t conv1_bias[CONV1_OUT_CH] = CONV1_BIAS;
//
//		// Input image and output data buffers
//		int8_t image_data[CONV1_IM_CH * CONV1_IM_DIM * CONV1_IM_DIM] = IMG_DATA;
//		int8_t output_data[CONV1_OUT_CH * CONV1_OUT_DIM * CONV1_OUT_DIM];
//
//
		// Mock data initialization for testing `arm_convolve_s8` function

		int32_t val = 9;
		int32_t multiplier = 1;   // Scale factor (often a fixed-point number)
		int32_t shift = 30;

//		int64_t total_shift = 31 - shift;
//		int64_t new_val = val * (int64_t)multiplier;
//
//		int32_t result = new_val >> (total_shift - 1);
//		result = (result + 1) >> 1;

		int32_t result = arm_nn_requantize(val, multiplier, shift);


		char buffer[20];
		int buffer_len = sprintf(buffer, "%ld", (long)result);  // Convert result to a string

		// Transmit result via UART
		HAL_UART_Transmit(&huart2, (uint8_t*)buffer, buffer_len, 10);

		// 2. Convolution parameters
		cmsis_nn_conv_params conv_params;
		conv_params.stride.h = 1;
		conv_params.stride.w = 1;
		conv_params.padding.h = 0;
		conv_params.padding.w = 0;
		conv_params.dilation.h = 1;
		conv_params.dilation.w = 1;
		conv_params.input_offset = 0;
		conv_params.output_offset = 0;
		conv_params.activation.min = -128;
		conv_params.activation.max = 127;

		// 3. Quantization parameters
		cmsis_nn_per_channel_quant_params quant_params;
		int32_t quant_mult[] = {1};  // Three elements for three channels
		int32_t quant_shift[] = {31};                         // Three elements for three channels
		quant_params.multiplier = quant_mult;
		quant_params.shift = quant_shift;

		// 4. Input dimensions
		cmsis_nn_dims conv_input_dims;
		conv_input_dims.n = 1;   // batch size
		conv_input_dims.h = 3;   // height
		conv_input_dims.w = 3;   // width
		conv_input_dims.c = 1;   // channels

		// 5. Input data (image_data)
		int8_t image_data[3 * 3 * 1] = IMG_DATA;

		// 6. Filter dimensions
		cmsis_nn_dims conv_filter_dims;
		conv_filter_dims.h = 3;  // filter height
		conv_filter_dims.w = 3;  // filter width
		conv_filter_dims.c = 1;  // input channels
		conv_filter_dims.n = 1;  // output channels

		// 7. Weights (conv1_wt)
		int8_t conv1_wt[3 * 3 * 1 * 1] = {0};

		for (int i = 0; i < 9; i++) {
			conv1_wt[i] = 1;
		}


		// 8. Bias dimensions
		cmsis_nn_dims conv_bias_dims;
		conv_bias_dims.c = 1;    // Number of output channels

		// 9. Biases (conv1_bias)
		int32_t conv1_bias[1] = {0};  // Initialize to zero for testing

		// 10. Output dimensions
		cmsis_nn_dims conv_output_dim;
		conv_output_dim.n = 1;   // batch size
		conv_output_dim.h = 1;   // output height
		conv_output_dim.w = 1;   // output width
		conv_output_dim.c = 1;   // output channels

		// 11. Output data buffer
		int8_t conv_output[1 * 1 * 1] = {0};  // Initialize to zero

		// 1. Define convolution context (CMSIS-NN context struct)
		cmsis_nn_context ctx;
		ctx.size = arm_convolve_s8_get_buffer_size(&conv_input_dims, &conv_filter_dims);
		ctx.buf = (ctx.size > 0) ? malloc(ctx.size) : NULL;

		// Now call the convolution function
		arm_cmsis_nn_status status = arm_convolve_s8(
			&ctx,
			&conv_params,
			&quant_params,
			&conv_input_dims,
			image_data,
			&conv_filter_dims,
			conv1_wt,
			&conv_bias_dims,
			conv1_bias,
			&conv_output_dim,
			conv_output
		);

		if (ctx.buf != NULL) {
			free(ctx.buf);
		}

		 // 2. ReLU Activation
		arm_relu6_s8(conv_output, 7 * 7 * 1);

		// 3. MaxPooling Layer
		cmsis_nn_pool_params pool_params;
		pool_params.stride.w = 2;
		pool_params.stride.h = 2;
		pool_params.padding.w = 0;
		pool_params.padding.h = 0;
		pool_params.activation.min = -128;    // Activation min (usually -128 for int8)
		pool_params.activation.max = 127;     // Activation max (usually 127 for int8)

		// Define input/output dimensions structures
		cmsis_nn_dims pool_input_dims;
		pool_input_dims.n = 1;
		pool_input_dims.w = 7;
		pool_input_dims.h = 7;
		pool_input_dims.c = 3;

		cmsis_nn_dims pool_filter_dims;
		pool_filter_dims.w = 2;
		pool_filter_dims.h = 2;

		cmsis_nn_dims pool_output_dim;
		pool_output_dim.w = 3;
		pool_output_dim.h = 3;
		pool_output_dim.c = 3;

		int8_t pool_output[3 * 3 * 3] = {0};  // Output buffer for max-pooling

		status = arm_max_pool_s8(
							&ctx,
							&pool_params,
							&pool_input_dims,
							conv_output,
							&pool_filter_dims,
							&pool_output_dim,
							pool_output
						);

		if (ctx.buf != NULL) {
			free(ctx.buf);
		}

		int8_t fc_weights[10 * 27] = FC_WEIGHTS;  // Weights for 10 output neurons and 27 input features
		int32_t fc_bias[10] = FC_BIAS;                // Bias for each output neuron

		int8_t fc_output[10] = {0};  // Output buffer for FC layer

		cmsis_nn_fc_params fc_params;
		fc_params.input_offset = 0;
		fc_params.output_offset = 0;
		fc_params.activation.min = -128;    // Activation min (usually -128 for int8)
		fc_params.activation.max = 127;     // Activation max (usually 127 for int8)

		cmsis_nn_per_tensor_quant_params quant_t_params;
		quant_params.multiplier = 1073741824;  // Unity scaling multiplier
		quant_params.shift = 0;                // No shift for simplicity

		// Define input/output dimensions structures
		cmsis_nn_dims fc_input_dims;
		fc_input_dims.n = 1;                     // Batch size
		fc_input_dims.w = 3;
		fc_input_dims.h = 3;
		fc_input_dims.c = 3;

		cmsis_nn_dims fc_filter_dims;
		fc_filter_dims.n = 3*3*3;
		fc_filter_dims.c = 10;        // Input channels for the filter

		cmsis_nn_dims fc_bias_dims;
		fc_bias_dims.n = 10;         // Bias size matches the output channels

		cmsis_nn_dims fc_output_dim;
		fc_output_dim.n = 1;
		fc_output_dim.c = 10;

		// 6. Execute Fully Connected Layer
		status = arm_fully_connected_s8(
							&ctx,
							&fc_params,
							&quant_t_params,
							&fc_input_dims,
							pool_output,
							&fc_filter_dims,
							fc_weights,
							&fc_bias_dims,
							fc_bias,
							&fc_output_dim,
							fc_output
						);

		if (ctx.buf != NULL) {
			free(ctx.buf);
		}



		// Check convolution status
		if (status != ARM_CMSIS_NN_SUCCESS) {
			char res[100];
			sprintf(res, "Failed: %d\n", status);
			HAL_UART_Transmit(&huart2, (uint8_t*)res, strlen(res), 10);
		} else {
			char *res = "Successful\n";
			HAL_UART_Transmit(&huart2, (uint8_t*)res, strlen(res), 10);
		}

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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
