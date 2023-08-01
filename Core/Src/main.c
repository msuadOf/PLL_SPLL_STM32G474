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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include "spll_sogi.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"

extern SPLL_1PH_SOGI spll1;

#define PR_ISR_FREQ 50000

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CLAMP(x, max, min) ((x < max) ? ((x > min) ? x : min) : max)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float HB_duty=0.2;
uint16_t ADC_Value[4] = {0};
uint16_t DAC_Value = 0;



uint32_t ARR;

typedef struct
{
  float A0;       /**< The derived gain, A0 = Kp + Ki + Kd . */
  float A1;       /**< The derived gain, A1 = -Kp - 2Kd. */
  float A2;       /**< The derived gain, A2 = Kd . */
  float state[3]; /**< The state array of length 3. */
  float Kp;       /**< The proportional gain. */
  float Ki;       /**< The integral gain. */
  float Kd;       /**< The derivative gain. */
} PID_t;
PID_t pid_i_Cur = {
    .Kp = 60,
    .Ki = 30};
PID_t pid_o_Vol = {
    .Kp = 0.03,
    .Ki = 0.05,
    .Kd = 0.00};
inline void pid_init(PID_t *S)
{
  /* Derived coefficient A0 */
  S->A0 = S->Kp + S->Ki + S->Kd;
  /* Derived coefficient A1 */
  S->A1 = (-S->Kp) - ((float)2.0f * S->Kd);
  /* Derived coefficient A2 */
  S->A2 = S->Kd;
}
inline float pid_process_i_Cur(PID_t *S, float aim, float current)
{
  pid_init(S);
  float out;
  float in = aim - current;

  /* u[t] = u[t - 1] + A0 * e[t] + A1 * e[t - 1] + A2 * e[t - 2]  */
  out = (S->A0 * in) +
        (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

  /* Update state */
  S->state[1] = S->state[0];
  S->state[0] = in;
  S->state[2] = out;

  out = CLAMP(out, 99, -99);
  /* return to application */
  return (out);
}
inline float pid_process_o_Vol(PID_t *S, float aim, float current)
{
  pid_init(S);
  float out;
  float in = aim - current;

  /* u[t] = u[t - 1] + A0 * e[t] + A1 * e[t - 1] + A2 * e[t - 2]  */
  out = (S->A0 * in) +
        (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

  /* Update state */
  S->state[1] = S->state[0];
  S->state[0] = in;
  S->state[2] = out;

  out = CLAMP(out, 5, 0.1);
  /* return to application */
  return (out);
}

int UART_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  int length;
  char buffer[128];
  length = vsnprintf(buffer, 128, fmt, ap);
  HAL_UART_Transmit(huart, (uint8_t *)buffer, length, HAL_MAX_DELAY); // HAL_MAX_DELAY
  //    CDC_Transmit_FS((uint8_t*)buffer,length);
  va_end(ap);
  return length;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_HRTIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  UART_printf(&huart1,"=== MCU Peripheral turn on ===\n");

  HAL_Delay(1000);

  HAL_TIM_Base_Start(&htim3);
  // HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Value, 4);
  // HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,(uint32_t *)&DAC_Value,1,DAC_ALIGN_12B_R);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  ARR = (hhrtim1.Instance->sTimerxRegs[0].PERxR);

  //	hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_SLAVE;
  //  hhrtim1.Init.SyncInputSource = HRTIM_SYNCINPUTSOURCE_INTERNALEVENT;

  HAL_Delay(1);

  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
  hhrtim1.Instance->sTimerxRegs[0].CMP1xR = (int)(0.5f * (hhrtim1.Instance->sTimerxRegs[0].PERxR));

  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
  hhrtim1.Instance->sTimerxRegs[1].CMP1xR = (int)(0.5f * (hhrtim1.Instance->sTimerxRegs[1].PERxR));

  HAL_GPIO_WritePin(check_GPIO_Port, check_Pin, 0);
  while (1)
  {
    UART_printf(&huart1,"=== MCU working ===\n");
    //UART_printf(&huart1, "i_cur:%f,%f,%f,%f,%f,%f\n", i_Cur, HB_duty, i_Vol, o_Vol, k,i_Cur_ref);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  float ac_value;
  if (hadc->Instance == ADC1)
  {


    hhrtim1.Instance->sTimerxRegs[0].CMP1xR = (int)((HB_duty) * (hhrtim1.Instance->sTimerxRegs[0].PERxR));
    hhrtim1.Instance->sTimerxRegs[1].CMP1xR = (int)((HB_duty) * (hhrtim1.Instance->sTimerxRegs[1].PERxR));

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Value);
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
