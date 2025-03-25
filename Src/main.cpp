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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rabcl/utils/type.hpp"
#include "rabcl/interface/can.hpp"
#include "rabcl/component/jga25_370.hpp"
#include "rabcl/controller/omni_drive.hpp"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t control_count = 0;
char printf_buf[100];

rabcl::Info robot_data;
rabcl::JGA25_370* yaw_motor;

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
int16_t read_tim2_encoder_value()
{
  uint16_t enc_buff = TIM2->CNT;
  int16_t enc_count = (int16_t)enc_buff - 32767;
  TIM2->CNT = 32767;
  return enc_count;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim15)
  {
    control_count++;
    if (control_count >= 10) // 100Hz
    {
      control_count = 0;

      // ---yaw motor
      yaw_motor->SetCmdVel(robot_data.yaw_vel_);
      // snprintf(printf_buf, 100, "robot_data.yaw_vel: %f[rad/s]\n", robot_data.yaw_vel_);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);

      yaw_motor->SetEncoderCount(read_tim2_encoder_value());
      yaw_motor->UpdataEncoder();
      // snprintf(printf_buf, 100, "yaw_act_vel: %f[rad/s]\n", yaw_motor->GetActVel());
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      // snprintf(printf_buf, 100, "yaw_act_pos: %f[rad], %f[deg]\n", yaw_motor->GetActPos(), yaw_motor->GetActPos() * 57.295779513);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);

      int16_t output = yaw_motor->CalcMotorOutput();
      // snprintf(printf_buf, 100, "yaw_output: %d[count]\n", output);
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
      if (output > 0)
      {
        HAL_GPIO_WritePin(MOTOR_PAHSE_GPIO_Port, MOTOR_PAHSE_Pin, GPIO_PIN_SET);
      }
      else
      {
        output *= -1;
        HAL_GPIO_WritePin(MOTOR_PAHSE_GPIO_Port, MOTOR_PAHSE_Pin, GPIO_PIN_RESET);
      }

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)output);
      // snprintf(printf_buf, 100, "yaw_cmd_vel: %f[rad/s]\n", yaw_motor->GetCmdVel());
      // HAL_UART_Transmit(&huart2, (uint8_t*)printf_buf, strlen(printf_buf), 1000);
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    if (rabcl::Can::UpdateData(RxHeader.StdId, RxData, robot_data))
    {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  yaw_motor = new rabcl::JGA25_370(1000 - 1, 75.0 * (38.0 / 25.0));
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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // ---start PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // ---start interrupt processing
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_CAN_Start(&hcan);
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  TIM2->CNT = 32767;
  HAL_TIM_Base_Start_IT(&htim15);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
