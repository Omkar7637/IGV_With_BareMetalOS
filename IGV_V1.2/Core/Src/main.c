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
#include "osKernel.c"
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Motor_EN_Port GPIOE
#define M_R_EN_A GPIO_PIN_8
#define M_R_EN_B GPIO_PIN_7
#define M_L_EN_A GPIO_PIN_10
#define M_L_EN_B GPIO_PIN_9

#define Motor_Pin_Port GPIOD
#define M_R_F_In_1 GPIO_PIN_0
#define M_R_F_In_2 GPIO_PIN_1
#define M_R_R_In_1 GPIO_PIN_3
#define M_R_R_In_2 GPIO_PIN_2

#define M_L_F_In_1 GPIO_PIN_6
#define M_L_F_In_2 GPIO_PIN_7
#define M_L_R_In_1 GPIO_PIN_5
#define M_L_R_In_2 GPIO_PIN_4

#define Dir_Servo_Tmr htim3
#define Servo_R_F TIM_CHANNEL_3
#define Servo_R_R TIM_CHANNEL_4
#define Servo_L_F TIM_CHANNEL_2
#define Servo_L_R TIM_CHANNEL_1
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

char str1[20] = "";

int MainCMD = 0;
int dcm_cmd = 0;
int srm_cmd = 0;

int i = 0;
int Servo_R_F_POS = 1180;
int Servo_R_R_POS = 1180;
int Servo_L_F_POS = 1180;
int Servo_L_R_POS = 1180;

int Servo_R_F_POS_D = 0;
int Servo_R_R_POS_D = 0;
int Servo_L_F_POS_D = 0;
int Servo_L_R_POS_D = 0;

int Servo_R_F_POS_Err = 50;
int Servo_R_R_POS_Err = 50;
int Servo_L_F_POS_Err = 0;
int Servo_L_R_POS_Err = -35;

int ServoOp = 0;

/* Function prototypes ------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
void PulseLEDTask(void);
void Servo1Task(void);
void LED4Task(void);
void DCMotorTask(void);

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&str1, 3);
    MainCMD = atoi(str1);
    sprintf(str1, "%d\r", MainCMD);
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)str1, strlen(str1));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // Handle UART TX complete interrupt
}

/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 1 */
    char str[20] = "Hello DESD\r\n";
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)str, strlen(str));
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&str1, 3);

    HAL_TIM_PWM_Start(&Dir_Servo_Tmr, Servo_R_F);
    HAL_TIM_PWM_Start(&Dir_Servo_Tmr, Servo_R_R);
    HAL_TIM_PWM_Start(&Dir_Servo_Tmr, Servo_L_F);
    HAL_TIM_PWM_Start(&Dir_Servo_Tmr, Servo_L_R);

    /* Create tasks */
    /* Create tasks using BareMetal OS API */
    BMOS_CreateTask(PulseLEDTask, "PulseLED", 128, NULL, 5);
    BMOS_CreateTask(DCMotorTask, "DCMotor", 128, NULL, 5);
    BMOS_CreateTask(LED4Task, "LED4", 128, NULL, 5);
    BMOS_CreateTask(Servo1Task, "Servo1", 128, NULL, 5);

    /* USER CODE END 2 */

    /* Start scheduler */
    BMOS_StartScheduler();

    /* Infinite loop */
    while (1)
    {
        /* USER CODE BEGIN WHILE */
        /* Your main loop code here */
        /* USER CODE END WHILE */
    }
}

/* USER CODE BEGIN 3 */

void PulseLEDTask(void)
{
    while (1)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
        BMOS_Delay(1000);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
        BMOS_Delay(1000);
    }
}

void Servo1Task(void)
{
    while (1)
    {
        if (srm_cmd >= 21 && srm_cmd <= 23)
        {
            if (srm_cmd == 21)
            {
                Servo_R_F_POS_D = Servo_R_F_POS + Servo_R_F_POS;
            }

            if (Servo_R_F_POS < (800 + Servo_R_F_POS_Err))
            {
                for (Servo_R_F_POS = Servo_R_F_POS; Servo_R_F_POS <= (800 + Servo_R_F_POS_Err); Servo_R_F_POS++)
                {
                    ServoOp = 0;
                    __HAL_TIM_SET_COMPARE(&Dir_Servo_Tmr, Servo_R_F, Servo_R_F_POS);
                    BMOS_Delay(1);
                }
            }
            else if (Servo_R_F_POS > 800)
            {
                for (Servo_R_F_POS = Servo_R_F_POS; Servo_R_F_POS >= 800; Servo_R_F_POS--)
                {
                    ServoOp = 0;
                    __HAL_TIM_SET_COMPARE(&Dir_Servo_Tmr, Servo_R_F, Servo_R_F_POS);
                    BMOS_Delay(1);
                }
            }
            else
            {
                ServoOp = 1;
            }
        }
        __HAL_TIM_SET_COMPARE(&Dir_Servo_Tmr, Servo_R_F, Servo_R_F_POS);
        __HAL_TIM_SET_COMPARE(&Dir_Servo_Tmr, Servo_L_F, Servo_L_F_POS);
        __HAL_TIM_SET_COMPARE(&Dir_Servo_Tmr, Servo_R_R, Servo_R_R_POS);
        __HAL_TIM_SET_COMPARE(&Dir_Servo_Tmr, Servo_L_R, Servo_L_R_POS);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
        BMOS_Delay(1000);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
        BMOS_Delay(1000);
    }
}

void LED4Task(void)
{
    while (1)
    {
        if (MainCMD <= 20)
        {
            dcm_cmd = MainCMD;
        }
        else if (MainCMD >= 20 && MainCMD <= 35)
        {
            srm_cmd = MainCMD;
        }
        BMOS_Delay(1);
    }
}

void DCMotorTask(void)
{
    while (1)
    {
        HAL_GPIO_WritePin(Motor_EN_Port, M_R_EN_A | M_R_EN_B | M_L_EN_A | M_L_EN_B, 1);

        if (dcm_cmd == 1)
        {
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_F_In_1, 0);
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_F_In_2, 1);
        }
        else if (dcm_cmd == 2)
        {
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_F_In_1, 0);
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_F_In_2, 0);
        }
        else if (dcm_cmd == 3)
        {
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_F_In_1, 1);
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_F_In_2, 0);
        }

        else if (dcm_cmd == 4)
        {
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_R_In_1, 0);
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_R_In_2, 1);
        }
        else if (dcm_cmd == 5)
        {
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_R_In_1, 0);
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_R_In_2, 0);
        }
        else if (dcm_cmd == 6)
        {
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_R_In_1, 1);
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_R_In_2, 0);
        }
        else
        {
            HAL_GPIO_WritePin(Motor_Pin_Port, M_R_F_In_1 | M_R_F_In_2 | M_R_R_In_1 | M_R_R_In_2, 0);
        }
        BMOS_Delay(1000);
    }
}

/* USER CODE END 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
