/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START      HAL_GPIO_ReadPin(GPIOG, START_Pin) // get START value
#define STOP       HAL_GPIO_ReadPin(GPIOG, STOP_Pin)  // get STOP value
#define E_STOP     HAL_GPIO_ReadPin(GPIOG, E_STOP_Pin) // get EMERGENCY STOP value
#define IR_LED_ON  HAL_GPIO_WritePin(GPIOD, IR_LED_Pin, GPIO_PIN_SET)   // IR LED off
#define IR_LED_OFF HAL_GPIO_WritePin(GPIOD, IR_LED_Pin, GPIO_PIN_RESET) // IR LED off
#define LED_ON     HAL_GPIO_WritePin(GPIOD, LED_Pin,    GPIO_PIN_SET)   // LED off
#define LED_OFF    HAL_GPIO_WritePin(GPIOD, LED_Pin,    GPIO_PIN_RESET) // LED off
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
 uint8_t  state; // state of process
 uint32_t time1, time2; // input capture times
 uint32_t diff; // difference between time1 and time2
 uint32_t f;    // frequency of input signal
 int16_t  dutycycle; // duty cycle percentage 
 uint16_t highduty, lowduty; // high & low duty cycle values
 
 uint8_t array[3];
 uint8_t count;
 uint8_t distance;
 uint8_t remainder;
 uint8_t firstDigit, secondDigit;
 char distanceString[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */

void Sendcmd(uint8_t cmd)
{
    array[0] = 0xF8;     //send 1111 1000 to sync LCD
    array[1] = (cmd & 0xF0);
    array[2] = ( (cmd << 4) & 0xF0);

    HAL_GPIO_WritePin(GPIOE,RS_Pin,GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi4,array,3,1);
    HAL_GPIO_WritePin(GPIOE,RS_Pin,GPIO_PIN_RESET);
    
}

void sendData(uint8_t temp)
{
    array[0] = 0xFA;  //send 1111 1010 to write to LCD (Rs=1, RW=0)
    array[1] = (temp & 0xF0);
    array[2] = ( (temp << 4) & 0xF0);

    HAL_GPIO_WritePin(GPIOE,RS_Pin,GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi4,array,3,1);
    HAL_GPIO_WritePin(GPIOE,RS_Pin,GPIO_PIN_RESET);
 
    
}

void startup()
{

   HAL_GPIO_WritePin(GPIOE,Reset_Pin,GPIO_PIN_RESET);
   HAL_Delay(100); //Wait >40ms after power is applied
   HAL_GPIO_WritePin(GPIOE,Reset_Pin,GPIO_PIN_SET);
   HAL_Delay(10); //wait > 100us
   Sendcmd(0x30); //wake up
   HAL_Delay(10); //wait > 100us
   Sendcmd(0x30); //wakeup
   HAL_Delay(10); //wait >37us
   Sendcmd(0x0C); //Display ON Cursor on 
   HAL_Delay(10); //Wait >100us
   Sendcmd(0x01); //Display Clear
   HAL_Delay(15); //wait >10ms
   Sendcmd(0x06);  
   HAL_Delay(10);
}

void writeString(char* string)
{
  int length = strlen(string);
    for(int i = 0; i < length; i++)
    {
      sendData(string[i]);
    }
 }

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
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  startup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
   if (E_STOP) 
   {
   
      if (START == 0 && state == 0 && E_STOP) // START is active low
      {
        HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
        TIM4->ARR = 20000-1;
        highduty = 10000;
        lowduty = 10000;
        HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
        for(int i = 0; i < 6; i++) // Blink LED 6 times
        {
          LED_ON;
          HAL_Delay(500); // on for .5 seconds
          LED_OFF;
          HAL_Delay(500); // off for .5 seconds
        }
        state = 1;
        HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
      }
      if (START == 0 && state == 1 && E_STOP)
      {
        IR_LED_ON;
        TIM4->ARR = 3000-1;
        highduty = 1500;
        lowduty = 1500;
        HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
        state = 2;
      }
      
      
      
      
      if ((state == 2) && E_STOP)
      {
       if (time1 > time2)
        { 
          diff = (time1-time2);  
          f = 90000000/diff;  
        }  
        else if (time2 > time1)
        {   
          diff = (time2-time1);   
          f = 90000000/diff;
        }
      
        dutycycle = (-f/20 + 325);
        highduty  = (uint16_t)dutycycle*3000/100 + 24;
        lowduty   = (100 - (uint16_t)dutycycle)*3000/100 - 24;

        if (HAL_GPIO_ReadPin(GPIOD, REVERSE_Pin))
        {
        HAL_Delay(100);
        Sendcmd(0x90);
        writeString("Moving Backward");
  //      Sendcmd(0x88);
       // distance = (int)0.5*count + 0x30;  
  //      distance = (count % 25 ); 
  //      sprintf(distanceString, "%x", distance);
  //      writeString(distanceString);
  //      remainder = distance;
  //      firstDigit = remainder%10;
  //      remainder = (remainder - firstDigit)/10;
  //      secondDigit = remainder;
  //     // distance = 'y';
  //      sendData(secondDigit + 0x30);
  //      sendData(firstDigit  + 0x30);
        }
        
        else
        {
        HAL_Delay(100);
        Sendcmd(0x90);
        writeString("Moving  Forward");
  //      Sendcmd(0x88);
  //      distance = (count % 25 ); 
  //      sprintf(distanceString, "%x", distance);
  //      writeString(distanceString);
        }      
        
        if (STOP == 0) // STOP is active low
        {
          HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_2); 
          HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1); 
          
          TIM4->ARR = 25000-1;
          highduty = 12500;
          lowduty = 12500;
          HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
          
          Sendcmd(0x01); //clear screen;
          HAL_Delay(100);
          Sendcmd(0x90);
          writeString("Halted");
          
          for(int i = 0; i < 10; i++) // Blink LED 6 times
          {
            LED_ON;
            HAL_Delay(250); // on for .5 seconds
            LED_OFF;
            HAL_Delay(250); // off for .5 seconds
          }
          IR_LED_OFF;
          HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
          state = 0;
          Sendcmd(0x01); //clear screen;
        }   
          
      }
     
   }
   
   else {
        HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_2); 
        HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);
        LED_OFF;
        IR_LED_OFF;
        HAL_Delay(100);
        Sendcmd(0x90);
        writeString("Emergency stop ");  
   }
    
    
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RS_Pin|Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_Pin|IR_LED_Pin|FORWARD_Pin|REVERSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin Reset_Pin */
  GPIO_InitStruct.Pin = RS_Pin|Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin IR_LED_Pin FORWARD_Pin REVERSE_Pin */
  GPIO_InitStruct.Pin = LED_Pin|IR_LED_Pin|FORWARD_Pin|REVERSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DIRECTION_Pin */
  GPIO_InitStruct.Pin = DIRECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIRECTION_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : E_STOP_Pin START_Pin STOP_Pin */
  GPIO_InitStruct.Pin = E_STOP_Pin|START_Pin|STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : DIRECTIONG12_Pin */
  GPIO_InitStruct.Pin = DIRECTIONG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIRECTIONG12_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
