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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define AC line frequency (50Hz or 60Hz)
#define AC_LINE_FREQUENCY 60  // Change to 60 for 60Hz systems

// Calculate half-cycle period in microseconds based on frequency
#define HALF_CYCLE_PERIOD_US (1000000 / (AC_LINE_FREQUENCY * 2))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t dimmerValue = 0;         // Dimmer value (0-100%)
char rxBuffer[10];              // Buffer to store received characters
uint8_t rxIndex = 0;            // Index for received characters
uint8_t rxComplete = 0;         // Flag to indicate reception complete
uint8_t rxData;                 // Single byte for UART reception

// Variables for AC dimming control
volatile uint16_t dimmerDelayUs = 9999;  // Calculated delay time in microseconds
volatile uint8_t triacTriggered = 0;  // Flag to indicate if triac has been triggered in current half-cycle

uint16_t ADC_VAL[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ProcessReceivedValue(void); // Function to process received data
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Process the received string and convert it to a dimmer value (0-100%)
void ProcessReceivedValue(void)
{
  // Null-terminate the string
  rxBuffer[rxIndex] = '\0';
  
  // Convert string to integer
  uint32_t tempValue = atoi(rxBuffer);
  
  // Limit to range (0-100)
  if (tempValue > 100) {
    tempValue = 100;
  }
  
  // Update dimmer value
  dimmerValue = (uint16_t)tempValue;
  
  // Convert value 1 to 2 for more stable operation
  if (dimmerValue == 1) {
    dimmerValue = 2;
  }
  
  // Calculate dimming delay in microseconds
  // Map dimmer value 0-100 to delay HALF_CYCLE_PERIOD_US-0 (inverted, 0% = full off, 100% = full on)
  if (dimmerValue == 0) {
    // If dimmer is 0%, keep SSR off
    dimmerDelayUs = HALF_CYCLE_PERIOD_US; // Set to max delay (never trigger)
  } else if (dimmerValue == 100) {
    // If dimmer is 100%, keep SSR fully on
    dimmerDelayUs = 0;     // No delay (trigger immediately)
  } else {
    // Calculate delay (0% = HALF_CYCLE_PERIOD_US delay, 100% = 0us delay)
    // Ensure the delay is within valid range for the timer
    dimmerDelayUs = HALF_CYCLE_PERIOD_US - ((dimmerValue * HALF_CYCLE_PERIOD_US) / 100);
    
    // Make sure we have a valid delay value (not 0 or too large)
    if (dimmerDelayUs >= HALF_CYCLE_PERIOD_US) {
      dimmerDelayUs = HALF_CYCLE_PERIOD_US - 100; // Leave a small margin
    } else if (dimmerDelayUs == 0) {
      dimmerDelayUs = 1;
    }
  }
  
  // Send confirmation back to user
  char txBuffer[50];
  sprintf(txBuffer, "Dimmer value set to: %u%% (Delay: %u us)\r\n", dimmerValue, dimmerDelayUs);
  HAL_UART_Transmit(&huart1, (uint8_t*)txBuffer, strlen(txBuffer), 100);
  
  // Reset reception variables
  rxIndex = 0;
  rxComplete = 0;
  memset(rxBuffer, 0, sizeof(rxBuffer));
}

// UART Reception Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    // Check for Enter key (CR or LF)
    if (rxData == '\r' || rxData == '\n') {
      if (rxIndex > 0) {
        rxComplete = 1;
      }
    }
    // Check for backspace
    else if (rxData == 127 || rxData == 8) {
      if (rxIndex > 0) {
        rxIndex--;
      }
    }
    // Check if the character is a digit and buffer isn't full
    else if (rxData >= '0' && rxData <= '9' && rxIndex < sizeof(rxBuffer) - 1) {
      rxBuffer[rxIndex++] = rxData;
      
      // Echo the character back to the terminal
      HAL_UART_Transmit(&huart1, &rxData, 1, 10);
    }
    
    // Start receiving next character
    HAL_UART_Receive_IT(&huart1, &rxData, 1);
  }
}

// Timer1 Update Interrupt Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    // This is called when the timer reaches the set delay time
    // Turn on the SSR
    HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin, GPIO_PIN_SET);
    
    // Set flag to indicate triac has been triggered in this half-cycle
    triacTriggered = 1;
    
    // Stop the timer
    HAL_TIM_Base_Stop_IT(&htim1);
    
    // Debug output - send trigger confirmation via UART
//    char debugMsg[50];
//    sprintf(debugMsg, "Triggered at delay: %u us\r\n", dimmerDelayUs);
//    HAL_UART_Transmit(&huart1, (uint8_t*)debugMsg, strlen(debugMsg), 10);
  }
}

// GPIO EXTI Callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check if this is our zero-crossing pin
  if (GPIO_Pin == GPIO_PIN_13) {
    // Debug output - send zero-crossing detection via UART
    // Uncomment for debugging
    // char zcdMsg[] = "ZCD detected\r\n";
    // HAL_UART_Transmit(&huart1, (uint8_t*)zcdMsg, strlen(zcdMsg), 10);
    
    // Reset triac triggered flag at each zero crossing
    triacTriggered = 0;
    
    // Turn off SSR at zero crossing if dimmer is not 100%
    if (dimmerValue < 100) {
      HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin, GPIO_PIN_RESET);
      
      // If dimmer value is 0%, don't trigger the timer
      if (dimmerValue > 0) {
        // Make sure timer is stopped before reconfiguring
        HAL_TIM_Base_Stop_IT(&htim1);
        
        // Set up Timer1 for one-shot operation with the calculated delay
        __HAL_TIM_SET_COUNTER(&htim1, 0);  // Reset counter
        __HAL_TIM_SET_AUTORELOAD(&htim1, dimmerDelayUs);  // Set delay in microseconds
        
        // Start the timer in interrupt mode
        HAL_TIM_Base_Start_IT(&htim1);
      }
    } else {
      // If dimmer is 100%, keep SSR fully on
      HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin, GPIO_PIN_SET);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Enable EXTI Line 13 interrupt for zero-crossing detection
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
  // Initialize SSR pin to OFF state
  HAL_GPIO_WritePin(SSR1_GPIO_Port, SSR1_Pin, GPIO_PIN_RESET);
  
  // Initialize Timer1 for microsecond timing
  // Stop any running timer operations first
  HAL_TIM_Base_Stop(&htim1);
  HAL_TIM_Base_Stop_IT(&htim1);
  
  // Enable Timer1 interrupt
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  
  // Start UART reception in interrupt mode
  HAL_UART_Receive_IT(&huart1, &rxData, 1);
  
  // Send welcome message
  char welcomeMsg[] = "\r\nAC Dimmer Control (0-100): ";
  HAL_UART_Transmit(&huart1, (uint8_t*)welcomeMsg, strlen(welcomeMsg), 100);

  HAL_ADC_Start_DMA(&hadc1, ADC_VAL, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastPrintTime = 0;
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Check if we have received a complete value
    if (rxComplete) {
      ProcessReceivedValue();
      
      // Prompt for next input
      char promptMsg[] = "Enter dimmer value (0-100): ";
      HAL_UART_Transmit(&huart1, (uint8_t*)promptMsg, strlen(promptMsg), 100);
    }
    
    // Print current sensor values every 500ms
    uint32_t currentTime = HAL_GetTick();
    if (currentTime - lastPrintTime >= 500) {
      lastPrintTime = currentTime;
      
      // Convert ADC values to milliamperes
      // Formula: Current(mA) = (ADC_Value - ZeroCurrentADC) * (3300mV / 4096) / (135mV/A) * 1000
      // Where: 
      // - 3300mV is the ADC reference voltage
      // - 4096 is the ADC resolution (12-bit)
      // - 135mV/A is the sensor sensitivity
      // - ZeroCurrentADC is the ADC value when no current is flowing (2300)
      // - 1000 is to convert from A to mA
      
      int32_t current1_mA = ((int32_t)ADC_VAL[0] - 2300) * 3300 / 4096 * 1000 / 135;
      int32_t current2_mA = ((int32_t)ADC_VAL[1] - 2300) * 3300 / 4096 * 1000 / 135;
      int32_t current3_mA = ((int32_t)ADC_VAL[2] - 2300) * 3300 / 4096 * 1000 / 135;
      int32_t current4_mA = ((int32_t)ADC_VAL[3] - 2300) * 3300 / 4096 * 1000 / 135;
      
      // Format and print the current sensor values in milliamperes
      char sensorMsg[150];
      sprintf(sensorMsg, "\r\nCurrent Sensors: %ld mA, %ld mA, %ld mA, %ld mA | Pressure: %u\r\n", 
              current1_mA, current2_mA, current3_mA, current4_mA, ADC_VAL[4]);
      HAL_UART_Transmit(&huart1, (uint8_t*)sensorMsg, strlen(sensorMsg), 100);
    }
    
    // The AC dimming is now handled entirely by interrupts:
    // 1. Zero-crossing detection triggers EXTI13 interrupt
    // 2. EXTI13 handler starts Timer1 with the appropriate delay
    // 3. Timer1 interrupt triggers the SSR after the delay
    // This provides microsecond-level precision for AC phase control
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
