/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l476xx.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_BUF_SIZE 300
#define GNGGA_SIZE 42
#define LORA_BUF_SIZE 240
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


extern uint8_t tx_data_lora[LORA_BUF_SIZE]; 
extern uint8_t rx_data_lora[LORA_BUF_SIZE];

extern uint8_t rx_data_gps[GPS_BUF_SIZE];
extern char gngga[GNGGA_SIZE];

extern const uint8_t stopCommand[18200];
extern const uint8_t  whistle[24580];
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_dac_ch1;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DAC_HandleTypeDef hdac1;
extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac_ch1);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */


  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */


  printf("Start of GPS interrupt\r\n");
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

  bzero(rx_data_gps, GPS_BUF_SIZE); //clear tx buffer
  // printf("GPS DATA: \n\r");
  while(HAL_UART_Receive(&huart1, rx_data_gps, GPS_BUF_SIZE, 6000)!=HAL_OK){} //wait until received
  // HAL_UART_Receive(&huart1, rx_data_gps, GPS_BUF_SIZE, 100000); //wait until received
  // printf("trasnmitting \n\r");
  // HAL_UART_Transmit(&huart2, rx_data_gps, GPS_BUF_SIZE, 10); //print received data to terminal  
  // printf("After GPS receive\n\r");

  char *gngga_loc = strstr((char *)rx_data_gps, "$GNGGA");
  
  bzero(gngga, GNGGA_SIZE); //clear gpgga buffer
  memcpy(gngga, gngga_loc, GNGGA_SIZE); //copy command to tx buffer
  if(gngga_loc != NULL){
    printf("GNGGA DATA: %s\n\r", gngga);

    if(gngga[18] == ','){ //No fix found
      // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
    }else{ //fix found
      // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
      
      char txsend[55] = "AT+SEND=123,42,";
      strcat(txsend, gngga);
      strcat(txsend, "\r\n");
      printf("RX gps: %s", txsend);
      
      HAL_UART_Transmit(&huart3, txsend, strlen(txsend), 1000);
      bzero(rx_data_lora, LORA_BUF_SIZE);
      printf("Done Sending\r\n");
    }
      
    
  }
  else{
    printf("None found\n\r");
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
  }
  HAL_IWDG_Refresh(&hiwdg);

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
