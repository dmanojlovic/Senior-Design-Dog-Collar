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
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l476xx.h"
#include <stdint.h>
#include <stdio.h>
// #include "stm32l4xx_hal_dac.h" //changed dac to ll not hal

#include "adpcm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_STARTUP_VAL (0x80)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void LoadAudioFiles(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
AudioElement AudioFile;
uint8_t AudioFileToPlay = 0;

uint8_t rx_data[1];
void led2_test(void);

uint8_t rx_data_gps[100]; //

uint8_t tx_data_lora[4] = {'A', 'T', '\r', '\n'}; //"AT\r\n";
uint8_t rx_data_lora[5]; //expects "+OK\r\n"

uint16_t dac_value=0;
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
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

  // uint8_t test = 5;
  // printf("Characters: %c %c\n\r", 'a', 65);
  // printf("Decimals: %d %ld\n\r", 1977, 650000L);
  // printf("Preceding with blanks: %10d\n\r", 1977);
  // printf("Preceding with zeros: %010d\n\r", 1977);
  // printf("Some different radices: %d %x %o %#x %#o\n\r", 100, 100, 100, 100, 100);
  // printf("floats: %4.2f %+.0e %E\n\r", 3.1416, 3.1416, 3.1416);
  // printf("Width trick: %*d\n\r", 5, 10);
  // printf("%s\n\r", "A string");
  // printf("unsingned int8 = %d\n\r", test);
  // printf("END OF PRINTS\r\n");

  printf("START OF CODE\r\n\n");
  // led2_test();

  // HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

  LoadAudioFiles();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // capture/compare registers (CC1 PWM duty 50%)
//  	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
//  	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x7FF);

	TIM2->CCR1 = DEFAULT_STARTUP_VAL;
	TIM2->CCR2 = DEFAULT_STARTUP_VAL;
	LL_TIM_EnableIT_UPDATE(TIM2);
	TIM2->CCER |= TIM_CCER_CC2E | TIM_CCER_CC1E;
	LL_TIM_EnableCounter(TIM2);
  while (1)
  {
    //print character in console ######################################################################
    // HAL_UART_Receive_IT(&huart2, rx_data, 1); //receive character from 

    // LoRA ###########################################################################################
    //  HAL_UART_Transmit_IT(&huart5, tx_data_lora, 4);
    // HAL_UART_Transmit(&huart5, tx_data_lora, 4, 1000);
    // if(HAL_UART_Receive(&huart5, rx_data_lora, 5, 1000)==HAL_OK) //if transfer is successful
    // { 
    //   HAL_UART_Transmit(&huart2, rx_data_lora, 5, 10);
    //   __NOP(); //You need to toggle a breakpoint on this line!
    // } else {
    //   printf("Transfer Failed\n\r");
    //   __NOP();
    // }

    //GPS ############################################################################################
    // HAL_UART_Receive_IT(&huart5, rx_data_gps, 1000);
    // for(int i = 0; i < 100; i++){
    //   printf("%c", rx_data_gps[i]);
    // }
    // printf("\n");

    //Flashing LED ###################################################################################
    // GPIOA->ODR ^= 0x0020;

    //DAC ############################################################################################
    // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value);
    // if (dac_value < 4095) {
    //   dac_value++;
    // } else {
    //   dac_value=0;
    // }
    // printf("DAC value: %d\n\r", dac_value);

    //PWM audio ######################################################################################
//    if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) //play audio while button is pushed
//		{
//			while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));
//			if(AudioFileToPlay>=1)
//			{
//				AudioFileToPlay = 0;
//			}else
//			{
//				AudioFileToPlay++;
//			}
//			/* Disable the TIM2 Interrupt */
//			NVIC_EnableIRQ(TIM2_IRQn);
//			// stop the timer
//			LL_TIM_EnableCounter(TIM2);
//		}

    if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){ //test if button is pushed
      // GPIOC->BSRR = GPIO_BSRR_BS3;
      GPIOA->BSRR = GPIO_BSRR_BS5;
    }
    else{
      // GPIOC->BSRR = GPIO_BSRR_BR3;
      GPIOA->BSRR = GPIO_BSRR_BR5;
    }

		// HAL_Delay(100);


//     HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) //play audio while button is pushed
	{
		while(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));

		if(AudioFileToPlay>=1)
		{
			AudioFileToPlay = 0;
		}else
		{
			AudioFileToPlay++;
		}
		/* Disable the TIM2 Interrupt */
		NVIC_EnableIRQ(TIM2_IRQn);
//		NVIC_DisableIRQ(TIM2_IRQn);
		// stop the timer
		LL_TIM_EnableCounter(TIM2);
//		LL_TIM_DisableCounter(TIM2);
	}
    HAL_Delay(100);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0)
  {
  }
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(80000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
  if(huart->Instance == USART2){ //COM port for printing
    if(rx_data[0] == '\r'){
      uint8_t temp[1];
      temp[0] = '\n';
      HAL_UART_Transmit(&huart2, temp, 1, 10); 
      HAL_UART_Transmit(&huart2, rx_data, 1, 10); 
    }
    else{
      HAL_UART_Transmit(&huart2, rx_data, 1, 10); 
    }
  }
  else if(huart->Instance == UART5){ //LoRa
    printf("UART5 RX HERE\n\r");
  } 
  else if(huart->Instance == UART4){ //GPS
    printf("UART4 RX HERE\n\r");
  } 
  else{
    printf("ERROR: WRONG UART RX\n\r");
  }
  
  
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_TxCpltCallback can be implemented in the user file.
   */
  if(huart->Instance == USART2){ //COM port for printing
     printf("USART2 TX HERE\n\r");
  }
  else if(huart->Instance == UART5){ //LoRa
    // printf("UART5 TX HERE\n\r");
     if(HAL_UART_Receive(&huart5, rx_data_lora, 5, HAL_MAX_DELAY)==HAL_OK) //if transfer is successful
    { 
      HAL_UART_Transmit(&huart2, rx_data_lora, 5, 10);
      __NOP(); //You need to toggle a breakpoint on this line!
    } else {
      printf("Transfer Failed\n\r");
      __NOP();
    }
  } 
  else if(huart->Instance == UART4){ //GPS
    printf("UART4 TX HERE\n\r");
  } 
  else{
    printf("ERROR: WRONG UART TX\n\r");
  }

}

// void led2_test(){
//   RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
//   GPIOA->MODER &= 0xfffff3ff;
//   GPIOA->MODER |= GPIO_MODER_MODE5_0; //0x00000400
//   GPIOA->BSRR = 1 << 5;
// }

void LoadAudioFiles(void)
{
	AudioFile.AudioFiles[0] = (uint32_t)&Stop_Command_3;
	AudioFile.AudioSize[0] = NELEMS(Stop_Command_3);
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
