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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l476xx.h"
#include "stm32l4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
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

//LoRa uart and dma enable
void init_usart3(void); 
void enable_tty_interrupt(void);

//LoRa settings and sending data
void send_lora(char *, int, int);
void setup_lora(void);

//timer for GPS interrupt
void init_tim7();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//LoRa uart rolling buffer
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

//LoRa buffers
#define LORA_BUF_SIZE 240
uint8_t tx_data_lora[LORA_BUF_SIZE]; 
uint8_t tx_received[24] = "AT+SEND=123,8,Received\r\n"; 
// uint8_t rx_data_lora[LORA_BUF_SIZE];

//GPS 
#define GNGGA_SIZE 36
uint8_t rx_data_gps[GNGGA_SIZE]; 
uint8_t isG;
uint8_t gpsbuf;

//Audio Sounds
const uint8_t zero[1] = {0};
extern const uint8_t stopCommand[18200];
extern const uint8_t whistle[24580];
extern const uint8_t Se7enNo[19316];

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
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  printf("Start of code\r\n");

  bzero(serfifo, FIFOSIZE);
  init_usart3();
  enable_tty_interrupt();
  setup_lora();

  init_tim7();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);

  HAL_TIM_Base_Start_IT(&htim2);

  // HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint8_t*)zero, 1, DAC_ALIGN_8B_R);

  // HAL_Delay(5000);
  // HAL_TIM_Base_Start_IT(&htim7);
  

  // HAL_NVIC_SetPriority(TIM7_IRQn, 15, 0);
  // HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // while (!(USART3->ISR & USART_ISR_RXNE)) { }
    // char c = USART3->RDR;
    // while(!(USART3->ISR & USART_ISR_TXE)) { }
    // USART3->TDR = c;

    // HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint8_t*)stopCommand, 18200, DAC_ALIGN_8B_R);
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(1000);
    HAL_IWDG_Refresh(&hiwdg);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 1875;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1250;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void setup_lora(void){
  
  HAL_Delay(200);
  send_lora("AT\r\n", 4, 5); //basic stm to lora check (rcv: +OK)
  // HAL_Delay(200);
  // send_lora("AT+FACTORY\r\n", 12, 8); //set to factory default (rcv: +FACTORY)
  HAL_Delay(200);
  send_lora("AT+ADDRESS=124\r\n", 16, 5); //set lora address (rcv: +OK)
  HAL_Delay(200);
  send_lora("AT+NETWORKID=7\r\n", 16, 5); //set network id (rcv: +OK)
  HAL_Delay(200);
  send_lora("AT+CPIN=102C064CA409E69030F73E7CABAA4B71\r\n", 42, 5); //set AES pin (rcv: +OK)
  HAL_Delay(200);

  printf("Done setting up LoRa\r\n");
  HAL_IWDG_Refresh(&hiwdg);
}

void send_lora(char *msg, int size, int resp_size){
  bzero(tx_data_lora, LORA_BUF_SIZE);
  // bzero(rx_data_lora, LORA_BUF_SIZE);
  memcpy(tx_data_lora, msg, size);

  for(int i = 0; i < size; i++){ //transmit each char, one at a time
    while(!(USART3->ISR & USART_ISR_TXE)) { }
    USART3->TDR = tx_data_lora[i];
  }
  HAL_IWDG_Refresh(&hiwdg);

  // while(HAL_UART_Receive(&huart3, rx_data_lora, resp_size, 1000)!=HAL_OK){}
  // HAL_UART_Transmit(&huart2, serfifo, resp_size, 10);
}


void init_usart3() { //LoRa
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; //turn on GPIO C
    GPIOC->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5); //clear pc12
    GPIOC->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1; //PC12 as alternate function

    GPIOC->AFR[0] &= ~0x00ff0000; //AF7
    GPIOC->AFR[0] |=  0x00770000; 

    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN; //enable usart5 clock
    USART3->CR1 &= ~USART_CR1_UE; //disable usart3

    USART3->CR1 &= ~USART_CR1_M; //set word size to 8 by clearing M0 and M1
    USART3->CR2 &= ~USART_CR2_STOP; //one stop bit
    USART3->CR1 &= ~USART_CR1_PCE; //disable parity bit
    USART3->CR1 &= ~USART_CR1_OVER8; //16x oversampling
    USART3->BRR = 40000000/115200; //Baud rate of 115200 (table 249)
    USART3->CR1 |= USART_CR1_TE; //enable transmitter
    USART3->CR1 |= USART_CR1_RE; //enable receiver

    USART3->CR1 |= USART_CR1_UE; //enable usart5

    while(!(USART3->ISR & (USART_ISR_TEACK | USART_ISR_REACK))); //wait for TE and RE to be acknowledged
    HAL_IWDG_Refresh(&hiwdg);
}

void enable_tty_interrupt(void) {
    // TODO
    USART3->CR1 |= USART_CR1_RXNEIE; //enable interupt when receive data register becomes not empty.
    NVIC->ISER[1] = 1 << (USART3_IRQn-32);

    USART3->CR3 |= USART_CR3_DMAR; //enable DMA mode
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; //Enable DMA1 clock

    DMA1_CSELR->CSELR &= ~0x00000f00; //clear channel 3
    DMA1_CSELR->CSELR |=  0x00000200; //select channel 3 to be usart3rx
    DMA1_Channel3->CCR &= ~DMA_CCR_EN; //disable dma1 channel 3

    DMA1_Channel3->CMAR = (uint32_t) serfifo; //set memory address
    DMA1_Channel3->CPAR = (uint32_t) &(USART3->RDR); //set peripheral address
    DMA1_Channel3->CNDTR = FIFOSIZE; //number of elements you are trying to transfer
    DMA1_Channel3->CCR &= ~DMA_CCR_DIR; //dir set to from-p-to-m
    DMA1_Channel3->CCR |= DMA_CCR_MINC; //minc set to increment memory
    DMA1_Channel3->CCR &= ~DMA_CCR_PINC; //pinc disabled
    DMA1_Channel3->CCR &= ~(DMA_CCR_PSIZE | DMA_CCR_MSIZE); //MSIZE and PSIZE set to 8 bits
    DMA1_Channel3->CCR |= DMA_CCR_CIRC; //circ enabled
    DMA1_Channel3->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE); //disable total and half transfer interrupt
    DMA1_Channel3->CCR &= ~DMA_CCR_MEM2MEM; //disable mem2mem
    DMA1_Channel3->CCR |= DMA_CCR_PL; //set priority level to highest

    DMA1_Channel3->CCR |= DMA_CCR_EN; //enable dma1 channel 3
}

void USART3_IRQHandler(void){
  // while(DMA1_Channel3->CNDTR != sizeof serfifo - seroffset) //printing character
  // {
  //   while(!(USART3->ISR & USART_ISR_TXE)) { }
  //   USART3->TDR = serfifo[seroffset];
  //   seroffset = (seroffset + 1) % sizeof serfifo;
  // }

  // while(DMA1_Channel3->CNDTR != sizeof serfifo - seroffset)
  // { 
  //   // while(!(USART3->ISR & USART_ISR_TXE)) { }
  //   // USART3->TDR = serfifo[seroffset];
  //   // HAL_UART_Transmit(&huart2, serfifo[seroffset], 1, 10);
  //   printf("Hello ");

  //   seroffset = (seroffset + 1) % sizeof serfifo;
  // }

  // printf("%c", serfifo[seroffset]);
  // if(seroffset == 0){
  //   printf("%c%c", serfifo[FIFOSIZE-1], serfifo[seroffset]);
  // }else{
  //   printf("%c%c", serfifo[seroffset-1], serfifo[seroffset]);
  // }

  while(DMA1_Channel3->CNDTR != sizeof serfifo - seroffset){
    printf("%c", serfifo[seroffset]);
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    uint8_t audio_cue = serfifo[seroffset];
    // printf("%c", audio_cue);

    if(audio_cue == 'a'){
      // printf("Audio %c", audio_cue);
       HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
       HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint8_t*)whistle, 24580, DAC_ALIGN_8B_R);
        for(int i = 0; i < 24; i++){ //send received signal
          while(!(USART3->ISR & USART_ISR_TXE)) { }
          USART3->TDR = tx_received[i];
        }
     }
     else if(audio_cue == 'b'){
      // printf("Audio %c", audio_cue);
       HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
       HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint8_t*)stopCommand, 18200, DAC_ALIGN_8B_R);
        for(int i = 0; i < 24; i++){ //send received signal
          while(!(USART3->ISR & USART_ISR_TXE)) { }
          USART3->TDR = tx_received[i];
        }
     }
     else if(audio_cue == 'd'){
      // printf("Audio %c", audio_cue);
       HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
       // HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint8_t*)noCommand, 22332, DAC_ALIGN_8B_R);
       HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint8_t*)Se7enNo, 19316, DAC_ALIGN_8B_R);
        for(int i = 0; i < 24; i++){ //send received signal
          while(!(USART3->ISR & USART_ISR_TXE)) { }
          USART3->TDR = tx_received[i];
        }

     }
    //  else if(audio_cue == 'D'){
    //    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    //    // HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint8_t*)comeHome, 22908, DAC_ALIGN_8B_R);
    //  }
    
    // if((audio_cue == 'A') || (audio_cue == 'B')||(audio_cue == 'D')){
    //   for(int i = 0; i < 24; i++){ //send received signal
    //     while(!(USART3->ISR & USART_ISR_TXE)) { }
    //     USART3->TDR = tx_received[i];
    //   }
    // }
    
    
    seroffset = (seroffset + 1) % sizeof serfifo;
  }

  // printf("%c", serfifo[seroffset]);
  // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
  // if((serfifo[seroffset] == 'A') || (serfifo[seroffset] == 'B') || (serfifo[seroffset] == 'C')){
  //   HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

  //   for(int i = 0; i < 24; i++){ //send received signal
  //     while(!(USART3->ISR & USART_ISR_TXE)) { }
  //     USART3->TDR = tx_received[i];
  //   }
  //   // send_lora(tx_received, 24, 5);
  //   // printf("Data sent\r\n");
  // }
  
  // seroffset = (seroffset + 1) % sizeof serfifo;
  

  // send_lora(tx_received, 24, 5);
  HAL_IWDG_Refresh(&hiwdg);
}



void TIM7_IRQHandler(void)
{
  TIM7->SR &= ~TIM_SR_UIF; //acknowledge the interrupt

  // printf("Start of GPS interrupt\r\n");
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

  isG = 0;
  while(1){
    HAL_UART_Receive(&huart1, &gpsbuf, 1, 10);
    if(gpsbuf == 'G'){ //G found
      isG = 1;
    }
    else if(isG == 1){
      if(gpsbuf == 'A'){ //GA found
        HAL_UART_Receive(&huart1, rx_data_gps, GNGGA_SIZE, 100);
        break;
      }
      else{
        isG = 0;
      }

    }
  } 

  // printf("GPS DATA: %s\n\r", rx_data_gps);
  // printf("Parsed: %s\n\r", rx_data_gps+12);
  
  
  // HAL_UART_Transmit(&huart3, txsend, strlen(txsend), 1000);
  // bzero(rx_data_lora, GNGGA_SIZE);
  // for(int i = 0; i < 38; i++){ //send received signal
  //   while(!(USART3->ISR & USART_ISR_TXE)) { }
  //   USART3->TDR = txsend[i];
  // }

  

  if(rx_data_gps[12] == ','){ //No fix found
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
  }else{ //fix found
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);

    // uint8_t temp[25] = "4025.7373,N,08654.7737,W";
    uint8_t txsend[41] = "AT+SEND=123,24,";
    strncat(txsend, rx_data_gps+12,24);
    // strncat(txsend, temp,24);
    strncat(txsend, "\r\n",2);
    // printf("RX gps: %s", txsend);

    // uint8_t temp2[41] = "AT+SEND=123,24,4025.7373,N,08654.7737,W\r\n"; 
    for(int i = 0; i < 41; i++){ //send received signal
      while(!(USART3->ISR & USART_ISR_TXE)) { }
      USART3->TDR = txsend[i];
      // USART3->TDR = temp2[i];
    }

    // uint8_t tx_received[24] = "AT+SEND=123,8,Received\r\n"; 
    // for(int i = 0; i < 24; i++){ //send received signal
    //     while(!(USART3->ISR & USART_ISR_TXE)) { }
    //     USART3->TDR = tx_received[i];
    //   }
  }

  bzero(rx_data_gps, GNGGA_SIZE);
  HAL_IWDG_Refresh(&hiwdg);

  /* USER CODE END TIM7_IRQn 1 */
}

void init_tim7() {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN; //enable TIM7 clock

    //period(sec) = (ARR * PSC) / CLK
    int period = 10;
    TIM7->PSC = 20000 - 1; //prescaler
    TIM7->ARR = (2000*period)- 1; //auto reload register
    TIM7->DIER |= TIM_DIER_UIE; //update interrupt enable
    NVIC->ISER[1] |= 1 << (TIM7_IRQn-32); //enable interrupt in nvic
    TIM7->CR1 |= TIM_CR1_CEN; //start timer
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
