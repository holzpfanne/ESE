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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "retarget.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct data{
	char up[11], down[11];
};

struct delaystruct{
	uint16_t time;
	uint8_t producer;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
_Bool full = FALSE;
uint32_t plus = 0, minus = -1;
struct data put;

//input buffer
char input[20] = {0};
char* input_pointer = input;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for Producer1 */
osThreadId_t Producer1Handle;
const osThreadAttr_t Producer1_attributes = {
  .name = "Producer1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Producer2 */
osThreadId_t Producer2Handle;
const osThreadAttr_t Producer2_attributes = {
  .name = "Producer2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Producer3 */
osThreadId_t Producer3Handle;
const osThreadAttr_t Producer3_attributes = {
  .name = "Producer3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Monitor */
osThreadId_t MonitorHandle;
const osThreadAttr_t Monitor_attributes = {
  .name = "Monitor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for FIFO */
osMessageQueueId_t FIFOHandle;
const osMessageQueueAttr_t FIFO_attributes = {
  .name = "FIFO"
};
/* Definitions for delayPipe */
osMessageQueueId_t delayPipeHandle;
const osMessageQueueAttr_t delayPipe_attributes = {
  .name = "delayPipe"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void ProducerTask(void *argument);
void MonitorTask(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void RXISR(UART_HandleTypeDef *huart){
	//write UART to internal buffer for later processing
	HAL_UART_Receive(&huart2, (uint8_t*)input_pointer, 1, 1);
	input_pointer++;
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
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart2);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of FIFO */
  FIFOHandle = osMessageQueueNew (16, sizeof(struct data), &FIFO_attributes);

  /* creation of delayPipe */
  delayPipeHandle = osMessageQueueNew (4, sizeof(struct delaystruct), &delayPipe_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Producer1 */
  Producer1Handle = osThreadNew(ProducerTask, (void*)1, &Producer1_attributes);

  /* creation of Producer2 */
  Producer2Handle = osThreadNew(ProducerTask, (void*)2, &Producer2_attributes);

  /* creation of Producer3 */
  Producer3Handle = osThreadNew(ProducerTask, (void*)3, &Producer3_attributes);

  /* creation of Monitor */
  MonitorHandle = osThreadNew(MonitorTask, NULL, &Monitor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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

  //enable Rx interrupt
  huart2.Instance->CR1 |= USART_CR1_RXNEIE;
  huart2.RxISR = &RXISR;

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ProducerTask */
/**
  * @brief  Function implementing the Producer1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ProducerTask */
void ProducerTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t delaytime = 100;
	struct delaystruct delay;

  /* Infinite loop */
  for(;;)
  {
    memset(&put, 0, sizeof(struct data));
    //change data and write to struct as string
	sprintf(put.up, "%lu", ++plus);
    sprintf(put.down, "%lu", --minus);

    //put data into Queue
    if(osMessageQueuePut(FIFOHandle, &put, 0, 0) == osOK){full = FALSE;}
    else{
    	full = TRUE;
    }
    //check if delaytime Queue has an objet
    if(osMessageQueueGet(delayPipeHandle, &delay, 0, 0) == osOK){
    	if(delay.producer != (int)argument){
    		//is wrong producer, put back into queue
    		osMessageQueuePut(delayPipeHandle, &delay, 0, 0);
    	}
    	else{
    		//set delay
    		delaytime = delay.time;
    	}
    }

	osDelay(delaytime);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MonitorTask */
/**
* @brief Function implementing the Monitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MonitorTask */
void MonitorTask(void *argument)
{
  /* USER CODE BEGIN MonitorTask */
  struct data get;
  int producer = 0, delaytime = 0;
  struct delaystruct delay;

  /* Infinite loop */
  for(;;)
  {
	  //get data from queue and print it
	  if(osMessageQueueGet(FIFOHandle, &get, 0, 0) == osOK){
		  printf(" plus: %s\nminus: %s\n%s", get.up, get.down, full ? "Queue full !!!\n" : "");
	  }
	  //read from UART buffer writen to by interrupt; not using scanf because it blocks for ever if nothing is read
	  if (input[0] != 0 && strchr(input, 's') != NULL){  //check if data it transfered completely
		  if(sscanf(input, "T%d %d", &producer, &delaytime) == 2){
			  if(producer > 3){
				  printf("----------------> No %d. Producer", producer);
			  }
			  else{
				  //put delay time and producer into Queue
				  delay.time = delaytime;
				  delay.producer = producer;
				  if(osMessageQueuePut(delayPipeHandle, &delay, 0, 0) != osOK) { printf("delay Pipline full!!!");}
				  printf("Producer %d delay set to %dms\n", producer, delaytime);
			  }

			  //clear input buffer
			  memset(&input, 0, 20);
			  input_pointer = input;
		  }
		  else{
			  printf("----------------> data invalid \"%s\"\n", input);
		  }
	  }
	  /*
	  //for debugging
	  printf("==================> stack high water mark: %ld\n", uxTaskGetStackHighWaterMark(MonitorHandle));
	  printf("==================> stack high water mark: %ld\n", uxTaskGetStackHighWaterMark(Producer1Handle));
	  printf("==================> stack high water mark: %ld\n", uxTaskGetStackHighWaterMark(Producer2Handle));
	  printf("==================> stack high water mark: %ld\n", uxTaskGetStackHighWaterMark(Producer3Handle));
	  */
	  osDelay(100);
  }
  /* USER CODE END MonitorTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
