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
  * If no LICENSE file comes with this software, it is provided A S-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
//ham format du lieu

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define READ_SENSOR_DONE 1

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float temp_global = 0;// tu khoa volatile khi he thong nhieu loi, da loi,, o day la bien arm co mot loi nen co the dung 1 loi
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_helloTask */
osThreadId_t task_helloTaskHandle;
const osThreadAttr_t task_helloTask_attributes = {
  .name = "task_helloTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for putQueue */
osThreadId_t putQueueHandle;
const osThreadAttr_t putQueue_attributes = {
  .name = "putQueue",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for temp_queue */
osMessageQueueId_t temp_queueHandle;
const osMessageQueueAttr_t temp_queue_attributes = {
  .name = "temp_queue"
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};
/* Definitions for UART_lock */
osMutexId_t UART_lockHandle;
const osMutexAttr_t UART_lock_attributes = {
  .name = "UART_lock"
};
/* Definitions for myCountingSem01 */
osSemaphoreId_t myCountingSem01Handle;
const osSemaphoreAttr_t myCountingSem01_attributes = {
  .name = "myCountingSem01"
};
/* Definitions for read_sensor_status */
osEventFlagsId_t read_sensor_statusHandle;
const osEventFlagsAttr_t read_sensor_status_attributes = {
  .name = "read_sensor_status"
};
/* USER CODE BEGIN PV */
float temp = 0;

void ADC_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* MODER = (uint32_t*)0x40020000;
	*MODER |= (0b11<<0);

	__HAL_RCC_ADC1_CLK_ENABLE();
	uint32_t* SMPR1 = (uint32_t*)0x4001200c;
	*SMPR1 |= (0b111<<18);
	uint32_t* SMPR2 = (uint32_t*)0x40012010;
	*SMPR2 |= (0b111<<0);

	uint32_t* JSQR = (uint32_t*)0x40012038;
	*JSQR |= (16<<15); //set channel 16 (temp sensor)

//	uint32_t* CR1 = (uint32_t*)0x40012004;
	uint32_t* CR2 = (uint32_t*)0x40012008;
	*CR2 |= (0b01 << 20) |(1<<0);

	uint32_t* CCR = (uint32_t*)0x40012304;
	*CCR |= (1<<23);
}
uint16_t Read_ADC ()
{
	uint32_t* CR2 = (uint32_t*)0x40012008;
	*CR2 |= (1<<22);
	uint32_t* SR = (uint32_t*)0x40012000;
	while (((*SR >> 2)&1) == 0){osDelay(1);};
	*SR &=~(1<<2);

	uint32_t* JDR1 = (uint32_t*)0x4001203c;
	return *JDR1;
}

void UART_Init()
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();

	uint32_t* GPIOA_MODER = (uint32_t*)0x40020000;
	*GPIOA_MODER &= ~(0b1111 << 4); // set PIN2,3
	*GPIOA_MODER |= (0b10 << 4) | (0b10 << 6);// Pin2,3 Analog
	uint32_t* GPIOA_AFRL = (uint32_t*)0x40020020;
	*GPIOA_AFRL |= (7 << 8) | (7 << 12);

	// set baud rate 9600
	uint32_t* UART2_BRR = (uint32_t*)0x40004408;
	*UART2_BRR = (104 << 4) | 3;

	//set 13 enable UART, set 2 r/w enable TX, set 3 r/w enable RX
	// set 5 enable Interrupt of UART
	//size 8 byte and check chan le
	uint32_t* UART2_CR1 = (uint32_t*)0x4000440c;
	*UART2_CR1 |= (0b1 << 13) | (0b1 << 2) | (0b1 << 3);// | (0b1 << 5);

	//set enable DMA of UART
//	uint32_t* UART2_CR3 = (uint32_t*)0x40004414;
//	*UART2_CR3 |= (0b1 << 6);
}




void UART_Send_1Byte(char data)
{
	uint32_t* DR = (uint32_t*)0x40004404;
	uint32_t* SR = (uint32_t*)0x40004400;
	while(((*SR >> 7)&1) !=1){osDelay(1);};
	*DR = data;
	while(((*SR >> 6)&1) !=1);
}


void UART_Send_Str(char* data)
{
	osMutexAcquire(UART_lockHandle, HAL_MAX_DELAY);
	int str_len = strlen(data);
	for(int i =0; i < str_len; i++)
	{
		UART_Send_1Byte( data [i]);
	}
	osMutexRelease(UART_lockHandle);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void send_data_task(void *argument);
void read_temp_ss_task(void *argument);
void print_hello_func(void *argument);
void StartTask04(void *argument);
void Callback_timer(void *argument);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of UART_lock */
  UART_lockHandle = osMutexNew(&UART_lock_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myCountingSem01 */
  myCountingSem01Handle = osSemaphoreNew(4, 4, &myCountingSem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer01 */
  myTimer01Handle = osTimerNew(Callback_timer, osTimerPeriodic, NULL, &myTimer01_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of temp_queue */
  temp_queueHandle = osMessageQueueNew (16, sizeof(float), &temp_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(send_data_task, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(read_temp_ss_task, NULL, &myTask02_attributes);

  /* creation of task_helloTask */
  task_helloTaskHandle = osThreadNew(print_hello_func, NULL, &task_helloTask_attributes);

  /* creation of putQueue */
  putQueueHandle = osThreadNew(StartTask04, NULL, &putQueue_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of read_sensor_status */
  read_sensor_statusHandle = osEventFlagsNew(&read_sensor_status_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  osTimerStart(myTimer01Handle, 1000);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/* USER CODE BEGIN Header_send_data_task */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_send_data_task */
void send_data_task(void *argument)
{
  /* USER CODE BEGIN 5 */

	UART_Init();
	float temp = 0;
  /* Infinite loop */
  for(;;)
  {
	  temp = temp_global;
	  osMessageQueueGet(temp_queueHandle, &temp,  osPriorityNormal , 10000);
	  char msg[32] = {0};
	  char frac = (int)((temp - (int)temp)*100);
	  sprintf(msg, "temp: %d.%d *C\r\n", (int)temp, frac);    //sprintf de format du lieu
	  UART_Send_Str(msg);
      osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_read_temp_ss_task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_read_temp_ss_task */
void read_temp_ss_task(void *argument)
{
  /* USER CODE BEGIN read_temp_ss_task */
	ADC_Init();
	uint16_t data_raw =0; Read_ADC();
	float vin =0;

  /* Infinite loop */
  for(;;)
  {
//	  {int temp;}
	data_raw = Read_ADC();
	vin = (data_raw*3.0)/4095.0;
	temp = ((vin - 0.76) / 0.0025) + 25;
	osEventFlagsSet(read_sensor_statusHandle, READ_SENSOR_DONE);
	osMessageQueuePut(temp_queueHandle, &temp,  osPriorityNormal , 10000);
//	temp_global = temp; //= la gan ghi du lieu vung nho
    osDelay(1);
  }
  /* USER CODE END read_temp_ss_task */
}

/* USER CODE BEGIN Header_print_hello_func */
/**
* @brief Function implementing the task_helloTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_print_hello_func */
void print_hello_func(void *argument)
{
  /* USER CODE BEGIN print_hello_func */
  /* Infinite loop */
  for(;;)
  {
	  UART_Send_Str("hello \r\n");
	  osDelay(1000);
  }
  /* USER CODE END print_hello_func */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the putQueue thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */

  for(;;)
  {
	  uint32_t flag_result = osEventFlagsWait(read_sensor_statusHandle, READ_SENSOR_DONE, osFlagsWaitAny, 10000);
	  if (flag_result == READ_SENSOR_DONE)
		  osMessageQueuePut(temp_queueHandle, &temp,  osPriorityNormal , 10000);
	  osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* Callback_timer function */
void Callback_timer(void *argument)
{
  /* USER CODE BEGIN Callback_timer */
	static int cnt = 0;
	char msg[32] = {0};
	sprintf(msg, "[%d] %s: %d\r\n", __LINE__, __FUNCTION__, cnt);
	UART_Send_Str(msg);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	if (cnt++ >10)
		osTimerStop(myTimer01Handle);
  /* USER CODE END Callback_timer */
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
