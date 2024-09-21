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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "PWM.h"
#include <std_msgs/msg/int32.h>
#include <stdlib.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint16_t FB_Read, LR_Read, BR_Read;
int Speed_L, Speed_R, Brush;
int MaxSpeed = 1000;
uint16_t freq;

PWM StepperL;
PWM StepperR;
PWM BrushMTR;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void StpperMotorControlled();
void BrusheMotorControlled();
int mapValue(uint16_t inputValue, int16_t Min, int16_t Max);
void subscription_callback(const void * msgin);
void subscription_callback_R(const void * msgin);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  PWM_init(&StepperL, &htim2, TIM_CHANNEL_1);
  PWM_init(&StepperR, &htim2, TIM_CHANNEL_2);
  PWM_init(&BrushMTR, &htim3, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


int mapValue(uint16_t inputValue, int16_t Min, int16_t Max)
{
    // Map from 1000-2010 to Min-Max
    return (int16_t)(((int32_t)(inputValue - 1000) * (Max - Min)) / 1010 + Min);
}

void StpperMotorControlled()
{
	if (Speed_L < -170){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		PWM_write_duty(&StepperL, fabs(Speed_L), 50);
	}
	else if (Speed_L > 170){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		PWM_write_duty(&StepperL, fabs(Speed_L), 50);
	}
	else{
		PWM_write_duty(&StepperL, 0, 50);
	}

	if (Speed_R < -170){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
		PWM_write_duty(&StepperR, fabs(Speed_R), 50);
	}
	else if (Speed_R > 170){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, SET);
		PWM_write_duty(&StepperR, fabs(Speed_R), 50);
	}
	else{
		PWM_write_duty(&StepperR, 0, 50);
	}
//	if (Speed < -20)
//	{
//		freq = 1;
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
//		if (fabs(Speed) > 170)
//		{
//			PWM_write_duty(&StepperL, fabs(Speed), 50);
//			PWM_write_duty(&StepperR, fabs(Speed), 50);
//		}
//		else
//		{
//			PWM_write_duty(&StepperL, 0, 50);
//			PWM_write_duty(&StepperR, 0, 50);
//		}
//	}
//	else if (Speed > 20)
//	{
//		freq = 2;
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, SET);
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
//		if (fabs(Speed) > 170)
//		{
//			PWM_write_duty(&StepperL, fabs(Speed), 50);
//			PWM_write_duty(&StepperR, fabs(Speed), 50);
//		}
//		else
//		{
//			PWM_write_duty(&StepperL, 0, 50);
//			PWM_write_duty(&StepperR, 0, 50);
//		}
//	}
//	else
//	{
//		freq = 0;
//		if (Turn <= 0)
//		{
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
//			if (fabs(Turn) > 170)
//			{
//				PWM_write_duty(&StepperL, fabs(Turn), 50);
//				PWM_write_duty(&StepperR, fabs(Turn), 50);
//			}
//			else
//			{
//				PWM_write_duty(&StepperL, 0, 50);
//				PWM_write_duty(&StepperR, 0, 50);
//			}
//		}
//		else if (Turn > 0)
//		{
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, SET);
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
//			if (fabs(Turn) > 170)
//			{
//				PWM_write_duty(&StepperL, fabs(Turn), 50);
//				PWM_write_duty(&StepperR, fabs(Turn), 50);
//			}
//			else
//			{
//				PWM_write_duty(&StepperL, 0, 50);
//				PWM_write_duty(&StepperR, 0, 50);
//			}
//		}
//	}
}
void BrusheMotorControlled()
{
	if (Brush)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, RESET);
		PWM_write_duty(&BrushMTR, 2000, 100);
	}
	else
	{
		PWM_write_duty(&BrushMTR, 2000, 0);
	}
}

void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

//    Speed = msg->data;
    Speed_L = msg->data;
}

void subscription_callback_R(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    Speed_R = msg->data;  // Update Turn (or handle the message however you need)
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
//
//  // micro-ROS configuration
//
//  rmw_uros_set_custom_transport(
//    true,
//    (void *) &huart2,
//    cubemx_transport_open,
//    cubemx_transport_close,
//    cubemx_transport_write,
//    cubemx_transport_read);
//
//  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
//  freeRTOS_allocator.allocate = microros_allocate;
//  freeRTOS_allocator.deallocate = microros_deallocate;
//  freeRTOS_allocator.reallocate = microros_reallocate;
//  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;
//
//  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
//      printf("Error on default allocators (line %d)\n", __LINE__);
//  }
//
//  // micro-ROS app
//
//  rcl_publisher_t publisher;
//  std_msgs__msg__Int32 msg;
//  rclc_support_t support;
//  rcl_allocator_t allocator;
//  rcl_node_t node;
//  rclc_executor_t executor;
//
//  allocator = rcl_get_default_allocator();
//
//  //create init_options
//  rclc_support_init(&support, 0, NULL, &allocator);
//
//  // create node
//  rclc_node_init_default(&node, "cubemx_node", "", &support);
//
//  // create publisher
//  rclc_publisher_init_default(
//    &publisher,
//    &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//    "cubemx_publisher");
//
//  rclc_executor_init(&executor, &support.context, 1, &allocator);
//
//  msg.data = 0;
//
//  for(;;)
//  {
//    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
//    if (ret != RCL_RET_OK)
//    {
//      printf("Error publishing (line %d)\n", __LINE__);
//    }
//
//    msg.data++;
//    osDelay(10);
//    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//  }
//
	 setenv("ROS_DOMAIN_ID", "1", 1);


	 rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart2,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }



	  // micro-ROS app
	  rcl_subscription_t subscriber;
	  rcl_subscription_t subscriber_R;
	  std_msgs__msg__Int32 msg;
	  std_msgs__msg__Int32 msg_R;

	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;
	  rclc_executor_t executor;

	  allocator = rcl_get_default_allocator();

	  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	  rcl_init_options_init(&init_options, allocator);
	  rcl_init_options_set_domain_id(&init_options, 1);

	  // Initialize rclc support object with custom options
//	  rclc_support_t support;
	  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
	  // Create init_options
//	  rclc_support_init(&support, 0, NULL, &allocator);
//	  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	  // Create node
	  rclc_node_init_default(&node, "cubemx_node", "", &support);

	  // Create subscriber
	  rclc_subscription_init_default(
	    &subscriber,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	    "cubemx_publisher");

	    rclc_subscription_init_default(
	        &subscriber_R,
	        &node,
	        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	        "cubemx_publisher_R");


	  // Initialize the executor
	  rclc_executor_init(&executor, &support.context, 2, &allocator);

	  // Add the subscriber callback to the executor
	  rclc_executor_add_subscription(&executor, &subscriber, &msg, subscription_callback, ON_NEW_DATA);

	  rclc_executor_add_subscription(&executor, &subscriber_R, &msg_R, subscription_callback_R, ON_NEW_DATA);




	  for(;;)
	  {
	    // Spin the executor to handle incoming messages
		StpperMotorControlled();
		BrusheMotorControlled();
	    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	    osDelay(10);
	  }

  /* USER CODE END 5 */
}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
