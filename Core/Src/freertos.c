/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId NMotor_TaskHandle;
osThreadId NExo_TaskHandle;
osThreadId NVofa_TaskHandle;
osThreadId NLed_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Motor_Task(void const * argument);
void Exo_Task(void const * argument);
void Vofa_Task(void const * argument);
void Led_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* definition and creation of NMotor_Task */
  osThreadDef(NMotor_Task, Motor_Task, osPriorityRealtime, 0, 256);
  NMotor_TaskHandle = osThreadCreate(osThread(NMotor_Task), NULL);

  /* definition and creation of NExo_Task */
  osThreadDef(NExo_Task, Exo_Task, osPriorityRealtime, 0, 256);
  NExo_TaskHandle = osThreadCreate(osThread(NExo_Task), NULL);

  /* definition and creation of NVofa_Task */
  osThreadDef(NVofa_Task, Vofa_Task, osPriorityRealtime, 0, 256);
  NVofa_TaskHandle = osThreadCreate(osThread(NVofa_Task), NULL);

  /* definition and creation of NLed_Task */
  osThreadDef(NLed_Task, Led_Task, osPriorityHigh, 0, 256);
  NLed_TaskHandle = osThreadCreate(osThread(NLed_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Motor_Task */
/**
* @brief Function implementing the NMotor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task */
__weak void Motor_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Motor_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_Task */
}

/* USER CODE BEGIN Header_Exo_Task */
/**
* @brief Function implementing the NExo_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Exo_Task */
__weak void Exo_Task(void const * argument)
{
  /* USER CODE BEGIN Exo_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Exo_Task */
}

/* USER CODE BEGIN Header_Vofa_Task */
/**
* @brief Function implementing the NVofa_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Vofa_Task */
__weak void Vofa_Task(void const * argument)
{
  /* USER CODE BEGIN Vofa_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Vofa_Task */
}

/* USER CODE BEGIN Header_Led_Task */
/**
* @brief Function implementing the NLed_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led_Task */
__weak void Led_Task(void const * argument)
{
  /* USER CODE BEGIN Led_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Led_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
