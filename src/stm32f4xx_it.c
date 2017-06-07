/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f4xx_it.h"

#include "signal.h"

#define MAG_INT1_PIN	1<<7

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

//extern volatile SemaphoreHandle_t i2c_data_ready;
extern TaskHandle_t mag_task_handler;
extern volatile sig_atomic_t waitingIts;
void EXTI9_5_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(MAG_INT1_PIN))
	{
	  __HAL_GPIO_EXTI_CLEAR_IT(MAG_INT1_PIN);
//	  if (errQUEUE_FULL == xSemaphoreGiveFromISR(i2c_data_ready, NULL)) {
//	      while(1) {;}
//	  }
         /* Notify the task that there are data to read */
//        configASSERT(NULL != mag_task_handler);
//        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//        vTaskNotifyGiveFromISR(mag_task_handler, &xHigherPriorityTaskWoken);
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	  ++waitingIts;
	}

	// clear all pending interrupt from 5-9
//	__HAL_GPIO_EXTI_CLEAR_IT(0x3E0);
}


