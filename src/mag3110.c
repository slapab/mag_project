/*
 * mag3110.c
 *
 *  Created on: 28 kwi 2017
 *      Author: patlas
 */

#include "mag3110.h"
#include <stdbool.h>
#include "stm32f429xx.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
//#include "stm32f4xx_hal_i2c.h"

#define MAG_I2C_ADDR (0x0E<<1)

volatile SemaphoreHandle_t i2c_data_ready;


static I2C_HandleTypeDef i2c_handler;
static I2C_InitTypeDef init_type;
static QueueHandle_t i2c_read_queue;
static TickType_t task_delay;
static TaskHandle_t mag_task_handler;

static uint8_t mag3110_read_reg(uint8_t addr);
static HAL_StatusTypeDef mag3110_write_reg(uint8_t addr, uint8_t data);
static void mag3110_task(void* params);
static bool mag3110_axix(mag3110_data_t *ptr);

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	if (hi2c->Instance == I2C3)
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* Configure I2C Tx as alternate function  */
		GPIO_InitStruct.Pin       = GPIO_PIN_8;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* Configure I2C Rx as alternate function  */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


		/* Configure the Discovery I2Cx peripheral -------------------------------*/
		/* Enable I2C3 clock */
		__HAL_RCC_I2C3_CLK_ENABLE();

		/* Force the I2C Peripheral Clock Reset */
		__HAL_RCC_I2C3_FORCE_RESET();

		/* Release the I2C Peripheral Clock Reset */
		__HAL_RCC_I2C3_RELEASE_RESET();
	}
}

bool init_i2c(void)
{

	init_type.ClockSpeed = 100000; //100kHz
	init_type.DutyCycle = I2C_DUTYCYCLE_2;
	init_type.OwnAddress1 = 0x00;
	init_type.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	init_type.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	init_type.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	init_type.NoStretchMode = I2C_NOSTRETCH_DISABLE;


	i2c_handler.Init = init_type;
	i2c_handler.Instance = I2C3;

	if(HAL_OK == HAL_I2C_Init(&i2c_handler))
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool init_mag3110(mag_sampling_rate_t sample, TickType_t mag_task_delay)
{
	task_delay = mag_task_delay;
	i2c_read_queue = xQueueCreate(I2C_QUEUE_SIZE, sizeof(mag3110_data_t));
	i2c_data_ready = xSemaphoreCreateBinary();
	xSemaphoreGive(i2c_data_ready);
	xSemaphoreTake(i2c_data_ready, portMAX_DELAY);

	// mag3110 int1 ping configuration
	GPIO_InitTypeDef  GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin       = GPIO_PIN_7;
	GPIO_InitStruct.Mode      = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull      = GPIO_PULLDOWN; //GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	NVIC_SetPriority(EXTI9_5_IRQn, 5);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


	// read and compare device factory ID
	if(0xC4 != mag3110_read_reg(WHO_AM_I)) return false;

	// set sensor sampling rate
	switch(sample)
	{
		default:
		case SAMPLING_80HZ:
			if(HAL_OK != mag3110_write_reg(CTRL_REG2, 0x80)) return false;
			if(HAL_OK != mag3110_write_reg(CTRL_REG1, 0x01)) return false;
			break;

		case SAMPLING_10HZ:
			if(HAL_OK != mag3110_write_reg(CTRL_REG2, 0x80)) return false;
			if(HAL_OK != mag3110_write_reg(CTRL_REG1, 0xC9)) return false;
			break;
	}

	if(pdPASS != xTaskCreate(mag3110_task, "MAG3110", 500, NULL, MAG_TASK_PRIORITY, &mag_task_handler))
		return false;

	//task suspend
	vTaskSuspend(mag_task_handler);
	return true;
}

static uint8_t mag3110_read_reg(uint8_t addr)
{
	uint8_t ret = 0;
	HAL_StatusTypeDef stat;

	stat = HAL_I2C_Master_Transmit(&i2c_handler, (0x00FF & MAG_I2C_ADDR), &addr, 1, 10);
	if(HAL_OK != stat)
	{
		return 0;
	}

	HAL_I2C_Master_Receive(&i2c_handler, (0x00FF & MAG_I2C_ADDR), &ret, 1, 10);
	if(HAL_OK != stat)
	{
		return 0;
	}
	return ret;
}

static HAL_StatusTypeDef mag3110_write_reg(uint8_t addr, uint8_t data)
{
	uint8_t dat[2] = { addr, data };

	return HAL_I2C_Master_Transmit(&i2c_handler, (0x00FF & MAG_I2C_ADDR), dat, 2, 10000);
}

static bool mag3110_axix(mag3110_data_t *ptr)
{
	ptr->temp = mag3110_read_reg(DIE_TEMP) + TEMP_OFFSET;

	ptr->x = ((int16_t)mag3110_read_reg(OUT_X_MSB))<<8;
	ptr->x |= mag3110_read_reg(OUT_X_LSB);

	ptr->y = ((int16_t)mag3110_read_reg(OUT_Y_MSB))<<8;
	ptr->y |= mag3110_read_reg(OUT_Y_LSB);

	ptr->z = ((int16_t)mag3110_read_reg(OUT_Z_MSB))<<8;
	ptr->z |= mag3110_read_reg(OUT_Z_LSB);

	return true;
}

static void mag3110_task(void* params)
{
	mag3110_data_t data_struct = {0};

	while(1)
	{
		// wait until INT1 pin of MAG3110 indicate that data are ready (see stm32f4xx_it.c)
		xSemaphoreTake(i2c_data_ready, portMAX_DELAY);
		if(mag3110_axix(&data_struct))
		{
			if(0 == uxQueueSpacesAvailable(i2c_read_queue))
			{
				// remove the oldest data in queue
				xQueueReceive(i2c_read_queue, NULL, portMAX_DELAY);
			}
			// insert the newest data queue tail
			xQueueSendToBack(i2c_read_queue, &data_struct, portMAX_DELAY);
		}
		//task delay
		vTaskDelay(task_delay);
	}


}

// if ret false that no data available
bool mag3110_get(mag3110_data_t *ptr)
{
	if(pdTRUE != xQueueReceive(i2c_read_queue, ptr, 0))
		return false;

	return true;
}

void mag3110_start()
{
	vTaskResume(mag_task_handler);
	// read register to invoke INT1 pin start working
	mag3110_read_reg(OUT_X_MSB);
}

void mag3110_stop()
{
	vTaskSuspend(mag_task_handler);
}
