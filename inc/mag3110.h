/*
 * mag3110.h
 *
 *  Created on: 28 kwi 2017
 *      Author: patla
 */

#ifndef MAG3110_H_
#define MAG3110_H_

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#define DR_STATUS	0x00
#define OUT_X_MSB	0x01
#define OUT_X_LSB	0x02
#define OUT_Y_MSB	0x03
#define OUT_Y_LSB	0x04
#define OUT_Z_MSB	0x05
#define OUT_Z_LSB	0x06
#define WHO_AM_I	0x07
#define SYSMOD		0x08
#define OFF_X_MSB	0x09
#define OFF_X_LSB	0x0A
#define OFF_Y_MSB	0x0B
#define OFF_Y_LSB	0x0C
#define OFF_Z_MSB	0x0D
#define OFF_Z_LSB	0x0E
#define DIE_TEMP	0x0F
#define CTRL_REG1	0x10
#define CTRL_REG2	0x11

typedef enum {
	SAMPLING_80HZ = 0,
	SAMPLING_10HZ
} mag_sampling_rate_t;


#define I2C_QUEUE_SIZE	10
#define MAG_TASK_PRIORITY 5
#define TEMP_OFFSET	24

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
	int8_t temp;
} mag3110_data_t;


bool init_i2c(void);
bool init_mag3110(mag_sampling_rate_t sample, TickType_t mag_task_delay);
bool mag3110_get(mag3110_data_t *ptr);
void mag3110_start();
void mag3110_stop();


#endif /* MAG3110_H_ */
