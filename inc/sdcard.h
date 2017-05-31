#ifndef SDCARD_H_
#define SDCARD_H_
#include "stm32f4xx_hal.h"
#include "fatfs.h"

void sdcard_init( SD_HandleTypeDef* hsd );

uint8_t sdcard_mount( FATFS* SDFatFs, const char* SDPath );

uint8_t sdcard_open_file( FIL* myFile, const char* fileName );

uint8_t sdcard_save2file( FIL* myFile, int16_t x, int16_t y, int16_t z );

void sdcard_close_file( FIL* myFile );

#endif /* SDCARD_H_ */
