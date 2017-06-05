#include "sdcard.h"

void sdcard_init( SD_HandleTypeDef* hsd )
{
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	hsd->Instance = SDIO;
	hsd->Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd->Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd->Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd->Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd->Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd->Init.ClockDiv = 0;
	MX_FATFS_Init( );
	BSP_SD_Init( );
}

uint8_t sdcard_mount( FATFS* SDFatFs, const char* SDPath  )
{
	if( f_mount( SDFatFs, SDPath, 1 ) != FR_OK )
		return 0;
	else
		return 1;
}

uint8_t sdcard_open_file( FIL* myFile, const char* fileName )
{
	if( f_open( myFile, fileName, FA_OPEN_ALWAYS | FA_WRITE| FA_READ ) != FR_OK )
			return 0;
		else
			return 1;
}

uint8_t sdcard_save2file( FIL* myFile, int16_t x, int16_t y, int16_t z )
{
	if( f_lseek( myFile, f_size( myFile ) ) != FR_OK )
				return 0;
	int res = f_printf( myFile, "%d,%d,%d\n", x, y, z );
	if ( res != FR_OK )
		return 0;
	else
		return 1;


}

void sdcard_close_file( FIL* myFile )
{
	f_close( myFile );
}
