/*
 * LCD.c
 *
 *  Created on: 19 May 2017
 *      Author: Marcin
 */
#include <stdlib.h>
#include "LCD.h"
#include "SPI.h"
#include "ili9341.h"
#include "stm32f4xx_hal.h"
#include "fonts.h"


#define WRX_SET()          HAL_GPIO_WritePin(WRX_GPIO_PORT, WRX_PIN, 1);
#define WRX_RESET()        HAL_GPIO_WritePin(WRX_GPIO_PORT, WRX_PIN, 0);
#define CS_SET()           HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, 1);
#define CS_RESET()         HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, 0);
#define RST_SET()          HAL_GPIO_WritePin(RST_GPIO_PORT, RST_PIN, 1);
#define RST_RESET()        HAL_GPIO_WritePin(RST_GPIO_PORT, RST_PIN, 0);

static char dataBuff[50];

typedef enum {
	LCD_Landscape,
	LCD_Portrait
}LCD_Orientation;

typedef struct {
	uint16_t width;
	uint16_t height;
	LCD_Orientation orientation;
}LCD_Options_t;

typedef struct {
	uint16_t width;
	uint16_t height;
}String_Size_t;

GPIO_InitTypeDef GPIO_InitStruct;
SPI_HandleTypeDef hspi;
DMA_HandleTypeDef hdma_spi_rx;
DMA_HandleTypeDef hdma_spi_tx;

uint16_t LCD_x;
uint16_t LCD_y;
LCD_Options_t LCD_Options;
String_Size_t String_Size;
uint8_t Data_From_Puts = 0;
int Flag = 0;


void LCD_Init(void) {
	GPIO_Init();
	CS_SET();
	DMA_Init();
	SPI_Init();
	LCD_InitLCD();
	LCD_x = LCD_y = 0;
	LCD_Options.width = LCD_WIDTH;
	LCD_Options.height = LCD_HEIGHT;
	LCD_Options.orientation = LCD_Portrait;
	LCD_Fill(LCD_COLOR_WHITE);
	LCD_Circle(LCD_COLOR_GREEN);
	LCD_SetScene1(120, 500, 60, 50, 60 ,300);
}

int LCD_InitLCD(void) {
	//LCD_SendCommand(0xCA);
	  //LCD_SendData(0xC3);
	  //LCD_SendData(0x08);
	  //LCD_SendData(0x50);
	  LCD_SendCommand(LCD_POWERB);
	  LCD_SendData(0x00);
	  LCD_SendData(0xC1);
	  LCD_SendData(0x30);
	  LCD_SendCommand(LCD_POWER_SEQ);
	  LCD_SendData(0x64);
	  LCD_SendData(0x03);
	  LCD_SendData(0x12);
	  LCD_SendData(0x81);
	  LCD_SendCommand(LCD_DTCA);
	  LCD_SendData(0x85);
	  LCD_SendData(0x00);
	  LCD_SendData(0x78);
	  LCD_SendCommand(LCD_POWERA);
	  LCD_SendData(0x39);
	  LCD_SendData(0x2C);
	  LCD_SendData(0x00);
	  LCD_SendData(0x34);
	  LCD_SendData(0x02);
	  LCD_SendCommand(LCD_PRC);
	  LCD_SendData(0x20);
	  LCD_SendCommand(LCD_DTCB);
	  LCD_SendData(0x00);
	  LCD_SendData(0x00);
	  LCD_SendCommand(LCD_FRMCTR1);
	  LCD_SendData(0x00);
	  LCD_SendData(0x1B);
	  LCD_SendCommand(LCD_DFC);
	  LCD_SendData(0x0A);
	  LCD_SendData(0xA2);
	  LCD_SendCommand(LCD_POWER1);
	  LCD_SendData(0x10);
	  LCD_SendCommand(LCD_POWER2);
	  LCD_SendData(0x10);
	  LCD_SendCommand(LCD_VCOM1);
	  LCD_SendData(0x45);
	  LCD_SendData(0x15);
	  LCD_SendCommand(LCD_VCOM2);
	  LCD_SendData(0x90);
	  LCD_SendCommand(LCD_MAC);
	  LCD_SendData(0xC8);
	  LCD_SendCommand(LCD_3GAMMA_EN);
	  LCD_SendData(0x00);
	  //
	  //LCD_SendCommand(LCD_RGB_INTERFACE);
	  //LCD_SendData(0xC2);
	  LCD_SendCommand(LCD_DFC);
	  LCD_SendData(0x0A);
	  LCD_SendData(0xA7);
	  LCD_SendData(0x27);
	  LCD_SendData(0x04);


	  LCD_SendCommand(LCD_PIXEL_FORMAT);
	  LCD_SendData(0x55);

	  /* Colomn address set */
	  LCD_SendCommand(LCD_COLUMN_ADDR);
	  LCD_SendData(0x00);
	  LCD_SendData(0x00);
	  LCD_SendData(0x00);
	  LCD_SendData(0xEF);
	  /* Page address set */
	  LCD_SendCommand(LCD_PAGE_ADDR);
	  LCD_SendData(0x00);
	  LCD_SendData(0x00);
	  LCD_SendData(0x01);
	  LCD_SendData(0x3F);
	  LCD_SendCommand(LCD_INTERFACE);
	  LCD_SendData(0x01);
	  LCD_SendData(0x00);
	  //  change from 0x02 (RGB) to 0x00 (internal clocking)
	  LCD_SendData(0x00);

	  LCD_SendCommand(LCD_GRAM);
	  HAL_Delay(200);

	  LCD_SendCommand(LCD_GAMMA);
	  LCD_SendData(0x01);

	  LCD_SendCommand(LCD_PGAMMA);
	  LCD_SendData(0x0F);
	  LCD_SendData(0x29);
	  LCD_SendData(0x24);
	  LCD_SendData(0x0C);
	  LCD_SendData(0x0E);
	  LCD_SendData(0x09);
	  LCD_SendData(0x4E);
	  LCD_SendData(0x78);
	  LCD_SendData(0x3C);
	  LCD_SendData(0x09);
	  LCD_SendData(0x13);
	  LCD_SendData(0x05);
	  LCD_SendData(0x17);
	  LCD_SendData(0x11);
	  LCD_SendData(0x00);
	  LCD_SendCommand(LCD_NGAMMA);
	  LCD_SendData(0x00);
	  LCD_SendData(0x16);
	  LCD_SendData(0x1B);
	  LCD_SendData(0x04);
	  LCD_SendData(0x11);
	  LCD_SendData(0x07);
	  LCD_SendData(0x31);
	  LCD_SendData(0x33);
	  LCD_SendData(0x42);
	  LCD_SendData(0x05);
	  LCD_SendData(0x0C);
	  LCD_SendData(0x0A);
	  LCD_SendData(0x28);
	  LCD_SendData(0x2F);
	  LCD_SendData(0x0F);

	  LCD_SendCommand(LCD_SLEEP_OUT);
	  HAL_Delay(200);
	  LCD_SendCommand(LCD_DISPLAY_ON);
	  /* GRAM start writing */
	  LCD_SendCommand(LCD_GRAM);
	  return 1;
}

void LCD_DisplayOn(void){
	LCD_SendCommand(LCD_DISPLAY_ON);
}

void LCD_DisplayOff(void) {
	LCD_SendCommand(LCD_DISPLAY_OFF);
}

void LCD_SendCommand(uint8_t data) {
	WRX_RESET();
	CS_RESET();
	HAL_SPI_Transmit(&hspi, &data, 1, 500);
	//  HAL_SPI_Transmit_IT(&hspi, &data, 1);
	CS_SET();
}

void LCD_SendData(uint8_t data) {
	WRX_SET();
	CS_RESET();
	HAL_SPI_Transmit(&hspi, &data, 1, 500);
	//  HAL_SPI_Transmit_IT(&hspi, &data, 1);
	CS_SET();
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint32_t color) {
	LCD_SetCursor(x,y,x,y);
	LCD_SendCommand(LCD_GRAM);
	LCD_SendData(color >> 8);
	LCD_SendData(color & 0xFF);
}

void LCD_SetCursor(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	LCD_SendCommand(LCD_COLUMN_ADDR);
	LCD_SendData(x1 >> 8);
	LCD_SendData(x1 & 0xFF);
	LCD_SendData(x2 >> 8);
	LCD_SendData(x2 & 0xFF);

	LCD_SendCommand(LCD_PAGE_ADDR);
	LCD_SendData(y1 >> 8);
	LCD_SendData(y1 & 0xFF);
	LCD_SendData(y2 >> 8);
	LCD_SendData(y2 & 0xFF);
}

void LCD_Fill(uint32_t color) {
	uint32_t n;
	uint8_t hi,lo;
	hi = color >> 8;
	lo = color & 0xFF;
	LCD_SetCursor(0,0,LCD_Options.width - 1,LCD_Options.height -1);
	LCD_SendCommand(LCD_GRAM);
	for(n=0;n<LCD_PIXEL;n++) {
		LCD_SendData(hi);
		LCD_SendData(lo);
	}
}

void LCD_Circle(uint32_t color) {
	if (color == LCD_COLOR_RED) {
		Flag = 1;
		LCD_DrawFilledCircle(120,100,80,LCD_COLOR_RED);
	}
	else {
		Flag = 0;
		LCD_DrawFilledCircle(120,100,80,LCD_COLOR_GREEN2);
	}
}

void LCD_Rotate(LCD_Orientation_t orientation) {
	LCD_SendCommand(LCD_MAC);
	if (orientation == LCD_Orientation_Portrait_1) {
		LCD_SendData(0x58);
	}
	else if (orientation == LCD_Orientation_Portrait_2) {
		LCD_SendData(0x88);
	}
	else if (orientation == LCD_Orientation_Landscape_1) {
		LCD_SendData(0x28);
	}
	else LCD_SendData(0xE8);
	if (orientation == LCD_Orientation_Portrait_1 || orientation == LCD_Orientation_Portrait_2) {
		LCD_Options.width = LCD_WIDTH;
		LCD_Options.height = LCD_HEIGHT;
		LCD_Options.orientation = LCD_Portrait;
	}
	else {
		LCD_Options.width = LCD_HEIGHT;
		LCD_Options.height = LCD_WIDTH;
		LCD_Options.orientation = LCD_Landscape;
	}
}

void LCD_SetScene1(uint32_t Val1, uint32_t Val2, uint32_t x, uint32_t y, uint32_t z, uint32_t T) {
	LCD_SetVal1(Val1);
	LCD_SetVal2(Val2);
	LCD_SetX(x);
	LCD_SetY(y);
	LCD_SetZ(z);
	LCD_SetTemp(T);
}

void LCD_SetVal1(uint32_t num) {
//	char data[50];
	sprintf(dataBuff,"%ld A/m",num);
	LCD_GetStringSize(&dataBuff[0], &Font24, &String_Size.width, &String_Size.height);
	if (Flag == 0) {
		LCD_Puts((LCD_Options.width-String_Size.width)/2, 90, &dataBuff[0], &Font24, LCD_COLOR_RED, LCD_COLOR_GREEN2);
	}
	else if (Flag == 1) {
		LCD_Puts((LCD_Options.width-String_Size.width)/2, 90, &dataBuff[0], &Font24, LCD_COLOR_GREEN2, LCD_COLOR_RED);
	}
}

void LCD_SetVal2(uint32_t num) {
//	char data[50];
	sprintf(dataBuff,"max = %ld A/m",num);
	LCD_GetStringSize(&dataBuff[0], &Font20, &String_Size.width, &String_Size.height);
	LCD_Puts((LCD_Options.width-String_Size.width)/2, 200, &dataBuff[0], &Font20, LCD_COLOR_RED, LCD_COLOR_WHITE);
}

void LCD_SetX(uint32_t num) {
//	char data[50];
	sprintf(dataBuff,"x = %ld A/m",num);
	LCD_GetStringSize(&dataBuff[0], &Font16, &String_Size.width, &String_Size.height);
	LCD_Puts(70, 230, &dataBuff[0], &Font16, LCD_COLOR_BLACK, LCD_COLOR_WHITE);
}

void LCD_SetY(uint32_t num) {
//	char data[50];
	sprintf(dataBuff,"y = %ld A/m",num);
	LCD_GetStringSize(&dataBuff[0], &Font16, &String_Size.width, &String_Size.height);
	LCD_Puts(70, 246, &dataBuff[0], &Font16, LCD_COLOR_BLACK, LCD_COLOR_WHITE);
}

void LCD_SetZ(uint32_t num) {
//	char data[50];
	sprintf(dataBuff,"z = %ld A/m",num);
	LCD_GetStringSize(&dataBuff[0], &Font16, &String_Size.width, &String_Size.height);
	LCD_Puts(70, 262, &dataBuff[0], &Font16, LCD_COLOR_BLACK, LCD_COLOR_WHITE);
}

void LCD_SetTemp(uint32_t num) {
//	char data[50];
	sprintf(dataBuff,"T = %ld K",num);
	LCD_GetStringSize(&dataBuff[0], &Font16, &String_Size.width, &String_Size.height);
	LCD_Puts(70, 278, &dataBuff[0], &Font16, LCD_COLOR_BLACK, LCD_COLOR_WHITE);
}

void LCD_Puts(uint16_t x, uint16_t y, char *str, sFONT* font, uint32_t foreground, uint32_t background) {
	uint16_t startX = x;
	LCD_x = x;
	LCD_y = y;
	while (*str) {
		if (*str == '\n') {
			LCD_y += font->Height + 1;
			if (*(str+1) == '\r') {
				LCD_x = 0;
				str++;
			}
			else {
				LCD_x = startX;
			}
			str++;
			continue;
		}
		else if (*str == '\r') {
			str++;
			continue;
		}
		LCD_Putc(LCD_x, LCD_y, *str++, font, foreground, background);
		LCD_x+=font->Width;
	}
}

void LCD_GetStringSize(char *str, sFONT* font, uint16_t *width, uint16_t *height) {
	uint16_t w = 0;
	*height = font->Height;
	while (*str++) {
		w += font->Width;
	}
	*width = w;
}

void LCD_Putc(uint16_t x, uint16_t y, char c, sFONT* font, uint32_t foreground, uint32_t background) {
	  if ((x + font->Width) > LCD_Options.width)
	  {
	      // Here, a check on display height would be in order!
	    y += font->Height;
	    x = 0;
	  }
	  // How many bytes in one font line?
	  uint32_t lineSize = (font->Width + 7) / 8;
	  // char offset in the table
	  uint32_t charOffset = (c - 32) * font->Height * lineSize;
	  // Outer loop, repeat for every line of lineSize bytes in the font
	  for (uint32_t line = 0; line < font->Height; line++)
	  {
	      // Loop over the bytes in one line
	    for (uint32_t byte = 0; byte < lineSize; byte++)
	    {
	        // How many bits are used in this byte?
	      uint32_t stopBit = 8;
	      // Last byte in line?
	      if (byte == lineSize - 1)
	        stopBit = font->Width % 8;
	    // Get the byte
	      uint32_t value = font->table[charOffset + line*lineSize + byte];
	      for (uint32_t bit = 0; bit < stopBit; bit++)
	      {
	          // Draw the pixel, if needed
	        if ((value << bit) & 0x80)
	        {
	        	LCD_DrawPixel(x + byte * 8 + bit, y + line, foreground);
	        }
	        else {
	        	LCD_DrawPixel(x + byte * 8 + bit, y + line, background);
	        }
	      }
	    }
	  }
}

void LCD_DrawLine(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, uint32_t color) {

	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	  curpixel = 0;
	  deltax = abs(X2 - X1);
	  deltay = abs(Y2 - Y1);
	  x = X1;
	  y = Y1;
	  if (X2 >= X1)
	  {
	    xinc1 = 1;
	    xinc2 = 1;
	  }
	  else
	  {
	    xinc1 = -1;
	    xinc2 = -1;
	  }
	  if (Y2 >= Y1)
	  {
	    yinc1 = 1;
	    yinc2 = 1;
	  }
	  else
	  {
	    yinc1 = -1;
	    yinc2 = -1;
	  }
	  if (deltax >= deltay)
	  {
	    xinc1 = 0;
	    yinc2 = 0;
	    den = deltax;
	    num = deltax / 2;
	    numadd = deltay;
	    numpixels = deltax;
	  }
	  else
	  {
	    xinc2 = 0;
	    yinc1 = 0;
	    den = deltay;
	    num = deltay / 2;
	    numadd = deltax;
	    numpixels = deltay;
	  }
	  for (curpixel = 0; curpixel <= numpixels; curpixel++)
	  {
		LCD_DrawPixel(x, y, color);
	    num += numadd;
	    if (num >= den)
	    {
	      num -= den;
	      x += xinc1;
	      y += yinc1;
	    }
	    x += xinc2;
	    y += yinc2;
	  }
}

void LCD_DrawRectangle(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1, uint32_t color) {
	LCD_DrawLine(x0,y0,x1,y0,color);
	LCD_DrawLine(x0,y0,x0,y1,color);
	LCD_DrawLine(x1,y0,x1,y1,color);
	LCD_DrawLine(x0,y1,x1,y1,color);
}

void LCD_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    LCD_DrawPixel(x0, y0 + r, color);
    LCD_DrawPixel(x0, y0 - r, color);
    LCD_DrawPixel(x0 + r, y0, color);
    LCD_DrawPixel(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        LCD_DrawPixel(x0 + x, y0 + y, color);
        LCD_DrawPixel(x0 - x, y0 + y, color);
        LCD_DrawPixel(x0 + x, y0 - y, color);
        LCD_DrawPixel(x0 - x, y0 - y, color);

        LCD_DrawPixel(x0 + y, y0 + x, color);
        LCD_DrawPixel(x0 - y, y0 + x, color);
        LCD_DrawPixel(x0 + y, y0 - x, color);
        LCD_DrawPixel(x0 - y, y0 - x, color);
    }
}

void LCD_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    LCD_DrawPixel(x0, y0 + r, color);
    LCD_DrawPixel(x0, y0 - r, color);
    LCD_DrawPixel(x0 + r, y0, color);
    LCD_DrawPixel(x0 - r, y0, color);
    LCD_DrawLine(x0 - r, y0, x0 + r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        LCD_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        LCD_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

        LCD_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
        LCD_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
    }
}




void SPI_Init(void) {

  hspi.Instance = SPIx;
  hspi.Init.Mode = SPI_MODE_MASTER;
  hspi.Init.Direction = SPI_DIRECTION_2LINES;
  hspi.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi.Init.NSS = SPI_NSS_SOFT;
  hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi);

}


void DMA_Init(void) {

	/* DMA controller clock enable */
  DMAx_CLOCK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);

}

void GPIO_Init(void) {
	WRX_GPIO_CLK_ENABLE();
	CS_GPIO_CLK_ENABLE();
	RST_GPIO_CLK_ENABLE();

	GPIO_InitStruct.Pin = WRX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(WRX_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(CS_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = RST_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW ;
	HAL_GPIO_Init(RST_GPIO_PORT, &GPIO_InitStruct);
}
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {

	SPIxCLK_ENABLE();
	SPIx_SCK_GPIO_CLK_ENABLE();
	SPIx_MISO_GPIO_CLK_ENABLE();
	SPIxMOSI_GPIO_CLK_ENABLE();


	GPIO_InitStruct.Pin = SPIx_SCK_PIN|SPIx_MISO_PIN|SPIx_MOSI_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = SPIx_SCK_AF;
	HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

	hdma_spi_rx.Instance = DMAx_RX_STREAM;
	hdma_spi_rx.Init.Channel = DMAx_RX_CHANNEL;
	hdma_spi_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_spi_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_spi_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_spi_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_spi_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_spi_rx.Init.Mode = DMA_NORMAL;
	hdma_spi_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_spi_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_spi_rx);

	__HAL_LINKDMA(hspi,hdmarx,hdma_spi_rx);

	hdma_spi_tx.Instance = DMAx_TX_STREAM;
	hdma_spi_tx.Init.Channel = DMAx_TX_CHANNEL;
	hdma_spi_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_spi_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_spi_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_spi_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_spi_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_spi_tx.Init.Mode = DMA_NORMAL;
	hdma_spi_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_spi_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_spi_tx);

	__HAL_LINKDMA(hspi,hdmatx,hdma_spi_tx);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {

	SPIxCLK_DISABLE();
	HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
	HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
	HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);


	HAL_DMA_DeInit(hspi->hdmarx);
	HAL_DMA_DeInit(hspi->hdmatx);
}

void Delay(volatile unsigned int delay) {
	for(;delay != 0; delay--);
}
