/*
 * LCD.h
 *
 *  Created on: 19 May 2017
 *      Author: Marcin
 */

/*Inicjalizacja LCD:
 *
 * void LCD_Init();
 */

/*Ustawienie kolorowego kola: color = {LCD_COLOR_RED, LCD_COLOR_GREEN2}
 *
 * void LCD_Circle(uint32_t color);
 */

/*Ustawienie wszystkich wartosci: Val1 - chwilowa; Val2 - max; x,y,z - skladowe; T - temperatura
 *
 * void LCD_SetScene1(uint32_t Val1, uint32_t Val2, uint32_t x, uint32_t y, uint32_t z, uint32_t T)
 */

/*
 * Ustawienie wartosci chwilowej:
 *
 * void LCD_SetVal1(uint32_t num);
 */

/*
 * Ustawienie wartosci chwilowej:
 *
 * void LCD_SetVal2(uint32_t num);
 */

/*
 * Ustawienie skladowej x:
 *
 * void LCD_SetX(uint32_t num);
 */

/*
 * Ustawienie skladowej y:
 *
 * void LCD_SetY(uint32_t num);
 */

/*
 * Ustawienie skladowej z:
 *
 * void LCD_SetZ(uint32_t num);
 */

/*
 * Ustawienie temperatury:
 *
 * void LCD_SetTemp(uint32_t num);
 */


#ifndef LCD_H_
#define LCD_H_

#include "fonts.h"
#include "mag3110.h"


#define LCD_WIDTH  240
#define LCD_HEIGHT 320
#define LCD_PIXEL  76800

#define LCD_COLOR_WHITE			0xFFFF
#define LCD_COLOR_BLACK			0x0000
#define LCD_COLOR_RED           0xF800
#define LCD_COLOR_GREEN			0x07E0
#define LCD_COLOR_GREEN2		0xB723
#define LCD_COLOR_BLUE			0x001F
#define LCD_COLOR_BLUE2			0x051D
#define LCD_COLOR_YELLOW		0xFFE0
#define LCD_COLOR_ORANGE		0xFBE4
#define LCD_COLOR_CYAN			0x07FF
#define LCD_COLOR_MAGENTA		0xA254
#define LCD_COLOR_GRAY			0x7BEF
#define LCD_COLOR_BROWN			0xBBCA
#define LCD_TRANSPARENT			0x80000000


typedef enum {
	LCD_Orientation_Portrait_1,
	LCD_Orientation_Portrait_2,
	LCD_Orientation_Landscape_1,
	LCD_Orientation_Landscape_2
} LCD_Orientation_t;

void lcd_task_routine(void* param);
void LCD_Init(void);
int LCD_InitLCD(void);
void LCD_SetScene1(uint32_t Val1, uint32_t Val2, uint32_t x, uint32_t y, uint32_t z, uint32_t T);
void LCD_SetVal1(uint32_t num);
void LCD_SetVal2(uint32_t num);
void LCD_SetX(int32_t num);
void LCD_SetY(int32_t num);
void LCD_SetZ(int32_t num);
void LCD_SetTemp(int32_t num);
void LCD_SendData(uint8_t data);
void LCD_SendCommand(uint8_t data);
void LCD_SetCursor(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint32_t color);
void LCD_Fill(uint32_t color);
void LCD_Circle(uint32_t color);
void LCD_Rotate(LCD_Orientation_t orientation);
void LCD_Putc(uint16_t x, uint16_t y, char c, sFONT* font, uint32_t foreground, uint32_t background);
void LCD_Puts(uint16_t x, uint16_t y, char* str, sFONT *font, uint32_t foreground, uint32_t background);
void LCD_GetStringSize(char* str, sFONT* font, uint16_t* width, uint16_t* height);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint32_t color);
void LCD_DrawRectangle(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1, uint32_t color);
void LCD_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color);
void LCD_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint32_t color);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void Delay(volatile unsigned int delay);

void LCD_PutMeasurement(const mag3110_data_t* const pMeasurement);
#endif /* LCD_H_ */
