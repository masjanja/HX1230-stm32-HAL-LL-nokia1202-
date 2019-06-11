#ifndef __LCD
#define __LCD

#include "stm32f1xx_hal.h"
#include "math.h"
#include "font.h"

#define LCD_X        96
#define LCD_Y        68
#define LCD_String    9
#define LCD_D         1		// Данные
#define LCD_C         0   // Команда
#define SetYAddr   0xB0
#define SetXAddr4  0x00
#define SetXAddr3  0x10



#define swap(a, b) {uint8_t t = a; a = b; b = t; }

void LCD_SendByte(uint8_t mode, uint8_t c);
void LCD_Clear(void);
void LCD_Update(void);
void LCD_Init(void);
void LCD_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void LCD_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color);
void LCD_DrawFastVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color);
void LCD_DrawFastHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color);
void LCD_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void LCD_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void LCD_FillScreen(uint8_t color);
void LCD_DrawChar(uint8_t x, uint8_t y, uint8_t color, unsigned char c);
void LCD_print(uint8_t x, uint8_t y, uint8_t color, char *str);
void LCD_write(uint8_t x, uint8_t y, uint8_t color, float num);
void LCD_DrawBitmap(uint8_t x, uint8_t y, const char *bitmap, uint8_t w, uint8_t h, uint8_t color);


#endif
