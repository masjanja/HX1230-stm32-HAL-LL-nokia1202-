#include "stm32f1xx_hal.h"
#include "main.h"
#include "n1202.h"
#include "stdlib.h"
#include "gpio.h"
//#include "spi.h"
uint8_t _LCD_RAM[LCD_X*LCD_String]; // Память нашего LCD


// Отправляем байт данных дисплею
void LCD_SendByte(uint8_t mode, uint8_t c)
{
  // Опускаем ножку CS для дисплея
	HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_RESET);
  // Формируем первый передаваемый бит - выбор память-команда
	if (mode) {
		HAL_GPIO_WritePin(LED_MOSI_GPIO_Port, LED_MOSI_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(LED_MOSI_GPIO_Port, LED_MOSI_Pin, GPIO_PIN_RESET);
	};
	// Проталкиваем тактовым импульсом
	HAL_GPIO_WritePin(LED_SCK_GPIO_Port, LED_SCK_Pin, GPIO_PIN_SET);
	// В цикле передаем остальные биты
	for(uint8_t i=0; i<8; i++) {
    // Сбрасываем тактовую ножку
    HAL_GPIO_WritePin(LED_SCK_GPIO_Port, LED_SCK_Pin, GPIO_PIN_RESET);
    // Выставляем бит данных
		if (c & 0x80) {
			HAL_GPIO_WritePin(LED_MOSI_GPIO_Port, LED_MOSI_Pin, GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(LED_MOSI_GPIO_Port, LED_MOSI_Pin, GPIO_PIN_RESET);
		};
		// Тактом ее, тактом
		HAL_GPIO_WritePin(LED_SCK_GPIO_Port, LED_SCK_Pin, GPIO_PIN_SET);
		// Сдвигаем данные
    c <<= 1;
	};
	HAL_GPIO_WritePin(LED_SCK_GPIO_Port, LED_SCK_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_SET);
}

/*
void LCD_SendByte(uint8_t mode, uint8_t c)
{
	uint8_t SPI_Data[2];
	HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_RESET);
	SPI_Data[0] = c;
	SPI_Data[1] = mode;

	HAL_Delay(1);
	HAL_GPIO_WritePin(LED_MOSI_GPIO_Port, LED_MOSI_Pin, mode);
	HAL_GPIO_WritePin(LED_MOSI_GPIO_Port, LED_MOSI_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LED_SCK_GPIO_Port, LED_SCK_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LED_SCK_GPIO_Port, LED_SCK_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1, SPI_Data, 1, 5);

	HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_SET);

}
*/

// Очистка памяти дисплея
void LCD_Clear(void) {
  for (int index = 0; index < 864 ; index++){
    _LCD_RAM[index] = (0x00);
  }
}

// Обновляем данные на экране
void LCD_Update(void) {
  for(uint8_t p = 0; p < 9; p++) {
    LCD_SendByte(LCD_C, SetYAddr | p);
    LCD_SendByte(LCD_C, SetXAddr4);
    LCD_SendByte(LCD_C, SetXAddr3);
    for(uint8_t col=0; col < LCD_X; col++){
      LCD_SendByte(LCD_D, _LCD_RAM[(LCD_X * p) + col]);
    }
  }
}

// Рисование пикселя по координатам и цвету
void LCD_DrawPixel (uint8_t x, uint8_t y, uint8_t color) {
  if ((x < 0) || (x >= LCD_X) || (y < 0) || (y >= LCD_Y)) return;

  if (color) _LCD_RAM[x+ (y/8)*LCD_X] |= 1<<(y%8);
  else       _LCD_RAM[x+ (y/8)*LCD_X] &= ~(1<<(y%8));
}

// Рисование линии
void LCD_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color) {
  int steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }
  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }
  int dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);
  int err = dx / 2;
  int ystep;
  if (y0 < y1) {ystep = 1;}
  else {ystep = -1;};
  for ( ; x0 <= x1; x0++) {
    if (steep) {LCD_DrawPixel(y0, x0, color);}
    else {LCD_DrawPixel(x0, y0, color);};
		err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// Рисование вертикальной линии
void LCD_DrawFastVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color) {
  LCD_DrawLine(x, y, x, y+h-1, color);
}

// Рисование горизонтальной линии
void LCD_DrawFastHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color) {
  LCD_DrawLine(x, y, x+w-1, y, color);
}

// Рисование прямоугольника
void LCD_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
  LCD_DrawFastHLine(x, y, w, color);
  LCD_DrawFastHLine(x, y+h-1, w, color);
  LCD_DrawFastVLine(x, y, h, color);
  LCD_DrawFastVLine(x+w-1, y, h, color);
}

// Рисование залитый прямоугольник
void LCD_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
  for (int16_t i=x; i<x+w; i++) {
    LCD_DrawFastVLine(i, y, h, color);
  }
}

// Заливка экрана
void LCD_FillScreen(uint8_t color) {
  LCD_FillRect(0, 0, LCD_X, LCD_Y, color);
}

// Нарисовать букву
void LCD_DrawChar(uint8_t x, uint8_t y, uint8_t color, unsigned char c) {
  if((x >= LCD_X) ||(y >= LCD_Y) || ((x + 4) < 0) || ((y + 7) < 0)) return;
  if(c<128)            c = c-32;
  if(c>=144 && c<=175) c = c-48;
  if(c>=128 && c<=143) c = c+16;
  if(c>=176 && c<=191) c = c-48;
  if(c>191)  return;
  for (uint8_t i=0; i<6; i++ ) {
    uint8_t line;
    if (i == 5) {line = 0x00;}
    else {line = font[(c*5)+i];
    for (uint8_t j = 0; j<8; j++)
			{
				if (line & 0x01) {LCD_DrawPixel(x+i, y+j, color);}
				else {LCD_DrawPixel(x+i, y+j, !color);};
				line >>= 1;
			}
		}
  }
}

// Вывод строки
void LCD_print(uint8_t x, uint8_t y, uint8_t color, char *str) {
  unsigned char type = *str;
  if(type>=128) x=x-3;
  while(*str){
    LCD_DrawChar(x, y, color, *str++);
    unsigned char type = *str;
    if (type>=128) {x=x+3;}
    else {x=x+6;};
  }
}

// Вывод числовых значений
void LCD_write(uint8_t x, uint8_t y, uint8_t color, float num){
  char c[10];
//	sprintf(c, "text %f\n", num);
	sprintf(c, "%5.2f", num);
  LCD_print(x, y, color, c);
}

// Вывод картинки
void LCD_DrawBitmap(uint8_t x, uint8_t y, const char *bitmap, uint8_t w, uint8_t h, uint8_t color) {
  for (int16_t j=0; j<h; j++) {
    for (int16_t i=0; i<w; i++ ) {
      if (bitmap[i + (j/8)*w] & 1<<(j%8)) { LCD_DrawPixel(x+i, y+j, color); }
    }
  }
}


// Инициализируем дисплей
void LCD_Init(void) {
  // Инициализация дисплея
	HAL_GPIO_WritePin(LED_RESET_GPIO_Port, LED_RESET_Pin, GPIO_PIN_RESET); // Активируем ресет
	HAL_Delay(15);
	HAL_GPIO_WritePin(LED_RESET_GPIO_Port, LED_RESET_Pin, GPIO_PIN_SET);   // Деактивируем ресет
	HAL_GPIO_WritePin(LED_SCK_GPIO_Port, LED_SCK_Pin, GPIO_PIN_RESET);     // Тактовый вывод низкое состояние


	HAL_GPIO_WritePin(LED_MOSI_GPIO_Port, LED_MOSI_Pin, GPIO_PIN_RESET);   // Вывод данных в низкое состояние
	HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_RESET);			 // Выбираем дисплей
	// Задержка
  	HAL_Delay(5);
	HAL_GPIO_WritePin(LED_CS_GPIO_Port, LED_CS_Pin, GPIO_PIN_SET);			 // Выбираем дисплей
	LCD_SendByte(LCD_C, 0xE2);  // Сброс чипа

	HAL_Delay(5);
  // Устанавливаем энергию заряда сегмента
	LCD_SendByte(LCD_C, 0x3D);  // Умножитель энергии заряда
	LCD_SendByte(LCD_C, 0x02); 	// Не понятное значение умножителя
  // Команда и следом данные по контрастности
	LCD_SendByte(LCD_C, 0xE1);  // Additional VOP for contrast increase
 	LCD_SendByte(LCD_C, 0x90);  // from -127 to +127
  // Устанавливаем режим работы Normal
 	LCD_SendByte(LCD_C, 0xA4);  // Power saver off

	LCD_SendByte(LCD_C, 0x2F);  // Booster ON Voltage regulator ON Voltage follover ON
	LCD_SendByte(LCD_C, 0xA0);  // Segment driver direction select: Normal
	LCD_SendByte(LCD_C, 0xAF);  // Включение дисплея
	HAL_Delay(10);
  // Очищаем, обновляем

  LCD_Clear();
  LCD_Update();
}
