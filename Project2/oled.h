#ifndef OLED_H
#define OLED_H

#include <avr/io.h>

#define OLED_ADDR 0x3C   // change to 0x3D if nothing shows

void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t row, uint8_t col);
void OLED_PrintChar(char c);
void OLED_PrintString(const char *str);
void OLED_DrawBitmap(const uint8_t *bmp, uint8_t pages, uint8_t cols);

#endif