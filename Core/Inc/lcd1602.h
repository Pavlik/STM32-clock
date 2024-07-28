#ifndef __LCD_1602__
#define __LCD_1602__

#include "stm32f1xx.h"
#include <string.h>
#include <stdbool.h>

void lcd1602_Backlight(bool state);
void lcd1602_Init(void);
void lcd1602_Clean(void);
void lcd1602_SetCursor(uint8_t x, uint8_t y);
void lcd1602_Print_symbol(uint8_t symbol);
void lcd1602_Print_text(char *message);
void lcd1602_Move_to_the_left(void);
void lcd1602_Move_to_the_right(void);
void lcd1602_Create_symbol( uint8_t *my_symbol, uint8_t memory_adress);
void lcd1602_Clean_Text(void);



#endif
