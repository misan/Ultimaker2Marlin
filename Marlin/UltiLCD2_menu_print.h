#ifndef ULTI_LCD2_MENU_PRINT_H
#define ULTI_LCD2_MENU_PRINT_H

#include "cardreader.h"

#ifdef  __AVR_ATmega2560__
#define LCD_CACHE_COUNT 16		// we can't show more than 6, but having a bigger buffer makes scrolling through menus faster
#else
#define LCD_CACHE_COUNT 6
#endif 

#define LCD_DETAIL_CACHE_SIZE (5+4*EXTRUDERS)
#define LCD_CACHE_SIZE (1 + (2 + LONG_FILENAME_LENGTH) * LCD_CACHE_COUNT + LCD_DETAIL_CACHE_SIZE)
extern uint8_t lcd_cache[LCD_CACHE_SIZE];

void lcd_menu_print_select();
void lcd_clear_cache();
void doCancelPrint();

FORCE_INLINE void ERROR_BEEP() 
	{
	lcd_lib_beep_ext(110,400);
	}
#endif//ULTI_LCD2_MENU_PRINT_H
