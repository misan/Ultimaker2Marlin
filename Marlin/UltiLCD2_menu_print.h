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
char * EchoTimeSpan (unsigned long t, char * buffer, bool seconds= true);
void drawTempHistory(byte x1, byte y1, byte x2, byte y2, char *data);
char* drawSpeedAndFlow( char * buffer, char* c , byte y);
// these are used by the menus to time slot fields
#define time_phase1 ((millis() >> 11) & 1)
#define time_phase0 (!time_phase1)

#define time_phase_a   (((millis() >> 13) & 3)==0)
#define time_phase_b  (((millis() >> 13) & 3)==1)
#define time_phase_c  (((millis() >> 13) & 3)==2)
#define time_phase_d  (((millis() >> 13) & 3)==3)



#define HISTORY_SIZE (( DISPLAY_RIGHT / 2) -5)
extern char temp_history[HISTORY_SIZE];
extern char bed_history[HISTORY_SIZE];



#endif//ULTI_LCD2_MENU_PRINT_H
