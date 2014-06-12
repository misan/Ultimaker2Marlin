#ifndef ULTI_LCD2_H
#define ULTI_LCD2_H

#include "Configuration.h"

#ifdef ENABLE_ULTILCD2
#include "UltiLCD2_low_lib.h"
#include "..\..\..\Users\Lars\AppData\Local\VMicro\Arduino\Builds\Marlin\mega_atmega2560\Marlin.h"

#define MAX_MESSAGE_LEN 20
extern char message_string [MAX_MESSAGE_LEN+1];
extern int message_counter ;
extern bool serialScreenShown;
#define DEFAULT_MESSAGE_DURATION 500

FORCE_INLINE bool is_message_shown () { return message_counter > 0; };
FORCE_INLINE void clear_message() { message_counter = 0; message_string[0] = 0;};

typedef  const PROGMEM char * ppstr;

void lcd_init();
void lcd_update();
void lcd_setstatus(const char* message);
void lcd_setstatusP(ppstr message);
void lcd_buttons_update();
FORCE_INLINE void lcd_reset_alert_level() {}
FORCE_INLINE void lcd_buzz(long duration,uint16_t freq) {}

#define LCD_MESSAGEPGM(x) {lcd_setstatusP(x);}
#define LCD_ALERTMESSAGEPGM(x) { lcd_setstatusP(x); lcd_lib_beep_ext(200,500);}; 

extern unsigned long lastSerialCommandTime;
extern uint8_t led_brightness_level;
extern uint8_t led_mode;
#define LED_MODE_ALWAYS_ON      0
#define LED_MODE_ALWAYS_OFF     1
#define LED_MODE_WHILE_PRINTING 2
#define LED_MODE_BLINK_ON_DONE  3

void lcd_menu_main();
bool lcd_lib_show_message(int position, bool decrement = true);
#endif

#endif//ULTI_LCD2_H
