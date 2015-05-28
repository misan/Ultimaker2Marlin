#ifndef ULTI_LCD2_H
#define ULTI_LCD2_H

#include "Configuration.h"

#ifdef ENABLE_ULTILCD2
#include "UltiLCD2_low_lib.h"
#include "Marlin.h"

extern bool serialScreenShown;

typedef  const PROGMEM char * ppstr;

void lcd_init();
void lcd_update();

void lcd_buttons_update();
FORCE_INLINE void lcd_reset_alert_level() {}
FORCE_INLINE void lcd_buzz(long duration,uint16_t freq) {}

//-----------------------------------------------------------------------------------------------------------------
void lcd_main_screensaver();
//-----------------------------------------------------------------------------------------------------------------
char* drawMaterialInfoScreen( char* c, char * buffer );
//-----------------------------------------------------------------------------------------------------------------
char* drawSystemInfoScreen( char* c, char * buffer );
//-----------------------------------------------------------------------------------------------------------------
char* drawLastPrintInfo( char * buffer, char* c );
//-----------------------------------------------------------------------------------------------------------------
void drawCustomFirmwareInfo();

extern unsigned long lastSerialCommandTime;
extern uint8_t led_brightness_level;
extern uint8_t led_mode;

#define LED_MODE_ALWAYS_ON      0
#define LED_MODE_ALWAYS_OFF     1
#define LED_MODE_WHILE_PRINTING 2
#define LED_MODE_BLINK_ON_DONE  3

void lcd_menu_main();

#endif

#endif//ULTI_LCD2_H
