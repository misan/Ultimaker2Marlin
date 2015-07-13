#ifndef ULTI_LCD2_LOW_LIB_H
#define ULTI_LCD2_LOW_LIB_H

#include <stdint.h>
#include <stddef.h>

#define LCD_GFX_WIDTH 128
#define LCD_GFX_HEIGHT 64


// these are in OCTAL, for your convenience.
#define DEGREE_C_SYMBOL "\x81"
#define ELIPSIS_SYMBOL '\x82'
#define SQUARED_SYMBOL "\x80"
#define CUBED_SYMBOL "\x7f"
#define PER_SECOND_SYMBOL "/s" 

// this is what appears between two temperature values (ie: current and target temperatures) 
// Octal 176 is 126 binary, which is a right arrow symbol in the lcd's font
#define TEMPERATURE_SEPARATOR '~'
#define TEMPERATURE_SEPARATOR_S "~"

extern bool i2Ctimeout;


void updateI2C();
void lcd_lib_init();
void lcd_lib_update_screen();   /* Start sending out the display buffer to the screen. Wait till lcd_lib_update_ready before issuing any draw functions */
bool lcd_lib_update_ready();
inline void lcd_lib_wait_for_screen_ready () { while (!lcd_lib_update_ready()) delay(1); } ;
bool lcd_lib_button_pressed();
void lcd_lib_draw_string(uint8_t x, uint8_t y, const char* str);
void lcd_lib_clear_string(uint8_t x, uint8_t y, const char* str);
void lcd_lib_draw_string_center(uint8_t y, const char* str);
void lcd_lib_clear_string_center(uint8_t y, const char* str);
void lcd_lib_draw_stringP(uint8_t x, uint8_t y, const char* pstr, bool clear=false);
void lcd_lib_clear_stringP(uint8_t x, uint8_t y, const char* pstr);
void lcd_lib_draw_string_centerP(uint8_t y, const char* pstr);
void lcd_lib_clear_string_centerP(uint8_t y, const char* pstr);
void lcd_lib_draw_string_center_atP(uint8_t x, uint8_t y, const char* pstr);
void lcd_lib_clear_string_center_atP(uint8_t x, uint8_t y, const char* pstr);
void lcd_lib_draw_vline(uint8_t x, uint8_t y0, uint8_t y1);
void lcd_lib_draw_hline(uint8_t x0, uint8_t x1, uint8_t y);
void lcd_lib_draw_box(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void lcd_lib_draw_shade(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void lcd_lib_clear();
void lcd_lib_clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void lcd_lib_invert(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void lcd_lib_set();
void lcd_lib_set(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void lcd_lib_draw_gfx(uint8_t x, uint8_t y, const uint8_t* gfx);
void lcd_lib_clear_gfx(uint8_t x, uint8_t y, const uint8_t* gfx);

void lcd_lib_beep();
void lcd_lib_buttons_update();
void lcd_lib_buttons_update_interrupt();
void lcd_lib_led_color(uint8_t r, uint8_t g, uint8_t b, bool forced=false);

extern int16_t lcd_lib_encoder_pos;
extern int8_t lcd_lib_encoder_pos_interrupt;

extern bool lcd_lib_button_down;


void lcd_lib_beep_ext( unsigned int freq, unsigned int dur );
void lcd_lib_tick();
void lcd_lib_draw_string_right(uint8_t y, const char* str, byte startpos = 126);
void lcd_lib_draw_dotted_hline(uint8_t x0, uint8_t x1, uint8_t y);

//-----------------------------------------------------------------------------------------------------------------

void lcd_lib_update_RGB_LED();
#endif//ULTI_LCD2_LOW_LIB_H
