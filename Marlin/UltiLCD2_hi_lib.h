#ifndef ULTI_LCD2_HI_LIB_H
#define ULTI_LCD2_HI_LIB_H

#include "UltiLCD2_low_lib.h"
#include "UltiLCD2_gfx.h"

typedef void (*menuFunc_t)();
typedef char* (*entryNameCallback_t)(uint8_t nr);
typedef void (*entryDetailsCallback_t)(uint8_t nr);

extern char main_menu_x_offset;

#define ENCODER_TICKS_PER_MAIN_MENU_ITEM 8
#define ENCODER_TICKS_PER_SCROLL_MENU_ITEM 4
#define ENCODER_NO_SELECTION (ENCODER_TICKS_PER_MAIN_MENU_ITEM * -11)
#define MAIN_MENU_ITEM_POS(n)  (main_menu_x_offset+(ENCODER_TICKS_PER_MAIN_MENU_ITEM * (n) + ENCODER_TICKS_PER_MAIN_MENU_ITEM / 2))
#define SCROLL_MENU_ITEM_POS(n)  (ENCODER_TICKS_PER_SCROLL_MENU_ITEM * (n) + ENCODER_TICKS_PER_SCROLL_MENU_ITEM / 2)
#define SELECT_MAIN_MENU_ITEM(n)  do { lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(n); } while(0)
#define SELECT_SCROLL_MENU_ITEM(n)  do { lcd_lib_encoder_pos = SCROLL_MENU_ITEM_POS(n); } while(0)
#define SELECTED_MAIN_MENU_ITEM() (lcd_lib_encoder_pos / ENCODER_TICKS_PER_MAIN_MENU_ITEM)
#define SELECTED_SCROLL_MENU_ITEM() (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM)
#define IS_SELECTED_MAIN(n) ((n) == SELECTED_MAIN_MENU_ITEM())
#define IS_SELECTED_SCROLL(n) ((n) == SELECTED_SCROLL_MENU_ITEM())

void lcd_change_to_menu(menuFunc_t nextMenu, int16_t newEncoderPos = ENCODER_NO_SELECTION, bool saveback= true);

void lcd_tripple_menu(const char* left, const char* right, const char* bottom);
void lcd_basic_screen();
void lcd_info_screen(menuFunc_t cancelMenu, menuFunc_t callbackOnCancel = NULL, const char* cancelButtonText = NULL);
void lcd_question_screen(menuFunc_t optionAMenu, menuFunc_t callbackOnA, const char* AButtonText, menuFunc_t optionBMenu, menuFunc_t callbackOnB, const char* BButtonText);
void lcd_scroll_menu(const char* menuNameP, int8_t entryCount, entryNameCallback_t entryNameCallback, entryDetailsCallback_t entryDetailsCallback);

void lcd_progressbar(uint8_t progress);
//-----------------------------------------------------------------------------------------------------------------
void lcd_low_triple_menu_draw( const char* left, const char* middle, const char* right, char offset=0 );
void lcd_triple_X_menu_low(const char* const * items,byte item_count);

void lcd_menu_edit_setting();
extern bool allow_encoder_acceleration;
FORCE_INLINE void lcd_lib_enable_encoder_acceleration( bool enabled ) 	{ allow_encoder_acceleration = enabled; }
void lcd_triple_menu_low(const char* left, const char* middle, const char* right);
void lcd_menu_go_back();
extern const char* lcd_setting_name;
extern const char* lcd_setting_postfix;
extern void* lcd_setting_ptr;
extern uint8_t lcd_setting_type;
extern int16_t lcd_setting_min;
extern int16_t lcd_setting_max;

#define MENU_DEPTH 10
void lcd_menu_clear_back () ;
void lcd_do_current_menu();
extern menuFunc_t postMenuCheck;
extern int16_t previousEncoderPos[MENU_DEPTH];
extern uint8_t minProgress;

#define LCD_EDIT_SETTING(_setting, _name, _postfix, _min, _max) do { \
            lcd_change_to_menu(lcd_menu_edit_setting); \
            lcd_setting_name = PSTR(_name); \
            lcd_setting_postfix = PSTR(_postfix); \
            lcd_setting_ptr = &_setting; \
            lcd_setting_type = sizeof(_setting); \
            lcd_lib_encoder_pos = _setting; \
            lcd_setting_min = _min; \
            lcd_setting_max = _max; \
	} while(0)
#define LCD_EDIT_SETTING_BYTE_PERCENT(_setting, _name, _postfix, _min, _max) do { \
            lcd_change_to_menu(lcd_menu_edit_setting); \
            lcd_setting_name = PSTR(_name); \
            lcd_setting_postfix = PSTR(_postfix); \
            lcd_setting_ptr = &_setting; \
            lcd_setting_type = 5; \
            lcd_lib_encoder_pos = int(_setting) * 100 / 255; \
            lcd_setting_min = _min; \
            lcd_setting_max = _max; \
	} while(0)
#define LCD_EDIT_SETTING_FAN_OVERRIDE(_setting, _name, _postfix, _min, _max) do { \
	lcd_change_to_menu(lcd_menu_edit_setting); \
	lcd_setting_name = PSTR(_name); \
	lcd_setting_postfix = PSTR(_postfix); \
	lcd_setting_ptr = &_setting; \
	lcd_setting_type = 9; \
	lcd_lib_encoder_pos = int(_setting) * 100 / 255; \
	lcd_setting_min = _min; \
     lcd_setting_max = _max; \
	} while(0)
#define LCD_EDIT_SETTING_FLOAT01(_setting, _name, _postfix, _min, _max) do { \
	lcd_change_to_menu(lcd_menu_edit_setting); \
	lcd_setting_name = PSTR(_name); \
	lcd_setting_postfix = PSTR(_postfix); \
	lcd_setting_ptr = &_setting; \
	lcd_setting_type = 10; \
	lcd_lib_encoder_pos = (_setting) * 10.0 + 0.5; \
	lcd_setting_min = (_min) * 10; \
	lcd_setting_max = (_max) * 10; \
	} while(0)
#define LCD_EDIT_SETTING_FLOAT001(_setting, _name, _postfix, _min, _max) do { \
            lcd_change_to_menu(lcd_menu_edit_setting); \
            lcd_setting_name = PSTR(_name); \
            lcd_setting_postfix = PSTR(_postfix); \
            lcd_setting_ptr = &_setting; \
            lcd_setting_type = 3; \
            lcd_lib_encoder_pos = (_setting) * 100.0 + 0.5; \
            lcd_setting_min = (_min) * 100; \
            lcd_setting_max = (_max) * 100; \
        } while(0)
#define LCD_EDIT_SETTING_FLOAT100(_setting, _name, _postfix, _min, _max) do { \
            lcd_change_to_menu(lcd_menu_edit_setting); \
            lcd_setting_name = PSTR(_name); \
            lcd_setting_postfix = PSTR("00" _postfix); \
            lcd_setting_ptr = &(_setting); \
            lcd_setting_type = 7; \
            lcd_lib_encoder_pos = (_setting) / 100 + 0.5; \
            lcd_setting_min = (_min) / 100 + 0.5; \
            lcd_setting_max = (_max) / 100 + 0.5; \
        } while(0)
#define LCD_EDIT_SETTING_FLOAT1(_setting, _name, _postfix, _min, _max) do { \
            lcd_change_to_menu(lcd_menu_edit_setting); \
            lcd_setting_name = PSTR(_name); \
            lcd_setting_postfix = PSTR(_postfix); \
            lcd_setting_ptr = &(_setting); \
            lcd_setting_type = 8; \
            lcd_lib_encoder_pos = (_setting) + 0.5; \
            lcd_setting_min = (_min) + 0.5; \
            lcd_setting_max = (_max) + 0.5; \
        } while(0)
#define LCD_EDIT_SETTING_SPEED(_setting, _name, _postfix, _min, _max) do { \
            lcd_change_to_menu(lcd_menu_edit_setting); \
            lcd_setting_name = PSTR(_name); \
            lcd_setting_postfix = PSTR(_postfix); \
            lcd_setting_ptr = &(_setting); \
            lcd_setting_type = 6; \
            lcd_lib_encoder_pos = (_setting) / 60 + 0.5; \
            lcd_setting_min = (_min) / 60 + 0.5; \
            lcd_setting_max = (_max) / 60 + 0.5; \
        } while(0)

extern uint8_t led_glow;
extern uint8_t led_glow_dir;
#define LED_NORMAL() lcd_lib_led_color(48,48,60)
#define LED_FLASH() lcd_lib_led_color(8 + (led_glow<<3), 8 + min(255-8,(led_glow<<3)), 32 + min(255-32,led_glow<<3))
#define LED_GLOW() lcd_lib_led_color(8 + led_glow, 8 + led_glow, 32 + led_glow)
#define LED_HEAT() lcd_lib_led_color(192 + led_glow/4, 8 + led_glow/4, 0)
#define LED_DONE() lcd_lib_led_color(0, 8 + led_glow, 8)
#define LED_COOL() lcd_lib_led_color(0, 4,16 + led_glow)
#define LED_GLOW_ERROR() lcd_lib_led_color(8+min(245,led_glow<<3),0,0,1);
#define LED_CLEAR_ERROR() lcd_lib_led_color(0,0,0,1);
#define LED_WHITE() lcd_lib_led_color(255,255,255,false);
#endif//ULTI_LCD2_HI_LIB_H
