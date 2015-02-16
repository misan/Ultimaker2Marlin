#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
#include "UltiLCD2.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_gfx.h"
#include "UltiLCD2_menu_maintenance.h"
#include "UltiLCD2_menu_first_run.h"
#include "UltiLCD2_menu_material.h"
#include "Marlin.h"
#include "cardreader.h"
#include "lifetime_stats.h"
#include "ConfigurationStore.h"
#include "temperature.h"
#include "pins.h"
#include "UltiLCD2_menu_print.h"


static void lcd_menu_maintenance_advanced();
static void lcd_menu_maintenance_advanced_heatup();
void lcd_menu_maintenance_advanced_bed_heatup();
static void lcd_menu_maintenance_led();
static void lcd_menu_maintenance_extrude();
static void lcd_menu_maintenance_retraction();
static void lcd_menu_advanced_version();
static void lcd_menu_advanced_stats();
static void lcd_menu_maintenance_motion();
static void lcd_menu_advanced_factory_reset();
static void lcd_menu_advanced_materials_reset();
static void  doMaterialsReset();
extern bool allow_encoder_acceleration;

void lcd_menu_maintenance()
{
    lcd_tripple_menu(PSTR("BUILD-|PLATE"), PSTR("ADVANCED"), PSTR("RETURN"));
	LED_NORMAL();

    if (lcd_lib_button_pressed)
    {
        if (IS_SELECTED_MAIN(0))
            lcd_change_to_menu(lcd_menu_first_run_start_bed_leveling);
        else if (IS_SELECTED_MAIN(1))
            lcd_change_to_menu(lcd_menu_maintenance_advanced);
        else if (IS_SELECTED_MAIN(2))
            lcd_change_to_menu(lcd_menu_main);
    }

    lcd_lib_update_screen();
}

static char* lcd_advanced_item(uint8_t nr)
{
    if (nr == 0)
        strcpy_P(card.longFilename, PSTR("< RETURN"));
    else if (nr == 1)
        strcpy_P(card.longFilename, PSTR("LED settings"));
    else if (nr == 2)
#if EXTRUDERS < 2
        strcpy_P(card.longFilename, PSTR("Heatup nozzle"));
#else
        strcpy_P(card.longFilename, PSTR("Heatup first nozzle"));
    else if (nr == 3)
        strcpy_P(card.longFilename, PSTR("Heatup second nozzle"));
#endif
    else if (nr == 2 + EXTRUDERS)
        strcpy_P(card.longFilename, PSTR("Heatup buildplate"));
    else if (nr == 3 + EXTRUDERS)
        strcpy_P(card.longFilename, PSTR("Home head"));
    else if (nr == 4 + EXTRUDERS)
        strcpy_P(card.longFilename, PSTR("Lower buildplate"));
    else if (nr == 5 + EXTRUDERS)
        strcpy_P(card.longFilename, PSTR("Raise buildplate"));
    else if (nr == 6 + EXTRUDERS)
#if EXTRUDERS < 2
        strcpy_P(card.longFilename, PSTR("Move material"));
#else
        strcpy_P(card.longFilename, PSTR("Move material (1)"));
    else if (nr == 7 + EXTRUDERS)
        strcpy_P(card.longFilename, PSTR("Move material (2)"));
#endif
    else if (nr == 6 + EXTRUDERS * 2)
        strcpy_P(card.longFilename, PSTR("Retraction settings"));
    else if (nr == 7 + EXTRUDERS * 2)
        strcpy_P(card.longFilename, PSTR("Motion settings"));
    else if (nr == 8 + EXTRUDERS * 2)
        strcpy_P(card.longFilename, PSTR("Version"));
    else if (nr == 9 + EXTRUDERS * 2)
        strcpy_P(card.longFilename, PSTR("Runtime stats"));
    else if (nr == 10 + EXTRUDERS * 2)
        strcpy_P(card.longFilename, PSTR("Factory reset"));
	else if (nr == 11 + EXTRUDERS * 2)
		strcpy_P(card.longFilename, PSTR("Reset Materials"));
	else
        strcpy_P(card.longFilename, PSTR("???"));
    return card.longFilename;
}

static void lcd_advanced_details(uint8_t nr)
{
}

static void lcd_menu_maintenance_advanced()
{
    lcd_scroll_menu(PSTR("ADVANCED"), 12 + EXTRUDERS * 2, lcd_advanced_item, lcd_advanced_details);
	LED_NORMAL();

    if (lcd_lib_button_pressed)
    {
        if (IS_SELECTED_SCROLL(0))
            lcd_change_to_menu(lcd_menu_maintenance);
        else if (IS_SELECTED_SCROLL(1))
            lcd_change_to_menu(lcd_menu_maintenance_led, 0);
        else if (IS_SELECTED_SCROLL(2))
        {
            active_extruder = 0;
            lcd_change_to_menu(lcd_menu_maintenance_advanced_heatup, 0);
        }
#if EXTRUDERS > 1
        else if (IS_SELECTED_SCROLL(3))
        {
            active_extruder = 1;
            lcd_change_to_menu(lcd_menu_maintenance_advanced_heatup, 0);
        }
#endif
        else if (IS_SELECTED_SCROLL(2 + EXTRUDERS))
            lcd_change_to_menu(lcd_menu_maintenance_advanced_bed_heatup, 0);
        else if (IS_SELECTED_SCROLL(3 + EXTRUDERS))
        {
            lcd_lib_beep();
            enquecommand_P(PSTR("G28 X0 Y0"));
        }
        else if (IS_SELECTED_SCROLL(4 + EXTRUDERS))
        {
            lcd_lib_beep();
            enquecommand_P(PSTR("G28 Z0"));
        }
        else if (IS_SELECTED_SCROLL(5 + EXTRUDERS))
        {
            lcd_lib_beep();
            enquecommand_P(PSTR("G28 Z0"));
            enquecommand_P(PSTR("G1 Z40"));
        }
        else if (IS_SELECTED_SCROLL(6 + EXTRUDERS))
        {
            set_extrude_min_temp(0);
            active_extruder = 0;
            target_temperature[active_extruder] = material[active_extruder].temperature;
            lcd_change_to_menu(lcd_menu_maintenance_extrude, 0);
        }
#if EXTRUDERS > 1
        else if (IS_SELECTED_SCROLL(7 + EXTRUDERS))
        {
            set_extrude_min_temp(0);
            active_extruder = 1;
            target_temperature[active_extruder] = material[active_extruder].temperature;
            lcd_change_to_menu(lcd_menu_maintenance_extrude, 0);
        }
#endif
        else if (IS_SELECTED_SCROLL(6 + EXTRUDERS * 2))
            lcd_change_to_menu(lcd_menu_maintenance_retraction, SCROLL_MENU_ITEM_POS(0));
        else if (IS_SELECTED_SCROLL(7 + EXTRUDERS * 2))
            lcd_change_to_menu(lcd_menu_maintenance_motion, SCROLL_MENU_ITEM_POS(0));
        else if (IS_SELECTED_SCROLL(8 + EXTRUDERS * 2))
            lcd_change_to_menu(lcd_menu_advanced_version, SCROLL_MENU_ITEM_POS(0));
        else if (IS_SELECTED_SCROLL(9 + EXTRUDERS * 2))
            lcd_change_to_menu(lcd_menu_advanced_stats, SCROLL_MENU_ITEM_POS(0));
        else if (IS_SELECTED_SCROLL(10 + EXTRUDERS * 2))
            lcd_change_to_menu(lcd_menu_advanced_factory_reset, SCROLL_MENU_ITEM_POS(1));
		else if (IS_SELECTED_SCROLL(11 + EXTRUDERS * 2))
			lcd_change_to_menu(lcd_menu_advanced_materials_reset, SCROLL_MENU_ITEM_POS(1));
    }
}

static void lcd_menu_maintenance_advanced_heatup()
{
    if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
    {
        target_temperature[active_extruder] = int(target_temperature[active_extruder]) + (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM);
        if (target_temperature[active_extruder] < 0)
            target_temperature[active_extruder] = 0;
        if (target_temperature[active_extruder] > HEATER_0_MAXTEMP - 15)
            target_temperature[active_extruder] = HEATER_0_MAXTEMP - 15;
        lcd_lib_encoder_pos = 0;
    }
    if (lcd_lib_button_pressed)
        lcd_change_to_menu(previousMenu, previousEncoderPos);
    lcd_lib_enable_encoder_acceleration(true);
	LED_HEAT();
    lcd_lib_clear();
    lcd_lib_draw_string_centerP(ROW2, PSTR("Nozzle temperature:"));
    lcd_lib_draw_string_centerP(ROW7, PSTR("Click to return"));
    char buffer[16];
    int_to_string(int(current_temperature[active_extruder]), buffer, PSTR( TEMPERATURE_SEPARATOR_S));
    int_to_string(int(target_temperature[active_extruder]), buffer+strlen(buffer), PSTR( DEGREE_C_SYMBOL ));
    lcd_lib_draw_string_center(ROW3, buffer);
    drawTempHistory (3+DISPLAY_RIGHT/4,ROW4+2,3*DISPLAY_RIGHT/4,ROW7-3,temp_history);
    lcd_lib_update_screen();
}

void lcd_menu_maintenance_extrude()
{
    if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
    {
        if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 3)
        {
            current_position[E_AXIS] += lcd_lib_encoder_pos * 0.1;

            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 10, active_extruder);
            lcd_lib_encoder_pos = 0;
        }
    }
    if (lcd_lib_button_pressed)
    {
        set_extrude_min_temp(EXTRUDE_MINTEMP);
        target_temperature[active_extruder] = 0;
        lcd_change_to_menu(previousMenu, previousEncoderPos);
    }
    lcd_lib_enable_encoder_acceleration(false);
    lcd_lib_clear();
	LED_GLOW();


    lcd_lib_draw_string_centerP(ROW2, PSTR("Nozzle temperature:"));
    lcd_lib_draw_string_centerP(ROW3, PSTR("Rotate to extrude"));		// TODO:  Make this string swap out with actual amounyt extruded when it's been moved
    lcd_lib_draw_string_centerP(ROW7, PSTR("Click to return"));
    char buffer[16];
    int_to_string(int(current_temperature[active_extruder]), buffer, PSTR( TEMPERATURE_SEPARATOR_S));
    int_to_string(int(target_temperature[active_extruder]), buffer+strlen(buffer), PSTR( DEGREE_C_SYMBOL ));
    lcd_lib_draw_string_center(ROW4, buffer);

	    drawTempHistory (3+DISPLAY_RIGHT/4,ROW4+2,3*DISPLAY_RIGHT/4,ROW7-3,temp_history);

    lcd_lib_update_screen();
}

void lcd_menu_maintenance_advanced_bed_heatup()
{
    if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
    {
        target_temperature_bed = int(target_temperature_bed) + (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM);
        if (target_temperature_bed < 0)
            target_temperature_bed = 0;
        if (target_temperature_bed > BED_MAXTEMP - 15)
            target_temperature_bed = BED_MAXTEMP - 15;
        lcd_lib_encoder_pos = 0;
    }
	LED_HEAT();
    if (lcd_lib_button_pressed)
        lcd_change_to_menu(previousMenu, previousEncoderPos);
    lcd_lib_enable_encoder_acceleration(true);
    lcd_lib_clear();
    lcd_lib_draw_string_centerP(ROW2, PSTR("Buildplate temp.:"));
    lcd_lib_draw_string_centerP(ROW7, PSTR("Click to return"));
    char buffer[16];
    int_to_string(int(current_temperature_bed), buffer, PSTR( TEMPERATURE_SEPARATOR_S));
    int_to_string(int(target_temperature_bed), buffer+strlen(buffer), PSTR( DEGREE_C_SYMBOL ));
    lcd_lib_draw_string_center(ROW3, buffer);
	drawTempHistory (3+DISPLAY_RIGHT/4,ROW4+2,3*DISPLAY_RIGHT/4,ROW7-3,temp_history);


    lcd_lib_update_screen();
}

void lcd_menu_advanced_version()
{
    lcd_info_screen(previousMenu, NULL, PSTR("Return"));
    lcd_lib_draw_string_centerP(30, PSTR(STRING_VERSION_CONFIG_H));
    lcd_lib_draw_string_centerP(40, PSTR(STRING_CONFIG_H_AUTHOR));
    lcd_lib_update_screen();
	LED_FLASH();

}

void lcd_menu_advanced_stats()
{
    lcd_info_screen(previousMenu, NULL, PSTR("Return"));
    lcd_lib_draw_string_centerP(10, PSTR("Machine on for:"));
    char buffer[16];
    char* c = int_to_string(lifetime_minutes / 60, buffer, PSTR(":"));
    if (lifetime_minutes % 60 < 10)
        *c++ = '0';
    c = int_to_string(lifetime_minutes % 60, c);
    lcd_lib_draw_string_center(20, buffer);

    lcd_lib_draw_string_centerP(30, PSTR("Printing:"));
    c = int_to_string(lifetime_print_minutes / 60, buffer, PSTR(":"));
    if (lifetime_print_minutes % 60 < 10)
        *c++ = '0';
    c = int_to_string(lifetime_print_minutes % 60, c);
    *c++ = ' ';
    *c++ = ' ';
    c = int_to_string(lifetime_print_centimeters / 100, c, PSTR("m"));
    lcd_lib_draw_string_center(40, buffer);
    lcd_lib_update_screen();
	LED_NORMAL();

}
static void doMaterialsReset()
{
	lcd_material_reset_defaults();
	lcd_lib_beep_ext(500,200);
	lcd_lib_beep_ext(700,350);
	lcd_material_read_current_material();
	 lcd_change_to_menu(lcd_menu_maintenance_advanced);
}

static void doFactoryReset()
{
	lcd_lib_beep_ext(50,200);
    //Clear the EEPROM settings so they get read from default.
    eeprom_write_byte((uint8_t*)100, 0);
    eeprom_write_byte((uint8_t*)101, 0);
    eeprom_write_byte((uint8_t*)102, 0);
    eeprom_write_byte((uint8_t*)EEPROM_FIRST_RUN_DONE_OFFSET, 0);
    eeprom_write_byte(EEPROM_MATERIAL_COUNT_OFFSET(), 0);
    
    cli();
    //NOTE: Jumping to address 0 is not a fully proper way to reset.
    // Letting the watchdog timeout is a better reset, but the bootloader does not continue on a watchdog timeout.
    // So we disable interrupts and hope for the best!
    //Jump to address 0x0000
#ifdef __AVR__
    asm volatile(
            "clr	r30		\n\t"
            "clr	r31		\n\t"
            "ijmp	\n\t"
            );
#else
    //TODO
#endif
}

static void lcd_menu_advanced_factory_reset()
{
    lcd_question_screen(NULL, doFactoryReset, PSTR("YES"), previousMenu, NULL, PSTR("NO"));
	lcd_lib_beep_ext(110,500);
	lcd_lib_beep_ext(50,500);
    lcd_lib_draw_string_centerP(10, PSTR("Reset everything"));
    lcd_lib_draw_string_centerP(20, PSTR("to default?"));
    lcd_lib_update_screen();
}

static void lcd_menu_advanced_materials_reset()
	{
	lcd_question_screen(NULL, doMaterialsReset, PSTR("YES"), previousMenu, NULL, PSTR("NO"));
	lcd_lib_draw_string_centerP(10, PSTR("Reset materials"));
	lcd_lib_draw_string_centerP(20, PSTR("to default?"));
	lcd_lib_update_screen();
	}

static char* lcd_retraction_item(uint8_t nr)
{
    if (nr == 0)
        strcpy_P(card.longFilename, PSTR("< RETURN"));
    else if (nr == 1)
        strcpy_P(card.longFilename, PSTR("Retract length"));
    else if (nr == 2)
        strcpy_P(card.longFilename, PSTR("Retract speed"));
    else
        strcpy_P(card.longFilename, PSTR("???"));
    return card.longFilename;
}

static void lcd_retraction_details(uint8_t nr)
{
	lcd_lib_enable_encoder_acceleration(true);
    char buffer[16];
    if (nr == 0)
        return;
    else if(nr == 1)
        float_to_string(retract_length, buffer, PSTR("mm"));
    else if(nr == 2)
        int_to_string(retract_feedrate / 60 + 0.5, buffer, PSTR("mm" PER_SECOND_SYMBOL ));
    lcd_lib_draw_string(5, 53, buffer);
	LED_NORMAL();

}

static void lcd_menu_maintenance_retraction()
{
LED_NORMAL();

    lcd_scroll_menu(PSTR("RETRACTION"), 3, lcd_retraction_item, lcd_retraction_details);
    if (lcd_lib_button_pressed)
    {
        if (IS_SELECTED_SCROLL(0))
        {
            Config_StoreSettings();
            lcd_change_to_menu(lcd_menu_maintenance_advanced, SCROLL_MENU_ITEM_POS(6 + EXTRUDERS * 2));
        }
        else if (IS_SELECTED_SCROLL(1))
            LCD_EDIT_SETTING_FLOAT001(retract_length, "Retract length", "mm", 0, 50);
        else if (IS_SELECTED_SCROLL(2))
            LCD_EDIT_SETTING_SPEED(retract_feedrate, "Retract speed", "mm" PER_SECOND_SYMBOL , 0, max_feedrate[E_AXIS] * 60);
    }
}

static char* lcd_motion_item(uint8_t nr)
{
    if (nr == 0)
        strcpy_P(card.longFilename, PSTR("< RETURN"));
    else if (nr == 1)
        strcpy_P(card.longFilename, PSTR("Acceleration"));
    else if (nr == 2)
        strcpy_P(card.longFilename, PSTR("X/Y Jerk"));
    else if (nr == 3)
        strcpy_P(card.longFilename, PSTR("Max speed X"));
    else if (nr == 4)
        strcpy_P(card.longFilename, PSTR("Max speed Y"));
    else if (nr == 5)
        strcpy_P(card.longFilename, PSTR("Max speed Z"));
    else
        strcpy_P(card.longFilename, PSTR("???"));
    return card.longFilename;
}

static void lcd_motion_details(uint8_t nr)
{
    char buffer[16];
    if (nr == 0)
        return;
    else if(nr == 1)
        int_to_string(acceleration, buffer, PSTR("mm" PER_SECOND_SYMBOL "" SQUARED_SYMBOL ));
    else if(nr == 2)
        int_to_string(max_xy_jerk, buffer, PSTR("mm" PER_SECOND_SYMBOL ));
    else if(nr == 3)
        int_to_string(max_feedrate[X_AXIS], buffer, PSTR("mm" PER_SECOND_SYMBOL ));
    else if(nr == 4)
        int_to_string(max_feedrate[Y_AXIS], buffer, PSTR("mm" PER_SECOND_SYMBOL ));
    else if(nr == 5)
        int_to_string(max_feedrate[Z_AXIS], buffer, PSTR("mm" PER_SECOND_SYMBOL ));
    lcd_lib_draw_string(5, 53, buffer);
}

static void lcd_menu_maintenance_motion()
{
    lcd_scroll_menu(PSTR("MOTION"), 6, lcd_motion_item, lcd_motion_details);
	LED_NORMAL();

    if (lcd_lib_button_pressed)
    {
        if (IS_SELECTED_SCROLL(0))
        {
            Config_StoreSettings();
            lcd_change_to_menu(lcd_menu_maintenance_advanced, SCROLL_MENU_ITEM_POS(7));
        }
        else if (IS_SELECTED_SCROLL(1))
            LCD_EDIT_SETTING_FLOAT100(acceleration, "Acceleration", "mm" PER_SECOND_SYMBOL  SQUARED_SYMBOL , 0, 20000);
        else if (IS_SELECTED_SCROLL(2))
            LCD_EDIT_SETTING_FLOAT1(max_xy_jerk, "X/Y Jerk", "mm" PER_SECOND_SYMBOL , 0, 100);
        else if (IS_SELECTED_SCROLL(3))
            LCD_EDIT_SETTING_FLOAT1(max_feedrate[X_AXIS], "Max speed X", "mm" PER_SECOND_SYMBOL , 0, 1000);
        else if (IS_SELECTED_SCROLL(4))
            LCD_EDIT_SETTING_FLOAT1(max_feedrate[Y_AXIS], "Max speed Y", "mm" PER_SECOND_SYMBOL , 0, 1000);
        else if (IS_SELECTED_SCROLL(5))
            LCD_EDIT_SETTING_FLOAT1(max_feedrate[Z_AXIS], "Max speed Z", "mm" PER_SECOND_SYMBOL , 0, 1000);
    }
}

static char* lcd_led_item(uint8_t nr)
{
    if (nr == 0)
        strcpy_P(card.longFilename, PSTR("< RETURN"));
    else if (nr == 1)
        strcpy_P(card.longFilename, PSTR("Brightness"));
    else if (nr == 2)
        strcpy_P(card.longFilename, PSTR(" Always On"));
    else if (nr == 3)
        strcpy_P(card.longFilename, PSTR(" Always Off"));
    else if (nr == 4)
        strcpy_P(card.longFilename, PSTR(" On while printing"));
    else if (nr == 5)
        strcpy_P(card.longFilename, PSTR(" Glow when done"));
    else
        strcpy_P(card.longFilename, PSTR("???"));
    if (nr - 2 == led_mode)
        card.longFilename[0] = '>';
    return card.longFilename;
}

static void lcd_led_details(uint8_t nr)
{
	analogWrite(LED_PIN, 255 * int(led_brightness_level) / 100);
    char buffer[16];
    if (nr == 0)
        return;
    else if(nr == 1)
    {
		lcd_lib_enable_encoder_acceleration(true);
        int_to_string(led_brightness_level, buffer, PSTR("%"));
        lcd_lib_draw_string(5, 53, buffer);
    }
	
}

static void lcd_menu_maintenance_led()
{
LED_NORMAL();

    analogWrite(LED_PIN, 255 * int(led_brightness_level) / 100);
    lcd_scroll_menu(PSTR("LED"), 6, lcd_led_item, lcd_led_details);
    if (lcd_lib_button_pressed)
    {
        if (IS_SELECTED_SCROLL(0))
        {
            if (led_mode != LED_MODE_ALWAYS_ON)
                analogWrite(LED_PIN, 0);
            Config_StoreSettings();
            lcd_change_to_menu(lcd_menu_maintenance_advanced, SCROLL_MENU_ITEM_POS(1));
        }
        else if (IS_SELECTED_SCROLL(1))
            LCD_EDIT_SETTING(led_brightness_level, "Brightness", "%", 0, 100);
        else if (IS_SELECTED_SCROLL(2))
            led_mode = LED_MODE_ALWAYS_ON;
        else if (IS_SELECTED_SCROLL(3))
            led_mode = LED_MODE_ALWAYS_OFF;
        else if (IS_SELECTED_SCROLL(4))
            led_mode = LED_MODE_WHILE_PRINTING;
        else if (IS_SELECTED_SCROLL(5))
            led_mode = LED_MODE_BLINK_ON_DONE;
    }
}

#endif//ENABLE_ULTILCD2
