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
#include "gcode.h"
#include "MenuUseful.h"
#include "stringHelpers.h"


#define MAINTENANCE_FUNCTION_TIMEOUT 0 

//void lcd_menu_maintenance_advanced_heatup();
void lcd_menu_maintenance_advanced_bed_heatup();
void lcd_menu_maintenance_led();
void lcd_menu_maintenance_extrude();
//void lcd_menu_maintenance_retraction();
void lcd_menu_advanced_version();
void lcd_menu_advanced_stats();
void lcd_menu_maintenance_motion();
void lcd_menu_advanced_factory_reset();
void lcd_menu_advanced_materials_reset();
void  doMaterialsReset();
extern bool allow_encoder_acceleration;
float extruded_amount;

extern unsigned char  LED_DIM_TIME;
// byte adjust_axis = 0;




enum MAINTENANCE_MENU
{
    MAINTENANCE_MENU_RETURN=0,
    MAINTENANCE_MENU_HOME_HEAD,
    MAINTENANCE_MENU_CENTER_HEAD,
	MAINTENANCE_MENU_LIMITS_HEAD,
    MAINTENANCE_MENU_RELAX_MOTORS,
    MAINTENANCE_MENU_HEAT_EXTR,
#if EXTRUDERS > 1
    MAINTENANCE_MENU_HEAT_EXTR2,
#endif
#if EXTRUDERS > 2
    MAINTENANCE_MENU_HEAT_EXTR3,
#endif
    MAINTENANCE_MENU_EXTRUDE,
#if EXTRUDERS > 1
    MAINTENANCE_MENU_EXTRUDE1,
#endif
#if EXTRUDERS > 2
    MAINTENANCE_MENU_EXTRUDE2,
#endif
    MAINTENANCE_MENU_HEAT_BED,
    MAINTENANCE_MENU_ADJ_BED,
    MAINTENANCE_MENU_LOWER_BED,
    MAINTENANCE_MENU_RAISE_BED,
    MAINTENANCE_MENU_RETRACTION,
    MAINTENANCE_MENU_ZLIFT,
    MAINTENANCE_MENU_MOVEMENT,

    MAINTENANCE_MENU_XMAX,
    MAINTENANCE_MENU_YMAX,
    MAINTENANCE_MENU_HEAD_FAN,
    MAINTENANCE_MENU_COOL_FAN,
    MAINTENANCE_MENU_CASE_FAN,

    MAINTENANCE_MENU_IDLE_TIMEOUT,
    MAINTENANCE_MENU_LED,
    MAINTENANCE_MENU_WRAP,
    MAINTENANCE_MENU_SHOW_VERSION,
    MAINTENANCE_MENU_SHOW_STATS,
    MAINTENANCE_MENU_RESET_MATERIALS,
    MAINTENANCE_MENU_FACTORY_RESET,
    MAINTENANCE_MENU_MAX
};

//#define DEBUG_INFO


static char* lcd_menu_maintenance_getString(uint8_t nr)
{
#ifdef DEBUG_INFO
    SERIAL_ECHO_START ;
    SERIAL_ECHOPAIR(("MENU "),(unsigned long) nr);


#endif

    char* c = lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring	;


    //  char * c = menu_temporary_string_buffer;
    switch (nr)
        {
            case MAINTENANCE_MENU_RETURN:
                strcpy_P(c, PSTR("< RETURN"));
                break;
            case MAINTENANCE_MENU_ADJ_BED:
                strcpy_P(c, PSTR("Adjust Bed Height"));
                break;

            case MAINTENANCE_MENU_LED:
                strcpy_P(c, PSTR("LED settings"));
                break;
            case MAINTENANCE_MENU_HEAT_EXTR:
#if EXTRUDERS > 1
            case MAINTENANCE_MENU_HEAT_EXTR1:
#if EXTRUDERS > 2
            case MAINTENANCE_MENU_HEAT_EXTR2:
#endif
#endif
                strcpy_P(c, PSTR("Heatup extr "));

#if EXTRUDERS > 1
                c += strlen (c);
                c = int_to_string(nr -MAINTENANCE_MENU_HEAT_EXTR + 1, c );
                *c++=0;
#endif

                break;
            case MAINTENANCE_MENU_HEAT_BED:
                strcpy_P(c, PSTR("Heatup buildplate"));
                break;
            case MAINTENANCE_MENU_HOME_HEAD:
                strcpy_P(c, PSTR("Home head"));
                break;

            case MAINTENANCE_MENU_CENTER_HEAD:
                strcpy_P(c, PSTR("Center head"));
                break;
            case MAINTENANCE_MENU_RELAX_MOTORS:
                strcpy_P(c, PSTR("Release motors"));
                break;
            case MAINTENANCE_MENU_LOWER_BED:
                strcpy_P(c, PSTR("Lower buildplate"));
                break;
            case MAINTENANCE_MENU_RAISE_BED:
                strcpy_P(c, PSTR("Raise buildplate"));
                break;
#if EXTRUDERS > 1
            case MAINTENANCE_MENU_EXTRUDE1:
#if EXTRUDERS > 2
            case MAINTENANCE_MENU_EXTRUDE2:
#endif
#endif
            case MAINTENANCE_MENU_EXTRUDE:
                c =  strcpy_P(c, PSTR("Extrude material "));

#if EXTRUDERS > 1
                c += strlen (c);
                c = int_to_string(nr -MAINTENANCE_MENU_EXTRUDE + 1, c );
                *c++=0;
#endif
                break;
            case MAINTENANCE_MENU_RETRACTION:
                strcpy_P(c, PSTR("Retraction settings"));
                break;
            case MAINTENANCE_MENU_MOVEMENT:
                strcpy_P(c, PSTR("Motion settings"));
                break;
            case MAINTENANCE_MENU_SHOW_VERSION:
                strcpy_P(c, PSTR("Version"));
                break;
            case MAINTENANCE_MENU_SHOW_STATS:
                strcpy_P(c, PSTR("Runtime stats"));
                break;
            case MAINTENANCE_MENU_FACTORY_RESET:
                strcpy_P(c, PSTR("Factory reset"));
                break;
            case MAINTENANCE_MENU_RESET_MATERIALS:
                strcpy_P(c, PSTR("Reset Materials"));
                break;
            case MAINTENANCE_MENU_ZLIFT:
                strcpy_P(c, PSTR("Adjust Z Lift"));
                break;
            case MAINTENANCE_MENU_IDLE_TIMEOUT:
                strcpy_P(c, PSTR("Adjust idle timeout"));
                break;
            case MAINTENANCE_MENU_WRAP:
                if (NOWRAP_MENUS)
                    strcpy_P(c, PSTR("Enable Menu wrap"));
                else
                    strcpy_P(c, PSTR("Disable Menu wrap"));
                break;

            default:
                strcpy_P(c, PSTR("???"));
                break;
            case MAINTENANCE_MENU_XMAX:
                strcpy_P(c, PSTR("Adjust X Maximum"));
                break;
            case MAINTENANCE_MENU_YMAX:
                strcpy_P(c, PSTR("Adjust Y Maximum"));
                break;
            case MAINTENANCE_MENU_HEAD_FAN:
                strcpy_P(c, PSTR("Turn on head fan"));
                break;
            case MAINTENANCE_MENU_CASE_FAN:
                strcpy_P(c, PSTR("Turn on case fan"));
                break;
            case MAINTENANCE_MENU_COOL_FAN:
                strcpy_P(c, PSTR("Turn on cooling fan"));
                break;
			case MAINTENANCE_MENU_LIMITS_HEAD:
				strcpy_P(c, PSTR("Run head through XY limits"));
				break;

        }
    return c;
}

void lcd_menu_maintenance_getDetails(uint8_t nr)
{
//    char* c = (char*)lcd_cache;
    char temporary_string_buffer[24];
    memset (temporary_string_buffer,0,sizeof(temporary_string_buffer));

    char  * c = temporary_string_buffer;

    switch (nr)
        {
            case MAINTENANCE_MENU_SHOW_VERSION:
                strcpy_P (c,PSTR(STRING_VERSION_CONFIG_H));
                break;
            case MAINTENANCE_MENU_IDLE_TIMEOUT:
                c= int_to_string((int) LED_DIM_TIME, c, PSTR(" minutes"));
                *c++=0;
                break;
            case MAINTENANCE_MENU_ZLIFT:
                c= float_to_string(retract_zlift, c, PSTR("mm"));
                *c++=0;
                break;
            case MAINTENANCE_MENU_RETRACTION:
                c= float_to_string1(retract_length, c, PSTR(" @ "));
                c= int_to_string((int) retract_feedrate/60, c, PSTR("mm" PER_SECOND_SYMBOL "="));
                c= float_to_string1(retract_length / (retract_feedrate/60), c, PSTR("s"));
                *c++=0;
                break;
#if EXTRUDERS > 2
            case MAINTENANCE_MENU_HEAT_EXTR2:
#endif
#if EXTRUDERS > 1
            case MAINTENANCE_MENU_HEAT_EXTR1:
#endif
            case MAINTENANCE_MENU_HEAT_EXTR:
                c = int_to_string(current_temperature[nr-MAINTENANCE_MENU_HEAT_EXTR], c /*,PSTR( DEGREE_C_SYMBOL )*/);
                *c++ = TEMPERATURE_SEPARATOR;
                c = int_to_string(target_temperature[nr-MAINTENANCE_MENU_HEAT_EXTR], c, PSTR( DEGREE_C_SYMBOL ));
                *c++=0;
                break;
            case MAINTENANCE_MENU_HEAT_BED:
                c = int_to_string(current_temperature_bed, c/*, PSTR( DEGREE_C_SYMBOL )*/);
                *c++ = TEMPERATURE_SEPARATOR;
                c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ));
                *c++=0;
                break;


            case MAINTENANCE_MENU_XMAX:
                c = float_to_string(max_pos[X_AXIS],c, PSTR("mm"));
                break;
            case MAINTENANCE_MENU_YMAX:
                c = float_to_string(max_pos[Y_AXIS],c, PSTR("mm"));
                break;
            default: return;
        }
    lcd_lib_draw_string(5, 53, temporary_string_buffer);
}

void lcd_menu_maintenance_doAction()
{
    lcd_scroll_menu(PSTR("SYSTEM"),MAINTENANCE_MENU_MAX, lcd_menu_maintenance_getString, lcd_menu_maintenance_getDetails);
    LED_NORMAL();
    if (millis() - last_user_interaction > MENU_TIMEOUT) lcd_menu_go_back();

    if (!lcd_lib_button_pressed()) return;
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("MAIN MENU BUTTON");

    byte index =0;
    switch (SELECTED_SCROLL_MENU_ITEM())
        {
            default:
                SERIAL_ECHO_START;
                SERIAL_ECHOLNPGM("UNKNOWN MENU ITEM");
                SERIAL_ECHOLN((unsigned long) (SELECTED_SCROLL_MENU_ITEM()));
                break;

            case MAINTENANCE_MENU_COOL_FAN:
                fanSpeedOverride = 255;
				analogWrite(FAN_PIN,fanSpeedOverride);
                break;
            case MAINTENANCE_MENU_CASE_FAN:
                WRITE (MOTHERBOARD_FAN,1);
				mobo_cooling_fan_timer.pause();
                break;
            case MAINTENANCE_MENU_HEAD_FAN:
				head_cooling_fan_timer.pause();
                setExtruderAutoFanState(EXTRUDER_0_AUTO_FAN_PIN, 1);
                break;
            case MAINTENANCE_MENU_RETURN:
                Config_StoreSettings();
                fanSpeedOverride = -1;
				head_cooling_fan_timer.resume();
				mobo_cooling_fan_timer.resume();
                checkExtruderAutoFans();
				controllerFan();
                lcd_menu_go_back();
                return;
                //lcd_change_to_menu(lcd_menu_main);
                break;
            case MAINTENANCE_MENU_ADJ_BED:
                lcd_change_to_menu ( lcd_menu_first_run_start_bed_leveling) ;
                break;

            case MAINTENANCE_MENU_LED:
                lcd_change_to_menu(lcd_menu_maintenance_led, 0);
                break;
#if EXTRUDERS > 1
            case MAINTENANCE_MENU_HEAT_EXTR1:
                index++;
#if EXTRUDERS > 2
            case MAINTENANCE_MENU_HEAT_EXTR2:
                index++;
#endif
#endif
            case MAINTENANCE_MENU_HEAT_EXTR:
                clearHistory();
                nozzle_adjust_id = index;
                if (target_temperature[nozzle_adjust_id] ==0) target_temperature[nozzle_adjust_id] = material[nozzle_adjust_id].temperature;
                lcd_change_to_menu(lcd_menu_print_tune_heatup_nozzle, 0);
                break;
            case MAINTENANCE_MENU_HEAT_BED:
                clearHistory();
                if (target_temperature_bed ==0) target_temperature_bed = material[active_extruder].bed_temperature;
                lcd_change_to_menu(lcd_menu_maintenance_advanced_bed_heatup, 0);
                break;
            case MAINTENANCE_MENU_HOME_HEAD:
                {
                    lcd_lib_beep();
                    enquecommand_P(PSTR("G28 X0 Y0"));
                }
                break;
            case MAINTENANCE_MENU_RELAX_MOTORS:
                {
                    lcd_lib_beep();
                    enquecommand_P(PSTR("M84"));
                }
                break;
            case MAINTENANCE_MENU_CENTER_HEAD:
                {
                    lcd_lib_beep();
                    enquecommand_P(PSTR("G1 X100 Y20 F35000"));
                }
                break;
			case MAINTENANCE_MENU_LIMITS_HEAD:
				{
				lcd_lib_beep();
				enquecommand_P(PSTR("G28 X0 Y0"));
				enquecommand_P(PSTR("G1 X0 Y0 F35000"));
				char buffer[20];
				memset (buffer,0,sizeof(buffer));
				sprintf_P(buffer, PSTR("G1 Y%i"), int(max_pos[Y_AXIS]));
				enquecommand(buffer);
				sprintf_P(buffer, PSTR("G1 X%i"), int(max_pos[X_AXIS]));
				enquecommand(buffer);
				enquecommand_P(PSTR("G1 Y0 F35000"));
				enquecommand_P(PSTR("G28 X0 Y0"));

				}
				break;

            case MAINTENANCE_MENU_LOWER_BED:
                {
                    lcd_lib_beep();
                    enquecommand_P(PSTR("G28 Z0"));
                }
                break;
            case MAINTENANCE_MENU_RAISE_BED:
                {
                    lcd_lib_beep();
                    enquecommand_P(PSTR("G28 Z0"));
                    enquecommand_P(PSTR("G1 Z60"));
                }
                break;
#if EXTRUDERS > 1
            case MAINTENANCE_MENU_EXTRUDE1
                    index++;
#if EXTRUDERS > 2
            case MAINTENANCE_MENU_EXTRUDE2:
                index++;
#endif
#endif
            case MAINTENANCE_MENU_EXTRUDE:
                {
                    clearHistory();
                    set_extrude_min_temp(0);
                    active_extruder =index;
                    target_temperature[active_extruder] = material[active_extruder].temperature;
                    extruded_amount=0;
                    lcd_change_to_menu(lcd_menu_maintenance_extrude, 0);
                }
                break;

            case MAINTENANCE_MENU_RETRACTION:
                lcd_change_to_menu(lcd_menu_retraction_doAction, SCROLL_MENU_ITEM_POS(0));
                break;

            case MAINTENANCE_MENU_MOVEMENT:
                lcd_change_to_menu(lcd_menu_maintenance_motion, SCROLL_MENU_ITEM_POS(0));
                break;
            case MAINTENANCE_MENU_SHOW_VERSION:
                lcd_change_to_menu(lcd_menu_advanced_version, SCROLL_MENU_ITEM_POS(0));
                break;

            case MAINTENANCE_MENU_SHOW_STATS:
                lcd_change_to_menu(lcd_menu_advanced_stats, SCROLL_MENU_ITEM_POS(0)); break;

            case MAINTENANCE_MENU_FACTORY_RESET:
                lcd_change_to_menu(lcd_menu_advanced_factory_reset, SCROLL_MENU_ITEM_POS(1));
                break;
            case MAINTENANCE_MENU_RESET_MATERIALS:
                lcd_change_to_menu(lcd_menu_advanced_materials_reset, SCROLL_MENU_ITEM_POS(1));
                break;

            case MAINTENANCE_MENU_IDLE_TIMEOUT:
                LCD_EDIT_SETTING(LED_DIM_TIME, "Idle timeout", " minutes", 0, 250);
                break;
            case MAINTENANCE_MENU_ZLIFT:
                LCD_EDIT_SETTING_FLOAT001(retract_zlift, "Retract ZLift", "mm", 0, 5);
                break;
            case MAINTENANCE_MENU_WRAP:
                NOWRAP_MENUS = !NOWRAP_MENUS;
                lcd_lib_beep();
                break;
            case MAINTENANCE_MENU_XMAX:
                {
                    max_pos[X_AXIS] = BASELINE_XLIMIT;
                    X_MAX_LENGTH = max_pos[0] - X_MIN_POS;
                    enquecommand_P(PSTR("G28 X0 Y0"));
                    char buffer[20];
                    memset (buffer,0,sizeof(buffer));
                    sprintf_P(buffer, PSTR("G1 F%i X%i Y%i"), int(homing_feedrate[0]), int(max_pos[X_AXIS]-MAX_ADJ_OFFSET), int(max_pos[Y_AXIS]/2) );
                    enquecommand(buffer);
                    lcd_change_to_menu(lcd_menu_maintenance_adjust_max_X, 0);
                }
                break;
            case MAINTENANCE_MENU_YMAX:
                {
                    max_pos[Y_AXIS] = BASELINE_YLIMIT;
                    Y_MAX_LENGTH = max_pos[1] - Y_MIN_POS;
                    enquecommand_P(PSTR("G28 X0 Y0"));
                    char buffer[20];
                    memset (buffer,0,sizeof(buffer));
                    sprintf_P(buffer, PSTR("G1 F%i X%i Y%i"), int(homing_feedrate[0]), int(max_pos[X_AXIS]/2), int(MAX_ADJ_OFFSET) );
                    enquecommand(buffer);
                    lcd_change_to_menu(lcd_menu_maintenance_adjust_max_Y, 0);
                }
                break;


        }
}
//-----------------------------------------------------------------------------------------------------------------


void lcd_menu_maintenance_adjust_max_Y()
{
    if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
        {
            if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 3)
                {
                    max_pos[1] += lcd_lib_encoder_pos * 0.1;
                    current_position[1]  -= lcd_lib_encoder_pos * 0.1;
                    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 10, active_extruder);
                    lcd_lib_encoder_pos = 0;
                }
        }
	// do we want a timeout here?
#if MAINTENANCE_FUNCTION_TIMEOUT
	if (millis() - last_user_interaction > MENU_TIMEOUT) {  enquecommand_P(PSTR("G28 X0 Y0"));  lcd_menu_go_back(); }
#endif 

    if (lcd_lib_button_pressed())		// exit
        {
            Y_MAX_LENGTH = max_pos[1] - Y_MIN_POS;
            //	base_home_pos[1] = max_pos[1];
            Config_StoreSettings();
            enquecommand_P(PSTR("G28 X0 Y0"));
            lcd_menu_go_back();
        }
    lcd_lib_enable_encoder_acceleration(false);
    lcd_lib_clear();
    lcd_lib_draw_string_centerP(ROW2, PSTR ("Move head with knob"));
    lcd_lib_draw_string_centerP(ROW3, PSTR ("to Y limit"));

    LED_GLOW();
    char buffer[10];
    memset (buffer,0,sizeof(buffer));
    float_to_string( max_pos[1],buffer,PSTR(" mm") );
    lcd_lib_draw_string_center(ROW5, buffer);
    lcd_lib_draw_string_centerP(ROW7, PSTR("Click to return"));
    lcd_lib_update_screen();
}

void lcd_menu_maintenance_adjust_max_X()
{
    if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
        {
            if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 3)
                {
                    max_pos[0] += lcd_lib_encoder_pos * 0.1;
                    current_position[0]  = max_pos[0];
                    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 10, active_extruder);
                    lcd_lib_encoder_pos = 0;
                }
        }

	// do we want a timeout here?
#if MAINTENANCE_FUNCTION_TIMEOUT
	if (millis() - last_user_interaction > MENU_TIMEOUT) {  enquecommand_P(PSTR("G28 X0 Y0"));  lcd_menu_go_back(); }
#endif 

    if (lcd_lib_button_pressed())		// exit
        {
            X_MAX_LENGTH = max_pos[0] - X_MIN_POS;
            Config_StoreSettings();
            enquecommand_P(PSTR("G28 X0 Y0"));
            lcd_menu_go_back();
        }
    lcd_lib_enable_encoder_acceleration(false);
    lcd_lib_clear();
    lcd_lib_draw_string_centerP(ROW2, PSTR ("Move head with knob"));
    lcd_lib_draw_string_centerP(ROW3, PSTR ("to X limit"));


    LED_GLOW();
    char buffer[10];
    memset (buffer,0,sizeof(buffer));
    float_to_string( max_pos[0],buffer,PSTR(" mm") );
    lcd_lib_draw_string_center(ROW5, buffer);
    lcd_lib_draw_string_centerP(ROW7, PSTR("Click to return"));
    lcd_lib_update_screen();
}



void lcd_menu_maintenance_extrude()
{
    run_history = true;

    if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
        {
            if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 3)
                {
                    extruded_amount+= (lcd_lib_encoder_pos * 0.1) * axis_steps_per_unit[E_AXIS] / 1000.0;
                    current_position[E_AXIS] += lcd_lib_encoder_pos * 0.1;
                    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 10, active_extruder);
                    lcd_lib_encoder_pos = 0;
                }
        }
// do we want a timeout here?
#if MAINTENANCE_FUNCTION_TIMEOUT
    if (millis() - last_user_interaction > MENU_TIMEOUT) {   lcd_menu_go_back(); return }
#endif

    if (lcd_lib_button_pressed())		// exit
        {
            set_extrude_min_temp(EXTRUDE_MINTEMP);
            target_temperature[active_extruder] = 0;
            lcd_menu_go_back();
            return;
        }
    lcd_lib_enable_encoder_acceleration(false);
    lcd_lib_clear();
    LED_GLOW();
    char buffer[20];
    memset (buffer,0,sizeof(buffer));
    float_to_string1(extruded_amount,buffer,PSTR(" mm") );

    lcd_lib_draw_string_center(ROW2, buffer);
    lcd_lib_draw_string_centerP(ROW7, PSTR("Click to return"));
    int_to_string(int(current_temperature[active_extruder]), buffer, PSTR( TEMPERATURE_SEPARATOR_S));
    int_to_string(int(target_temperature[active_extruder]), buffer+strlen(buffer), PSTR( DEGREE_C_SYMBOL ));
    lcd_lib_draw_string_center(ROW3, buffer);

    drawTempHistory (DISPLAY_RIGHT/3,ROW4+2,2*DISPLAY_RIGHT/3,ROW7-3,lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.temp_history);

    lcd_lib_update_screen();
}

void lcd_menu_maintenance_advanced_bed_heatup()
{
    run_history = true;

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
    // do we want a timeout here?
#if MAINTENANCE_FUNCTION_TIMEOUT
    if (millis() - last_user_interaction > MENU_TIMEOUT) {   lcd_menu_go_back(); return }
#endif
    if (lcd_lib_button_pressed())
        {
            lcd_menu_go_back();
            return;
        }
    lcd_lib_enable_encoder_acceleration(true);
    lcd_lib_clear();
    lcd_lib_draw_string_centerP(ROW2, PSTR("Bed temperature"));
    lcd_lib_draw_string_centerP(ROW7, PSTR("Click to return"));
    char buffer[20];
    memset (buffer,0,sizeof(buffer));
    char *c = buffer;
    c=int_to_string(int(current_temperature_bed), c, PSTR( TEMPERATURE_SEPARATOR_S));
    c= int_to_string(int(target_temperature_bed),c, PSTR( DEGREE_C_SYMBOL ));
    *c++=0;
    lcd_lib_draw_string_center(ROW3, buffer);
    drawTempHistory (DISPLAY_RIGHT/3,ROW4+2,2*DISPLAY_RIGHT/3,ROW7-3,lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.temp_history);


    lcd_lib_update_screen();
}

void lcd_menu_advanced_version()
{
    lcd_info_screen(NULL, NULL, PSTR("Return"));
    lcd_lib_draw_string_centerP(10, PSTR(STRING_VERSION_CONFIG_H));
    lcd_lib_draw_string_centerP(30, PSTR(STRING_CONFIG_H_AUTHOR));
    lcd_lib_update_screen();
    LED_FLASH();

}

void lcd_menu_advanced_stats()
{
    lcd_info_screen(NULL, NULL, PSTR("Return"));
    lcd_lib_draw_string_centerP(10, PSTR("Machine on for:"));
    char buffer[20];
    memset (buffer,0,sizeof(buffer));
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
void doMaterialsReset()
{
    lcd_material_reset_defaults();
    lcd_lib_beep_ext(500,200);
    lcd_lib_beep_ext(700,350);
    lcd_material_read_current_material();
    lcd_menu_go_back();
//    lcd_change_to_menu(lcd_menu_maintenance_doAction);
}

void doFactoryReset()
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

void lcd_menu_advanced_factory_reset()
{
    lcd_question_screen(NULL, doFactoryReset, PSTR("YES"), NULL, NULL, PSTR("NO"));
    lcd_lib_beep_ext(110,500);
    lcd_lib_beep_ext(50,500);
    lcd_lib_draw_string_centerP(10, PSTR("Reset everything"));
    lcd_lib_draw_string_centerP(20, PSTR("to default?"));
    lcd_lib_update_screen();
}

void lcd_menu_advanced_materials_reset()
{
    lcd_question_screen(NULL, doMaterialsReset, PSTR("YES"), NULL, NULL, PSTR("NO"));
    lcd_lib_draw_string_centerP(10, PSTR("Reset materials"));
    lcd_lib_draw_string_centerP(20, PSTR("to default?"));
    lcd_lib_update_screen();
}
#if 0

// static char* lcd_menu_retraction_getString(uint8_t nr)
// {
//     char* c = (char*)lcd_cache;
//     if (nr == 0)
//         strcpy_P(c, PSTR("< RETURN"));
//     else
//         if (nr == 1)
//             strcpy_P(c, PSTR("Retract length"));
//         else
//             if (nr == 2)
//                 strcpy_P(c, PSTR("Retract speed"));
//             else
//                 strcpy_P(c, PSTR("???"));
//     return c;
// }

// duplicated with lcd print
// void lcd_menu_retraction_getDetails(uint8_t nr)
// {
//     lcd_lib_enable_encoder_acceleration(true);
//     char buffer[16];
//     memset (buffer,0,sizeof(buffer));
//
//     if (nr == 0)
//         return;
//     else
//         if(nr == 1)
//             float_to_string(retract_length, buffer, PSTR("mm"));
//         else
//             if(nr == 2)
//                 int_to_string(retract_feedrate / 60 + 0.5, buffer, PSTR("mm" PER_SECOND_SYMBOL ));
//     lcd_lib_draw_string(5, 53, buffer);
//     LED_NORMAL();
//
// }

void lcd_menu_maintenance_retraction()
{
    LED_NORMAL();

    lcd_scroll_menu(PSTR("RETRACTION"), 3, lcd_menu_retraction_getString, lcd_menu_retraction_getDetails);
    if (lcd_lib_button_pressed())
        {
            if (IS_SELECTED_SCROLL(0))
                {
                    Config_StoreSettings();
                    lcd_change_to_menu(lcd_menu_maintenance_doAction, SCROLL_MENU_ITEM_POS(6 + EXTRUDERS * 2));
                }
            else
                if (IS_SELECTED_SCROLL(1))
                    LCD_EDIT_SETTING_FLOAT001(retract_length, "Retract length", "mm", 0, 50);
                else
                    if (IS_SELECTED_SCROLL(2))
                        LCD_EDIT_SETTING_SPEED(retract_feedrate, "Retract speed", "mm" PER_SECOND_SYMBOL , 0, max_feedrate[E_AXIS] * 60);
        }
}
#endif

static char* lcd_motion_item(uint8_t nr)
{
    char* c = lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring	;
    if (nr == 0)
        strcpy_P(c, PSTR("< RETURN"));
    else
        if (nr == 1)
            strcpy_P(c, PSTR("Acceleration"));
        else
            if (nr == 2)
                strcpy_P(c, PSTR("X/Y Jerk"));
            else
                if (nr == 3)
                    strcpy_P(c, PSTR("Max speed X"));
                else
                    if (nr == 4)
                        strcpy_P(c, PSTR("Max speed Y"));
                    else
                        if (nr == 5)
                            strcpy_P(c, PSTR("Max speed Z"));
                        else
                            strcpy_P(c, PSTR("???"));
    return c;
}

void lcd_motion_details(uint8_t nr)
{
    char buffer[16];
    memset (buffer,0,sizeof(buffer));

    if (nr == 0)
        return;
    else
        if(nr == 1)
            int_to_string(acceleration, buffer, PSTR("mm" PER_SECOND_SYMBOL "" SQUARED_SYMBOL ));
        else
            if(nr == 2)
                int_to_string(max_xy_jerk, buffer, PSTR("mm" PER_SECOND_SYMBOL ));
            else
                if(nr == 3)
                    int_to_string(max_feedrate[X_AXIS], buffer, PSTR("mm" PER_SECOND_SYMBOL ));
                else
                    if(nr == 4)
                        int_to_string(max_feedrate[Y_AXIS], buffer, PSTR("mm" PER_SECOND_SYMBOL ));
                    else
                        if(nr == 5)
                            int_to_string(max_feedrate[Z_AXIS], buffer, PSTR("mm" PER_SECOND_SYMBOL ));
    lcd_lib_draw_string(5, 53, buffer);
}

void lcd_menu_maintenance_motion()
{
    lcd_scroll_menu(PSTR("MOTION"), 6, lcd_motion_item, lcd_motion_details);
    LED_NORMAL();
    if (millis() - last_user_interaction > MENU_TIMEOUT) {   lcd_menu_go_back(); return; }
    if (lcd_lib_button_pressed())
        {
            if (IS_SELECTED_SCROLL(0))
                {
                    //                   Config_StoreSettings();
                    lcd_menu_go_back();
//                     lcd_change_to_menu(lcd_menu_maintenance_doAction, SCROLL_MENU_ITEM_POS(7));
                }
            else
                if (IS_SELECTED_SCROLL(1))
                    LCD_EDIT_SETTING_FLOAT100(acceleration, "Acceleration", "mm" PER_SECOND_SYMBOL  SQUARED_SYMBOL , 0, 20000);
                else
                    if (IS_SELECTED_SCROLL(2))
                        LCD_EDIT_SETTING_FLOAT1(max_xy_jerk, "X/Y Jerk", "mm" PER_SECOND_SYMBOL , 0, 100);
                    else
                        if (IS_SELECTED_SCROLL(3))
                            LCD_EDIT_SETTING_FLOAT1(max_feedrate[X_AXIS], "Max speed X", "mm" PER_SECOND_SYMBOL , 0, 1000);
                        else
                            if (IS_SELECTED_SCROLL(4))
                                LCD_EDIT_SETTING_FLOAT1(max_feedrate[Y_AXIS], "Max speed Y", "mm" PER_SECOND_SYMBOL , 0, 1000);
                            else
                                if (IS_SELECTED_SCROLL(5))
                                    LCD_EDIT_SETTING_FLOAT1(max_feedrate[Z_AXIS], "Max speed Z", "mm" PER_SECOND_SYMBOL , 0, 1000);
        }
}

static char* lcd_led_item(uint8_t nr)
{
    char* c = lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring	;
    if (nr == 0)
        strcpy_P(c, PSTR("< RETURN"));
    else
        if (nr == 1)
            strcpy_P(c, PSTR("Brightness"));
        else
            if (nr == 2)
                strcpy_P(c, PSTR(" Always On"));
            else
                if (nr == 3)
                    strcpy_P(c, PSTR(" Always Off"));
                else
                    if (nr == 4)
                        strcpy_P(c, PSTR(" On while printing"));
                    else
                        if (nr == 5)
                            strcpy_P(c, PSTR(" Glow when done"));
                        else
                            strcpy_P(c, PSTR("???"));
    if (nr - 2 == led_mode)
        c[0] = '>';
    return c;
}

void lcd_led_details(uint8_t nr)
{
    analogWrite(LED_PIN, 255 * int(led_brightness_level) / 100);

    if (nr == 0)
        return;
    else
        if(nr == 1)
            {
                char buffer[16];
                memset (buffer,0,sizeof(buffer));
                lcd_lib_enable_encoder_acceleration(true);
                int_to_string(led_brightness_level, buffer, PSTR("%"));
                lcd_lib_draw_string(5, 53, buffer);
            }

}

void lcd_menu_maintenance_led()
{
    if (millis() - last_user_interaction > MENU_TIMEOUT) {   lcd_menu_go_back(); return; }
    LED_NORMAL();
    analogWrite(LED_PIN, 255 * int(led_brightness_level) / 100);
    lcd_scroll_menu(PSTR("LED"), 6, lcd_led_item, lcd_led_details);
    if (lcd_lib_button_pressed())
        {
            if (IS_SELECTED_SCROLL(0))
                {
                    if (led_mode != LED_MODE_ALWAYS_ON)
                        analogWrite(LED_PIN, 0);
                    Config_StoreSettings();
                    lcd_menu_go_back();
                    return;
//                    lcd_change_to_menu(lcd_menu_maintenance_doAction, SCROLL_MENU_ITEM_POS(1));
                }
            else
                if (IS_SELECTED_SCROLL(1))
                    LCD_EDIT_SETTING(led_brightness_level, "Brightness", "%", 0, 100);
                else
                    if (IS_SELECTED_SCROLL(2))
                        led_mode = LED_MODE_ALWAYS_ON;
                    else
                        if (IS_SELECTED_SCROLL(3))
                            led_mode = LED_MODE_ALWAYS_OFF;
                        else
                            if (IS_SELECTED_SCROLL(4))
                                led_mode = LED_MODE_WHILE_PRINTING;
                            else
                                if (IS_SELECTED_SCROLL(5))
                                    led_mode = LED_MODE_BLINK_ON_DONE;
        }
}
//-----------------------------------------------------------------------------------------------------------------
void lcd_menu_draw_temp_adj_screen()
{
    run_history = true;

    lcd_lib_enable_encoder_acceleration(true);
    LED_HEAT();
    lcd_lib_clear();
    lcd_lib_draw_string_centerP(ROW2, PSTR("Nozzle temperature:"));
    lcd_lib_draw_string_centerP(ROW7, PSTR("Click to return"));
    char buffer[16];
    memset (buffer,0,sizeof(buffer));
    char *c = buffer;
    c=int_to_string(int(current_temperature[active_extruder]), c, PSTR( TEMPERATURE_SEPARATOR_S));
    c=int_to_string(int(target_temperature[active_extruder]), c, PSTR( DEGREE_C_SYMBOL ));
    *c++=0;
    lcd_lib_draw_string_center(ROW3, buffer);
    drawTempHistory (DISPLAY_RIGHT/3,ROW4+2,2*DISPLAY_RIGHT/3,ROW7-3,lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.temp_history);
    lcd_lib_update_screen();
}
#endif//ENABLE_ULTILCD2
