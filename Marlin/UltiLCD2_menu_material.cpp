#include <avr/pgmspace.h>

#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
#include "Marlin.h"
#include "cardreader.h"//This code uses thestring_buffer as buffer to store data, to save memory.
#include "temperature.h"
#include "UltiLCD2.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_menu_material.h"
#include "UltiLCD2_menu_print.h"
#include "gcode.h"
#include "MenuUseful.h"
#include "stringHelpers.h"

#ifndef eeprom_read_float
//Arduino IDE compatibility, lacks the eeprom_read_float function
float inline eeprom_read_float(float* addr)
{
    union { uint32_t i; float f; } n;
    n.i = eeprom_read_dword((uint32_t*)addr);
    return n.f;
}
void inline eeprom_write_float(float* addr, float f)
{
    union { uint32_t i; float f; } n;
    n.f = f;
    eeprom_write_dword((uint32_t*)addr, n.i);
}
#endif

const int MATERIAL_PRESETS=9;

char material_name_buf[MATERIAL_NAME_LENGTH+1];
char material_name[EXTRUDERS][MATERIAL_NAME_LENGTH+1];

struct material_preset
{
    char PROGMEM name[MATERIAL_NAME_LENGTH];		//  `12 chars max
    int temperature;
    int bed;
    byte fan_speed;
    int flow;
    float diameter;
};

PROGMEM const material_preset  presets[MATERIAL_PRESETS] =
{
    {(("GENERIC PLA")   )  , 215, 70, 100, 100, 2.85},
    {(("GENERIC ABS")   )  , 235, 95,  0, 107, 2.85},
    {(("IC3D ABS")      )  , 225, 95,  0, 107, 2.85},
    {(("JET ABS")       )  , 240, 95,  0, 107, 2.95},
    {(("ECOFLEX PLA")   )  , 235, 75, 100, 110, 2.85},
    {(("T 618 NYLON")   )  , 245, 45, 100, 120, 2.85},
    {(("TGLASE/PET+")   )  , 225, 70, 100, 120, 2.9 },
    {(("NINJAFLEX")     )  , 245, 55, 100, 135, 2.7 },
    {(("POLYCARB")      )  , 275,110, 100, 110, 2.85}
};

struct materialSettings material[EXTRUDERS];

void doCooldown();//TODO
static void lcd_menu_material_main();
static void lcd_menu_change_material_preheat();
static void lcd_menu_change_material_remove();
static void lcd_menu_change_material_remove_wait_user();
static void lcd_menu_change_material_remove_wait_user_ready();
static void lcd_menu_change_material_insert_wait_user();
static void lcd_menu_change_material_insert_wait_user_ready();
static void lcd_menu_change_material_insert_forward();
static void lcd_menu_change_material_insert();
static void lcd_menu_change_material_select_material();
static void lcd_menu_material_select();
static void lcd_menu_material_selected();
static void lcd_menu_material_settings();
static void lcd_menu_material_settings_store();

static void cancelMaterialInsert()
{
    digipot_current(2, motor_current_setting[2]);//Set E motor power to default.
    doCooldown();
    enquecommand_P(PSTR("G28 X0 Y0"));
}

void lcd_menu_material()
{
#if EXTRUDERS > 1
    lcd_tripple_menu(PSTR("PRIMARY|NOZZLE"), PSTR("SECONDARY|NOZZLE"), PSTR("RETURN"));

    if (lcd_lib_button_pressed)
        {
            if (IS_SELECTED_MAIN(0))
                {
                    active_extruder = 0;
                    lcd_change_to_menu(lcd_menu_material_main);
                }
            else
                if (IS_SELECTED_MAIN(1))
                    {
                        active_extruder = 1;
                        lcd_change_to_menu(lcd_menu_material_main);
                    }
                else
                    if (IS_SELECTED_MAIN(2))
                        lcd_change_to_menu(lcd_menu_main);
        }

    lcd_lib_update_screen();
#else
    currentMenu = lcd_menu_material_main;
#endif
}

static void lcd_menu_material_main()
{
    lcd_tripple_menu(PSTR("CHANGE"), PSTR("SETTINGS"), PSTR("RETURN"));

    if (lcd_lib_button_pressed)
        {
            if (IS_SELECTED_MAIN(0))
                {
                    minProgress = 0;
                    clearHistory();
                    lcd_change_to_menu(lcd_menu_change_material_preheat);
                }
            else
                if (IS_SELECTED_MAIN(1))
                    lcd_change_to_menu(lcd_menu_material_select, SCROLL_MENU_ITEM_POS(0));
                else
                    if (IS_SELECTED_MAIN(2))
                        lcd_change_to_menu(lcd_menu_main);
        }

    lcd_lib_update_screen();
}

static void lcd_menu_change_material_preheat()
{
    run_history = true;
    setTargetHotend(material[active_extruder].temperature, active_extruder);
    int16_t temp = degHotend(active_extruder) - 20;
    int16_t target = degTargetHotend(active_extruder) - 20 - 10;
    if (temp < 0) temp = 0;
    if (temp > target && !is_command_queued())
        {
            set_extrude_min_temp(0);
            for(uint8_t e=0; e<EXTRUDERS; e++)
                volume_to_filament_length[e] = 1.0;//Set the extrusion to 1mm per given value, so we can move the filament a set distance.

            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], 20.0, retract_feedrate/60.0, active_extruder);

            float old_max_feedrate_e = max_feedrate[E_AXIS];
            float old_retract_acceleration = retract_acceleration;
            max_feedrate[E_AXIS] = FILAMENT_REVERSAL_SPEED;
            retract_acceleration = FILAMENT_LONG_MOVE_ACCELERATION;

            current_position[E_AXIS] = 0;
            plan_set_e_position(current_position[E_AXIS]);
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], -1.0, FILAMENT_REVERSAL_SPEED, active_extruder);
            for(uint8_t n=0; n<6; n++)
                plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], (n+1)*-FILAMENT_REVERSAL_LENGTH/6, FILAMENT_REVERSAL_SPEED, active_extruder);

            max_feedrate[E_AXIS] = old_max_feedrate_e;
            retract_acceleration = old_retract_acceleration;

            currentMenu = lcd_menu_change_material_remove;
            temp = target;
        }

    uint8_t progress = uint8_t(temp * 125 / target);
    if (progress < minProgress)
        progress = minProgress;
    else
        minProgress = progress;

    lcd_info_screen(lcd_menu_material_main, cancelMaterialInsert);
    lcd_lib_draw_stringP(3, 0, PSTR("Heating printhead"));
    lcd_lib_draw_stringP(3, 10, PSTR("for material removal"));

    char buffer[20];
    memset (buffer,0,sizeof(buffer));


    char* c;

    c = int_to_string(temp, buffer/*, PSTR( DEGREE_C_SYMBOL )*/);
    *c++ = TEMPERATURE_SEPARATOR;
    c = int_to_string(target, c, PSTR( DEGREE_C_SYMBOL ));
    lcd_lib_draw_string_center(20, buffer);

    lcd_progressbar(progress);
    LED_HEAT();
    lcd_lib_update_screen();
}

static void lcd_menu_change_material_remove()
{
    run_history = true;
    lcd_info_screen(lcd_menu_material_main, cancelMaterialInsert);
    lcd_lib_draw_stringP(3, 20, PSTR("Reversing material"));

    if (!blocks_queued())
        {
            lcd_lib_beep();
            led_glow_dir = led_glow = 0;
            currentMenu = lcd_menu_change_material_remove_wait_user;
            SELECT_MAIN_MENU_ITEM(0);
            //Disable the extruder motor so you can pull out the remaining filament.
            disable_e0();
            disable_e1();
            disable_e2();
        }

    long pos = -st_get_position(E_AXIS);
    long targetPos = lround(FILAMENT_REVERSAL_LENGTH*axis_steps_per_unit[E_AXIS]);
    uint8_t progress = (pos * 125 / targetPos);
    lcd_progressbar(progress);
    lcd_lib_led_color(48,48,255);
    lcd_lib_update_screen();
}

static void lcd_menu_change_material_remove_wait_user_ready()
{
    run_history = true;
    current_position[E_AXIS] = 0;
    plan_set_e_position(current_position[E_AXIS]);
    lcd_change_to_menu(lcd_menu_change_material_insert_wait_user, MAIN_MENU_ITEM_POS(0));

    char buffer[32];
    memset (buffer,0,sizeof(buffer));

    enquecommand_P(PSTR("G28 X0 Y0"));
    sprintf_P(buffer, PSTR("G1 F%i X%i Y%i"), int(homing_feedrate[0]), int (X_MAX_LENGTH/2), 10);
    enquecommand(buffer);
    LED_GLOW();
}

static void lcd_menu_change_material_remove_wait_user()
{
    LED_GLOW();
    run_history = true;
    lcd_question_screen(NULL, lcd_menu_change_material_remove_wait_user_ready, PSTR("READY"), lcd_menu_main, cancelMaterialInsert, PSTR("CANCEL"));
    lcd_lib_draw_string_centerP(20, PSTR("Remove material"));
    lcd_lib_update_screen();
}

static void lcd_menu_change_material_insert_wait_user()
{
    LED_GLOW();
    run_history = true;
    if (printing_state == PRINT_STATE_NORMAL && movesplanned() < 2)
        {
            current_position[E_AXIS] += 0.5;
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_SPEED, active_extruder);
        }

    lcd_question_screen(NULL, lcd_menu_change_material_insert_wait_user_ready, PSTR("READY"), lcd_menu_main, cancelMaterialInsert, PSTR("CANCEL"));
    lcd_lib_draw_string_centerP(10, PSTR("Insert new material"));
    lcd_lib_draw_string_centerP(20, PSTR("from the backside of"));
    lcd_lib_draw_string_centerP(30, PSTR("your machine,"));
    lcd_lib_draw_string_centerP(40, PSTR("above the arrow."));
    lcd_lib_update_screen();
}

static void lcd_menu_change_material_insert_wait_user_ready()
{
    run_history = true;
    //Override the max feedrate and acceleration values to get a better insert speed and speedup/slowdown
    float old_max_feedrate_e = max_feedrate[E_AXIS];
    float old_retract_acceleration = retract_acceleration;
    max_feedrate[E_AXIS] = FILAMENT_INSERT_FAST_SPEED;
    retract_acceleration = FILAMENT_LONG_MOVE_ACCELERATION;

    current_position[E_AXIS] = 0;
    plan_set_e_position(current_position[E_AXIS]);
    for(uint8_t n=0; n<6; n++)
        {
            current_position[E_AXIS] += FILAMENT_FORWARD_LENGTH / 6;
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_FAST_SPEED, active_extruder);
        }

    //Put back origonal values.
    max_feedrate[E_AXIS] = old_max_feedrate_e;
    retract_acceleration = old_retract_acceleration;

    lcd_change_to_menu(lcd_menu_change_material_insert_forward);
}

static void lcd_menu_change_material_insert_forward()
{
    run_history = true;
    lcd_info_screen(lcd_menu_main, cancelMaterialInsert);
    lcd_lib_draw_stringP(3, 20, PSTR("Forwarding material"));

    if (!blocks_queued())
        {
            lcd_lib_beep();
            led_glow_dir = led_glow = 0;

            digipot_current(2, motor_current_setting[2]*2/3);//Set the E motor power lower to we skip instead of grind.
            currentMenu = lcd_menu_change_material_insert;
            SELECT_MAIN_MENU_ITEM(0);
        }

    long pos = st_get_position(E_AXIS);
    long targetPos = lround(FILAMENT_FORWARD_LENGTH*axis_steps_per_unit[E_AXIS]);
    uint8_t progress = (pos * 125 / targetPos);
    lcd_progressbar(progress);
    lcd_lib_led_color(48,48,255);
    lcd_lib_update_screen();
}

static void materialInsertReady()
{
    current_position[E_AXIS] -= END_OF_PRINT_RETRACTION;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 25*60, active_extruder);
    cancelMaterialInsert();

}

static void lcd_menu_change_material_insert()
{
    LED_GLOW();
    run_history = true;
    lcd_question_screen(lcd_menu_change_material_select_material, materialInsertReady, PSTR("READY"), lcd_menu_main, cancelMaterialInsert, PSTR("CANCEL"));
    lcd_lib_draw_string_centerP(20, PSTR("Wait till material"));
    lcd_lib_draw_string_centerP(30, PSTR("comes out the nozzle"));

    if (movesplanned() < 2)
        {
            current_position[E_AXIS] += 0.5;
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], FILAMENT_INSERT_EXTRUDE_SPEED, active_extruder);
        }
    lcd_lib_led_color(48,48,255);
    lcd_lib_update_screen();
}

static char* lcd_menu_change_material_select_material_callback(uint8_t nr)
{
    char* c =lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring	;
    eeprom_read_block(material_name_buf, EEPROM_MATERIAL_NAME_OFFSET(nr), MATERIAL_NAME_LENGTH);
    material_name_buf[MATERIAL_NAME_LENGTH] = '\0';
    strcpy (c,material_name_buf);
    return c;
}

static void lcd_menu_change_material_select_material_details_callback(uint8_t nr)
{
    char buffer[32];
    char* c = buffer;

    if (led_glow_dir)
        {
            c = float_to_string(eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(nr)), c, PSTR("mm"));
            while(c < buffer + 10) *c++ = ' ';
            strcpy_P(c, PSTR("Flow:"));
            c += 5;
            c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(nr)), c, PSTR("%"));
        }
    else
        {
            c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(nr)), c, PSTR( DEGREE_C_SYMBOL ));
            *c++ = ' ';
            c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(nr)), c, PSTR( DEGREE_C_SYMBOL ));
            while(c < buffer + 10) *c++ = ' ';
            strcpy_P(c, PSTR("Fan: "));
            c += 5;
            c = int_to_string(eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(nr)), c, PSTR("%"));
        }
    lcd_lib_draw_string(5, 53, buffer);
}

static void lcd_menu_change_material_select_material()
{
    uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());

    lcd_scroll_menu(PSTR("MATERIAL"), count, lcd_menu_change_material_select_material_callback, lcd_menu_change_material_select_material_details_callback);
    if (lcd_lib_button_pressed)
        {
            lcd_material_set_material(SELECTED_SCROLL_MENU_ITEM(), active_extruder);

            lcd_change_to_menu(lcd_menu_material_selected, MAIN_MENU_ITEM_POS(0));
        }
}


static char* lcd_material_select_callback(uint8_t nr)
{
    char* c = lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring	;


    uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
    if (nr == 0)
        strcpy_P(c, PSTR("< RETURN"));
    else
        if (nr > count)
            strcpy_P(c, PSTR("Customize"));
        else
            {
                eeprom_read_block(material_name_buf, EEPROM_MATERIAL_NAME_OFFSET(nr - 1), MATERIAL_NAME_LENGTH);
                material_name_buf[MATERIAL_NAME_LENGTH] = '\0';
                strcpy (c,material_name_buf);

            }
    return c;
}

static void lcd_material_select_details_callback(uint8_t nr)
{
    uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
    if (nr == 0)
        {

        }
    else
        if (nr <= count)
            {
                char buffer[32];
                memset (buffer,0,sizeof(buffer));

                char* c = buffer;
                nr -= 1;

                if (led_glow_dir)
                    {
                        c = float_to_string(eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(nr)), c, PSTR("mm"));
                        while(c < buffer + 10) *c++ = ' ';
                        strcpy_P(c, PSTR("Flow:"));
                        c += 5;
                        c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(nr)), c, PSTR("%"));
                    }
                else
                    {
                        c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(nr)), c, PSTR( DEGREE_C_SYMBOL ));
                        *c++ = ' ';
                        c = int_to_string(eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(nr)), c, PSTR( DEGREE_C_SYMBOL ));
                        while(c < buffer + 10) *c++ = ' ';
                        strcpy_P(c, PSTR("Fan: "));
                        c += 5;
                        c = int_to_string(eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(nr)), c, PSTR("%"));
                    }
                lcd_lib_draw_string(5, 53, buffer);
            }
        else
            {
                lcd_lib_draw_string_centerP(53, PSTR("Modify the settings"));
            }
}

static void lcd_menu_material_select()
{
    uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());

    lcd_scroll_menu(PSTR("MATERIAL"), count + MATERIAL_PRESETS, lcd_material_select_callback, lcd_material_select_details_callback);
    if (lcd_lib_button_pressed)
        {
            if (IS_SELECTED_SCROLL(0))
                lcd_change_to_menu(lcd_menu_material_main);
            else
                if (IS_SELECTED_SCROLL(count + 1))
                    lcd_change_to_menu(lcd_menu_material_settings);
                else
                    {
                        lcd_material_set_material(SELECTED_SCROLL_MENU_ITEM() - 1, active_extruder);

                        lcd_change_to_menu(lcd_menu_material_selected, MAIN_MENU_ITEM_POS(0));
                    }
        }
}

static void lcd_menu_material_selected()
{
    lcd_info_screen(lcd_menu_main, NULL, PSTR("OK"));
    lcd_lib_draw_string_centerP(20, PSTR("Selected material:"));
    lcd_lib_draw_string_center(30,material_name[active_extruder]);
#if EXTRUDERS > 1
    if (active_extruder == 0)
        lcd_lib_draw_string_centerP(40, PSTR("for primary nozzle"));
    else
        if (active_extruder == 1)
            lcd_lib_draw_string_centerP(40, PSTR("for secondary nozzle"));
#endif
    lcd_lib_update_screen();
}

static char* lcd_material_settings_callback(uint8_t nr)
{
    char* c = lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring	;


    if (nr == 0)
        strcpy_P(c, PSTR("< RETURN"));
    else
        if (nr == 1)
            strcpy_P(c, PSTR("Temperature"));
        else
            if (nr == 2)
                strcpy_P(c, PSTR("Heated buildplate"));
            else
                if (nr == 3)
                    strcpy_P(c, PSTR("Diameter"));
                else
                    if (nr == 4)
                        strcpy_P(c, PSTR("Fan"));
                    else
                        if (nr == 5)
                            strcpy_P(c, PSTR("Flow %"));
                        else
                            if (nr == 6)
                                strcpy_P(c, PSTR("Store as preset"));
                            else
                                strcpy_P(c, PSTR("???"));
    return c;
}

static void lcd_material_settings_details_callback(uint8_t nr)
{
    char buffer[10];
    memset (buffer,0,sizeof(buffer));
    ;
    if (nr == 0)
        {
            return;
        }
    else
        if (nr == 1)
            {
                int_to_string(material[active_extruder].temperature, buffer, PSTR( DEGREE_C_SYMBOL ));
            }
        else
            if (nr == 2)
                {
                    int_to_string(material[active_extruder].bed_temperature, buffer, PSTR( DEGREE_C_SYMBOL ));
                }
            else
                if (nr == 3)
                    {
                        float_to_string(material[active_extruder].diameter, buffer, PSTR("mm"));
                    }
                else
                    if (nr == 4)
                        {
                            int_to_string(material[active_extruder].fan_speed, buffer, PSTR("%"));
                        }
                    else
                        if (nr == 5)
                            {
                                int_to_string(material[active_extruder].flow, buffer, PSTR("%"));
                            }
    lcd_lib_draw_string(5, 53, buffer);
}

static void lcd_menu_material_settings()
{
    lcd_scroll_menu(PSTR("MATERIAL"), 7, lcd_material_settings_callback, lcd_material_settings_details_callback);
    if (lcd_lib_button_pressed)
        {
            if (IS_SELECTED_SCROLL(0))
                {
                    lcd_change_to_menu(lcd_menu_material_main);
                    lcd_material_store_current_material();
                }
            else
                if (IS_SELECTED_SCROLL(1))
                    LCD_EDIT_SETTING(material[active_extruder].temperature, "Temperature", "C", 0, HEATER_0_MAXTEMP - 15);
                else
                    if (IS_SELECTED_SCROLL(2))
                        LCD_EDIT_SETTING(material[active_extruder].bed_temperature, "Buildplate Temp.", "C", 0, BED_MAXTEMP - 15);
                    else
                        if (IS_SELECTED_SCROLL(3))
                            LCD_EDIT_SETTING_FLOAT001(material[active_extruder].diameter, "Material Diameter", "mm", 0, 100);
                        else
                            if (IS_SELECTED_SCROLL(4))
                                LCD_EDIT_SETTING(material[active_extruder].fan_speed, "Fan speed", "%", 0, 100);
                            else
                                if (IS_SELECTED_SCROLL(5))
                                    LCD_EDIT_SETTING(material[active_extruder].flow, "Material flow", "%", 1, 1000);
                                else
                                    if (IS_SELECTED_SCROLL(6))
                                        lcd_change_to_menu(lcd_menu_material_settings_store);
        }
}

static char* lcd_menu_material_settings_store_callback(uint8_t nr)
{
    char* c = lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring	;


    uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
    if (nr == 0)
        strcpy_P(c, PSTR("< RETURN"));
    else
        if (nr > count)
            strcpy_P(c, PSTR("New preset"));
        else
            {
                eeprom_read_block(material_name_buf, EEPROM_MATERIAL_NAME_OFFSET(nr - 1), MATERIAL_NAME_LENGTH);
                material_name_buf[MATERIAL_NAME_LENGTH] = '\0';
                strncpy (c,material_name_buf,MATERIAL_NAME_LENGTH);
                return c;
            }
    return c;
}

static void lcd_menu_material_settings_store_details_callback(uint8_t nr)
{
}
//-----------------------------------------------------------------------------------------------------------------
static void makeCustomName( uint8_t idx );
static void lcd_menu_material_settings_store()
{
    uint8_t count = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
    if (count == EEPROM_MATERIAL_SETTINGS_MAX_COUNT)
        count--;
    lcd_scroll_menu(PSTR("PRESETS"), MATERIAL_PRESETS + count, lcd_menu_material_settings_store_callback, lcd_menu_material_settings_store_details_callback);

    if (lcd_lib_button_pressed)
        {
            if (!IS_SELECTED_SCROLL(0))
                {
                    uint8_t idx = SELECTED_SCROLL_MENU_ITEM() - 1;
                    if (idx == count)
                        {
                            makeCustomName(idx);
                            eeprom_write_block(material_name_buf, EEPROM_MATERIAL_NAME_OFFSET(idx), MATERIAL_NAME_LENGTH);
                            eeprom_write_byte(EEPROM_MATERIAL_COUNT_OFFSET(), idx + 1);
                        }
                    lcd_material_store_material(idx);
                    lcd_lib_beep_ext (600,100);
                }
            lcd_change_to_menu(lcd_menu_material_settings, SCROLL_MENU_ITEM_POS(6));
        }
}



void lcd_material_reset_defaults()
{
    //Fill in the defaults
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Materials reset");
    char buffer[MATERIAL_NAME_LENGTH];
    for (int a= 0; a< MATERIAL_PRESETS; a++)
        {
            strncpy_P(buffer, (char PROGMEM *) presets[a].name,MATERIAL_NAME_LENGTH);		// this will pad out the whole length with 0 as needed.
            eeprom_write_block(buffer                                  , EEPROM_MATERIAL_NAME_OFFSET(a), MATERIAL_NAME_LENGTH);
            material_preset temp;
            memcpy_P((void*) &temp,&presets[a],sizeof (material_preset));
            eeprom_write_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(a)    ,temp.temperature);
            eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(a),temp.bed);
            eeprom_write_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(a)      ,temp.fan_speed);
            eeprom_write_word(EEPROM_MATERIAL_FLOW_OFFSET(a)           ,temp.flow);
            eeprom_write_float(EEPROM_MATERIAL_DIAMETER_OFFSET(a)      ,temp.diameter);
        }
    uint8_t e;
    for(e=0; e<EXTRUDERS; e++)
        {
            makeCustomName(e);
            strcpy(material_name[e],material_name_buf);
            lcd_material_store_current_material();
        }

    eeprom_write_byte(EEPROM_MATERIAL_COUNT_OFFSET(), MATERIAL_PRESETS);
    lcd_lib_beep_ext(500,150);
    lcd_lib_beep_ext(750,100);
}

void lcd_material_set_material(uint8_t nr, uint8_t e)
{
    material[e].temperature = eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(nr));
    material[e].bed_temperature = eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(nr));
    material[e].flow = eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(nr));

    material[e].fan_speed = eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(nr));
    material[e].diameter = eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(nr));
    eeprom_read_block(material_name[e], EEPROM_MATERIAL_NAME_OFFSET(nr), MATERIAL_NAME_LENGTH);
    material_name[e][MATERIAL_NAME_LENGTH] = '\0';
    if (material[e].temperature > HEATER_0_MAXTEMP - 15)
        material[e].temperature = HEATER_0_MAXTEMP - 15;
    if (material[e].bed_temperature > BED_MAXTEMP - 15)
        material[e].bed_temperature = BED_MAXTEMP - 15;

    lcd_material_store_current_material();
}

void lcd_material_store_material(uint8_t nr)
{
    eeprom_write_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(nr), material[active_extruder].temperature);
    eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(nr), material[active_extruder].bed_temperature);
    eeprom_write_word(EEPROM_MATERIAL_FLOW_OFFSET(nr), material[active_extruder].flow);

    eeprom_write_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(nr), material[active_extruder].fan_speed);
    eeprom_write_float(EEPROM_MATERIAL_DIAMETER_OFFSET(nr), material[active_extruder].diameter);
    eeprom_write_block(material_name_buf, EEPROM_MATERIAL_NAME_OFFSET(nr), MATERIAL_NAME_LENGTH);
}

void lcd_material_read_current_material()
{
    for(uint8_t e=0; e<EXTRUDERS; e++)
        {
            material[e].temperature = eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));
            material[e].bed_temperature = eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));
            material[e].flow = eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));

            material[e].fan_speed = eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));
            material[e].diameter = eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e));
            strcpy_P(material_name[e],PSTR("???"));

            eeprom_read_block(material_name[e], EEPROM_MATERIAL_NAME_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), MATERIAL_NAME_LENGTH);
            material_name[e][MATERIAL_NAME_LENGTH] = '\0';

        }
}

void lcd_material_store_current_material()
{
    for(uint8_t e=0; e<EXTRUDERS; e++)
        {
            eeprom_write_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].temperature);
            eeprom_write_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].bed_temperature);
            eeprom_write_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].fan_speed);
            eeprom_write_word(EEPROM_MATERIAL_FLOW_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].flow);
            eeprom_write_float(EEPROM_MATERIAL_DIAMETER_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), material[e].diameter);
            eeprom_write_block(material_name[e], EEPROM_MATERIAL_NAME_OFFSET(EEPROM_MATERIAL_SETTINGS_MAX_COUNT+e), MATERIAL_NAME_LENGTH);
        }
}

bool lcd_material_verify_material_settings()
{
    SERIAL_ECHO_START;
    uint8_t max_mats = eeprom_read_byte(EEPROM_MATERIAL_COUNT_OFFSET());
    SERIAL_ECHOPAIR("Checking ", (unsigned long) max_mats);
    SERIAL_ECHOPAIR(" presets and ", (unsigned long)  EXTRUDERS);
    SERIAL_ECHOLNPGM (" extruder settings:");

    if (max_mats < 2 || max_mats > EEPROM_MATERIAL_SETTINGS_MAX_COUNT)
        return false;
    byte cnt =0;
    while(cnt < max_mats+EXTRUDERS)
        {
            SERIAL_ECHOPAIR("Checking preset # ",(unsigned long) cnt);
            if (eeprom_read_word(EEPROM_MATERIAL_TEMPERATURE_OFFSET(cnt)) > HEATER_0_MAXTEMP)
                return false;
            if (eeprom_read_word(EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(cnt)) > BED_MAXTEMP)
                return false;
            if (eeprom_read_byte(EEPROM_MATERIAL_FAN_SPEED_OFFSET(cnt)) > 100)
                return false;
            if (eeprom_read_word(EEPROM_MATERIAL_FLOW_OFFSET(cnt)) > 1000)
                return false;
            if (eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(cnt)) > 10.0)
                return false;
            if (eeprom_read_float(EEPROM_MATERIAL_DIAMETER_OFFSET(cnt)) < 0.1)
                return false;
            strcpy_P (material_name_buf,"???");
            eeprom_read_block(material_name_buf, EEPROM_MATERIAL_NAME_OFFSET(cnt), MATERIAL_NAME_LENGTH);
            material_name_buf[MATERIAL_NAME_LENGTH] = '\0';

            if (cnt >= max_mats )
                { SERIAL_ECHOPAIR(".....verified extruder setting # ",(unsigned long) cnt-max_mats);}
            else
                { SERIAL_ECHOPAIR(".....verified preset # ",(unsigned long) cnt);}
            SERIAL_ECHO(" (");
            SERIAL_ECHO(material_name_buf);

            SERIAL_ECHO(")");
            SERIAL_ECHOLN("");
            cnt++;
        }
    return true;
}
//-----------------------------------------------------------------------------------------------------------------
void makeCustomName( uint8_t idx )
{
    char buffer[MATERIAL_NAME_LENGTH+1];
    strcpy_P(buffer,PSTR ("CUSTOM_")) ;
    char * c= int_to_string(idx - 1 - MATERIAL_PRESETS+2, buffer + 7);
    *c++=0;

    strcpy (material_name_buf,buffer);
}
#endif//ENABLE_ULTILCD2
