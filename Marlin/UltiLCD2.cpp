#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
#include "UltiLCD2.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_gfx.h"
#include "UltiLCD2_menu_material.h"
#include "UltiLCD2_menu_print.h"
#include "UltiLCD2_menu_first_run.h"
#include "UltiLCD2_menu_maintenance.h"
#include "cardreader.h"
#include "ConfigurationStore.h"
#include "temperature.h"
#include "pins.h"
#include "Marlin.h"
#include "lifetime_stats.h"
#include "voltage.h"
#include "dht.h"
#include "ultralcd_st7920_u8glib_rrd.h"
#include "MenuUseful.h"
#include "gcode.h"
#include "stringHelpers.h"
#include "ScreenSaver.h"

extern long position[4];

#define SERIAL_CONTROL_TIMEOUT 5000

#pragma GCC diagnostic ignored "-Wstrict-aliasing" // Code that causes warning goes here #pragma GCC diagnostic pop


unsigned long lastSerialCommandTime;
bool serialScreenShown;
uint8_t led_brightness_level = 100;
uint8_t led_mode = LED_MODE_ALWAYS_ON;
char message_string[MAX_MESSAGE_LEN+1] ="---";
int message_counter = 0;



//-----------------------------------------------------------------------------------------------------------------
void lcd_lib_show_message();

void updateTempHistory();

static void lcd_menu_startup();
#ifdef SPECIAL_STARTUP
static void lcd_menu_special_startup();
#endif//SPECIAL_STARTUP


extern float final_e_position;

char* drawStatsInfo( char * buffer, char* c );
//-----------------------------------------------------------------------------------------------------------------
void clearHistory();
//-----------------------------------------------------------------------------------------------------------------
void lcd_init()
{
    lcd_lib_init();
    currentMenu = lcd_menu_startup;
    analogWrite(LED_PIN, 0);
    lastSerialCommandTime = millis() - SERIAL_CONTROL_TIMEOUT;
}

//-----------------------------------------------------------------------------------------------------------------
void doStoppedScreen()
{
    lcd_lib_clear();
    lcd_lib_draw_stringP(15, 10, PSTR("ERROR - STOPPED"));
    switch(StoppedReason())
        {
            case STOP_REASON_MAXTEMP:
            case STOP_REASON_MINTEMP:
                lcd_lib_draw_stringP(15, 20, PSTR("Temp sensor"));
                break;
            case STOP_REASON_MAXTEMP_BED:
                lcd_lib_draw_stringP(15, 20, PSTR("Temp sensor BED"));
                break;
            case STOP_REASON_SAFETY_TRIGGER:
                lcd_lib_draw_stringP(15, 20, PSTR("Safety circuit"));
                break;
        }
    lcd_lib_draw_stringP(1, 40, PSTR("Contact:"));
    lcd_lib_draw_stringP(1, 50, PSTR("support@ultimaker.com"));
    LED_GLOW_ERROR();
    lcd_lib_update_screen();
    lcd_lib_wait_for_screen_ready();
}

//-----------------------------------------------------------------------------------------------------------------
void lcd_update()
{
    lcd_lib_wait_for_screen_ready();
    lcd_lib_buttons_update();
    card.updateSDInserted();

    if (led_glow_dir)
        {
            led_glow-=2;
            if (led_glow <=1) led_glow_dir = 0;
        }
    else
        {
            led_glow+=2;
            if (led_glow >= 126)
                {
                    led_glow_dir = 1;
                    if (!IsStopped() )
                        updateTempHistory();
                }
        }

    if (IsStopped())
        doStoppedScreen();
    else
        if (millis() - lastSerialCommandTime < SERIAL_CONTROL_TIMEOUT)
            {
                if (!serialScreenShown)
                    {
                        lcd_lib_clear();
                        lcd_lib_draw_string_centerP(20, PSTR("Printing with USB..."));
                        lcd_lib_show_message(40);
                        serialScreenShown = true;
                    }
                if (printing_state == PRINT_STATE_HEATING || printing_state == PRINT_STATE_HEATING_BED || printing_state == PRINT_STATE_HOMING)
                    lastSerialCommandTime = millis();
                lcd_lib_update_screen();
            }
        else
            {
                serialScreenShown = false;
                currentMenu();
                if (postMenuCheck) postMenuCheck();
            }
}

//-----------------------------------------------------------------------------------------------------------------
void lcd_menu_startup()
{
    lcd_lib_encoder_pos = ENCODER_NO_SELECTION;
    LED_GLOW();
    lcd_cache_new.getData(LCD_CACHE::NO_MODE);
    lcd_lib_wait_for_screen_ready();
    if (led_glow < 84)
        {

            lcd_lib_draw_gfx(40, 0, ultimakerRobotGfx);
//            lcd_lib_draw_gfx(0, 22, ultimakerTextGfx);
            for(uint8_t n=0; n<10; n++)
                {
                    // 	lcd_lib_set (40+)
                    if (led_glow*2 >= n + 20)
                        lcd_lib_clear(0, n*6, led_glow*2-n-20, 1+n*6);
                    if (led_glow*2 >= n)
                        lcd_lib_clear(led_glow*2 - n, n*6, 127, 1+n*6);
                    else
                        lcd_lib_clear(0, n*6, 127, 1+n*6);
#if 0
                    if (led_glow*2 >= n + 20)
                        lcd_lib_clear(0, 22+n*2, led_glow*2-n-20, 23+n*2);
                    if (led_glow*2 >= n)
                        lcd_lib_clear(led_glow*2 - n, 22+n*2, 127, 23+n*2);
                    else
                        lcd_lib_clear(0, 22+n*2, 127, 23+n*2);
#endif
                }
            /*
            }else if (led_glow < 86) {
                led_glow--;
                //lcd_lib_set();
                //lcd_lib_clear_gfx(0, 22, ultimakerTextGfx);
                lcd_lib_draw_gfx(0, 22, ultimakerTextGfx);
            */
        }
    else
        {
            led_glow--;
            lcd_lib_draw_gfx(40, 0, ultimakerRobotGfx);
            //lcd_lib_clear_gfx(0, 22, ultimakerTextOutlineGfx);
            // lcd_lib_draw_gfx(0, 22, ultimakerTextGfx);
        }
// 
// 	static byte ramp_up = 0;
// 	if (ramp_up < led_brightness_level) ramp_up++;
// 	if (led_mode == LED_MODE_ALWAYS_ON)
// 		analogWrite(LED_PIN, 255 * ramp_up / 100);

    lcd_lib_update_screen();
    if (led_glow_dir || lcd_lib_button_pressed)
        {
            led_glow = led_glow_dir = 0;
            LED_NORMAL();
            if (lcd_lib_button_pressed)
                lcd_lib_beep();

#ifdef SPECIAL_STARTUP
            currentMenu = lcd_menu_special_startup;
#else
            if (!IS_FIRST_RUN_DONE())
                {
                    currentMenu = lcd_menu_first_run_init;
                }
            else
                {
                    //                  lcd_lib_clear();
// 			lcd_lib_draw_string_center(10,"UM" SQUARED_SYMBOL " - Nerd fork");
// 			lcd_lib_draw_string_center(30,STRING_CONFIG_H_AUTHOR);
// 			lcd_lib_draw_string_center(40,STRING_VERSION_CONFIG_H);
                    lcd_lib_led_color(255,255,255,false);
//                    lcd_lib_update_screen();

//			delay (2500);
                    currentMenu = lcd_menu_main;
                    LED_GLOW();
                }
#endif//SPECIAL_STARTUP
        }
}

#ifdef SPECIAL_STARTUP
static void lcd_menu_special_startup()
{
    LED_GLOW();

    lcd_lib_clear();
    lcd_lib_draw_gfx(7, 12, specialStartupGfx);
    lcd_lib_draw_stringP(3, 2, PSTR("Welcome"));
    lcd_lib_draw_string_centerP(47, PSTR("To the Ultimaker2"));
    lcd_lib_draw_string_centerP(55, PSTR("experience!"));
    lcd_lib_update_screen();

    if (lcd_lib_button_pressed)
        {
            if (!IS_FIRST_RUN_DONE())
                {
                    lcd_change_to_menu(lcd_menu_first_run_init);
                }
            else
                {
                    lcd_change_to_menu(lcd_menu_main);
                }
        }
}
#endif//SPECIAL_STARTUP

//-----------------------------------------------------------------------------------------------------------------
void doCooldown()
{
    for(uint8_t n=0; n<EXTRUDERS; n++)
        setTargetHotend(0, n);
    setTargetBed(0);
    fanSpeed = 0;
    fanSpeedOverride=0;	// auto

    //quickStop();         //Abort all moves already in the planner
}


enum MAIN_MENU
{
    MAIN_MENU_PRINT,
    MAIN_MENU_FILAMENT,
// 	 MAIN_MENU_HEAD,
// 	 MAIN_MENU_BED,
    MAIN_MENU_SYSTEM,

    MAIN_MENU_MAX
};

/*

const char  STR_1[] PROGMEM   = ("PRINT");
const char  STR_2[] PROGMEM  = "FILA";
const char  STR_3[] PROGMEM  = ("HEAD");
const char  STR_4[] PROGMEM  = ("BED");
const char  STR_5[] PROGMEM  = ("SYSTEM");

const char * const MainMenuLlist[MAIN_MENU_MAX] PROGMEM ={ STR_1,STR_2,STR_3,STR_4,STR_5};
*/


//-----------------------------------------------------------------------------------------------------------------
// the default menu that appears when you are not printing....
void lcd_menu_main()
{
    did_beep = false;
    run_history = false;
    LED_GLOW();
    if (LED_DIM_TIME>0 && (millis() -  last_user_interaction> LED_DIM_TIME*MILLISECONDS_PER_MINUTE))
        {
            lcd_main_screensaver();
            delay(35);
            return;
        }

    lcd_lib_wait_for_screen_ready();
    lcd_lib_clear();

    lcd_triple_menu_low(PSTR("PRINT"), PSTR("FILA"), PSTR("SYSTEM"));

//    lcd_triple_X_menu_low(MainMenuLlist,5);

    if (lcd_lib_button_pressed)
        {
            switch (SELECTED_MAIN_MENU_ITEM())
                {
                    case MAIN_MENU_PRINT:
                        {
                            lcd_cache_new.getData(LCD_CACHE::NO_MODE);
                            card.release();
                            lcd_change_to_menu(lcd_sd_filemenu_doAction, SCROLL_MENU_ITEM_POS(0));
                        }
                        break;
                    case MAIN_MENU_FILAMENT:
                        lcd_change_to_menu(lcd_menu_material);
                        break;
                    case MAIN_MENU_SYSTEM:
                        lcd_change_to_menu(lcd_menu_maintenance_doAction);
                        break;
                }
        }
    if (lcd_lib_button_down && lcd_lib_encoder_pos == ENCODER_NO_SELECTION)
        {
            led_glow_dir = 0;
            if (led_glow > 200)
                lcd_change_to_menu(lcd_menu_breakout);
        }
    else
        {
            led_glow = led_glow_dir = 0;
        }

    char * buffer = lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring;
    char* c = buffer;
// Show the extruder temperature and target temperature:
    c = int_to_string(current_temperature[0], c, PSTR(TEMPERATURE_SEPARATOR_S));
    c = int_to_string(target_temperature[0], c, PSTR( DEGREE_C_SYMBOL "  "),true);
    lcd_lib_draw_string(5,ROW1, buffer);
    c = buffer;
    c = int_to_string(current_temperature_bed, c, PSTR(TEMPERATURE_SEPARATOR_S));
    c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL "  "),true);
    lcd_lib_draw_string(64+12,ROW1, buffer);
    c = buffer;
    lcd_lib_draw_hline(0,127,ROW2-2);

// we haven't printed anythig, so cycle through a set of information screens
    bool did_print =! (starttime ==0 || stoptime ==0 ) ;

    if (!lcd_lib_show_message (ROW3))
        {
            delay (50);
            bool recently_printed = did_print && (millis() - stoptime < 30000UL);
            if (IS_SELECTED_MAIN(MAIN_MENU_FILAMENT) || (time_phase_a(2) && !recently_printed))	// SHOW CURRENTLY LOADED MATERIAL SETTINGS (EXTR 0)
                {
                    c = drawMaterialInfoScreen(c, buffer);
                    lcd_lib_update_screen();
                    return;
                }

            if (IS_SELECTED_MAIN(MAIN_MENU_SYSTEM) || (time_phase_b(2) && !recently_printed)	)	// SHOW SYSTEM INFO (volts, uptime)
                {
                    c = drawSystemInfoScreen(c, buffer);
                    lcd_lib_update_screen();
                    return;
                }
            if (IS_SELECTED_MAIN(MAIN_MENU_PRINT)|| time_phase_c(2) || recently_printed)		// SHOWLAST PRINT INFO, OR SYSTEM VERSION INFO
                {
                    if (did_print)
                        c = drawLastPrintInfo(buffer, c);
                    else
                        drawCustomFirmwareInfo();
                    lcd_lib_update_screen();
                    return;
                }
            //  if (time_phase_d && !recently_printed)			// SHOW LIFETIME STATS						// default case,  so at least we aleways draw *something*
            {
                c = drawStatsInfo(buffer, c);
                lcd_lib_update_screen();
                return;
            }
        }

}
//-------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
/* Warning: This function is called from interrupt context */
void lcd_buttons_update()
{
    lcd_lib_buttons_update_interrupt();
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------
void lcd_main_screensaver()
{
    lcd_lib_wait_for_screen_ready();
    lcd_lib_clear();

    int a;
    if (lcd_cache_new.getMode()!=LCD_CACHE::SCREENSAVER)
        for (a=0; a<SCREENSAVER_BALLS; a++)
            lcd_cache_new.getData(LCD_CACHE::SCREENSAVER).ss.balls[a].init();

    for (a=0; a<SCREENSAVER_BALLS; a++)
        lcd_cache_new.getData(LCD_CACHE::SCREENSAVER).ss.balls[a].update();
    lcd_lib_update_screen();
}



//-----------------------------------------------------------------------------------------------------------------
char* drawMaterialInfoScreen( char* c, char * buffer )
{
    c = buffer;
    c = float_to_string(material[0].diameter, c, PSTR("mm   "));
    *c++=0;
    strcpy (buffer+8, material_name[0]);
    lcd_lib_draw_string(5,ROW2, buffer);

    c = buffer+6;
    strcpy_P(buffer, PSTR("Temp: "));
    c = int_to_string(material[0].temperature, c, PSTR( DEGREE_C_SYMBOL));
    *c++=0;
    lcd_lib_draw_string_center(ROW3, buffer);
    c = buffer+5;
    strcpy_P(buffer, PSTR("Bed: "));
    c = int_to_string(material[0].bed_temperature, c, PSTR( DEGREE_C_SYMBOL));
    *c++=0;
    lcd_lib_draw_string_center(ROW4, buffer);


    c = buffer+6;
    strcpy_P(buffer, PSTR("Flow: "));
    c = int_to_string(material[0].flow, c, PSTR( "%"));
    *c++=0;
    lcd_lib_draw_string(5,ROW5, buffer);

    c = buffer+5;
    strcpy_P(buffer, PSTR("Fan: "));
    c = int_to_string(material[0].fan_speed, c, PSTR( "%"));
    *c++=0;
    lcd_lib_draw_string_right(ROW5, buffer);	return c;
}



//-----------------------------------------------------------------------------------------------------------------
char* drawSystemInfoScreen( char* c, char * buffer )
{
    c = buffer+8;
    strcpy_P(buffer, PSTR("UPTIME:     "));
    c =EchoTimeSpan(millis() / 1000L,c);
    *c++=0;
    lcd_lib_draw_string_center(ROW2,buffer);
    static float last_voltage = readVoltage();
    static float last_voltage2 = readAVR_VCC();
    static int last_memory = freeMemory();


    static byte counter = 00;
    c = buffer+4;
    strcpy_P(buffer, PSTR("VCC:"));
    c = float_to_string(last_voltage2, c, PSTR("v"));
    *c++=0;
    lcd_lib_draw_string(5,ROW3, buffer);

    c = buffer+4;
    strcpy_P(buffer, PSTR("PSU:"));
    c = float_to_string(last_voltage, c, PSTR("v"));
    *c++=0;
    lcd_lib_draw_string_right(ROW3, buffer);

#ifdef DHT_ENVIRONMENTAL_SENSOR

    c = buffer+4;
    strcpy_P(buffer, PSTR("AMB:"));
    c = int_to_string(last_temp, c,  PSTR( DEGREE_C_SYMBOL));
    *c++=0;
    lcd_lib_draw_string(5,ROW4, buffer);

    c = buffer+6;
    strcpy_P(buffer, PSTR("HUMID:"));
    c = int_to_string(last_humid, c,  PSTR( "%"));
    lcd_lib_draw_string_right (ROW4, buffer);
#endif
    c = buffer+10;
    strcpy_P(buffer, PSTR("Free Mem: "));
    c = int_to_string(last_memory, c, PSTR(" bytes"));
    *c++=0;
    lcd_lib_draw_string_center(ROW5, buffer);

    counter ++;
    if (counter > 10)
        {
            counter =0;
            last_voltage = readVoltage();
            last_voltage2 = readAVR_VCC();
            last_memory = freeMemory();

#ifdef DHT_ENVIRONMENTAL_SENSOR
            updateAmbientSensor();
#endif
        }	return c;
}


//-----------------------------------------------------------------------------------------------------------------
char* drawLastPrintInfo( char * buffer, char* c )
{
    lcd_lib_draw_string_center(ROW2, last_print_name);
    unsigned long printTimeSec = (stoptime-starttime)/1000;

    strcpy_P(buffer, PSTR("Time: "));
    c =EchoTimeSpan(printTimeSec,buffer+5);
    *c++=0;
    lcd_lib_draw_string_center(ROW3, buffer);
    strcpy_P(buffer, PSTR("Est:  "));
    c =EchoTimeSpan(estimatedTime,buffer+4);
    *c++=0;
    lcd_lib_draw_string_center(ROW4, buffer);
    c = buffer;
    if (printTimeSec>0)
        {
            strcpy_P(buffer, PSTR("Net speed:  "));
            c = float_to_string(( estimatedTime)/(float) printTimeSec, c, PSTR(" x"));
            *c++=0;
            lcd_lib_draw_string_center(ROW5, buffer);
        }

    c =float_to_string((final_e_position/**volume_to_filament_length[0]*/)/1000.0,buffer,PSTR("m"));
    strcpy_P(c, PSTR("m of "));
    c+=5;
    strncpy(c, material_name[0],10);
    lcd_lib_draw_string_center(ROW6, buffer);	return c;
}




//-----------------------------------------------------------------------------------------------------------------
void drawCustomFirmwareInfo()
{
    lcd_lib_draw_string_centerP(ROW2,PSTR ("UM" SQUARED_SYMBOL " - Nerd fork"));
    if (time_phase0)
        lcd_lib_draw_string_centerP((ROW3),PSTR (STRING_CONFIG_H_AUTHOR));
    else
        lcd_lib_draw_string_centerP((ROW4),PSTR (STRING_VERSION_CONFIG_H));
    if (time_phase0)
        lcd_lib_draw_string_centerP(HALF_ROW(ROW5),PSTR ("USE_AT_YOUR_OWN_RISK!"));
}


//-----------------------------------------------------------------------------------------------------------------
char* drawStatsInfo( char * buffer, char* c )
{
    strcpy_P(buffer, PSTR("Filament:  "));
    c =float_to_string(lifetime_print_centimeters/100.0,buffer+10,PSTR("m"));
    *c++=0;
    lcd_lib_draw_string_center(ROW2, buffer);

    strcpy_P(buffer, PSTR("Total On:  "));
    c =EchoTimeSpan(lifetime_minutes*60L,buffer+9, false);
    *c++=0;
    lcd_lib_draw_string_center(ROW3, buffer);
    c = buffer;

    strcpy_P(buffer, PSTR("Printing:  "));
    c =EchoTimeSpan(lifetime_print_minutes*60L,buffer+10, false);
    *c++=0;
    lcd_lib_draw_string_center(ROW4, buffer);

    c = buffer;
    c = float_to_string(100.0 * lifetime_print_minutes/lifetime_minutes, c, PSTR(" %"));
    strcpy_P(c , PSTR(" active"));
    lcd_lib_draw_string_center(ROW5, buffer);	return c;
}


#endif//ENABLE_ULTILCD2
