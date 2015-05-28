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

extern long position[4];

#define SERIAL_CONTROL_TIMEOUT 5000

#pragma GCC diagnostic ignored "-Wstrict-aliasing" // Code that causes warning goes here #pragma GCC diagnostic pop


unsigned long lastSerialCommandTime;
bool serialScreenShown;
uint8_t led_brightness_level = 100;
uint8_t led_mode = LED_MODE_ALWAYS_ON;
char message_string [MAX_MESSAGE_LEN+1] ="---";
int message_counter = 0;



//-----------------------------------------------------------------------------------------------------------------
void lcd_lib_show_message();

void updateTempHistory();

static void lcd_menu_startup();
#ifdef SPECIAL_STARTUP
static void lcd_menu_special_startup();
#endif//SPECIAL_STARTUP

static void lcd_menu_breakout();

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
// 	memset (bed_history,0,HISTORY_SIZE);
// 	memset (temp_history,0,HISTORY_SIZE);
	clearHistory();

}

//-----------------------------------------------------------------------------------------------------------------
void lcd_update()
{
    if (!lcd_lib_update_ready()) return;
    lcd_lib_buttons_update();
    card.updateSDInserted();

    if (led_glow_dir)
        {
            led_glow-=2;
            if (led_glow == 0) led_glow_dir = 0;
        }
    else
        {
            led_glow+=2;
            if (led_glow == 126)
                {
                    led_glow_dir = 1;
					if (!IsStopped() ) updateTempHistory();
                }
        }

    if (IsStopped())
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
            while (!lcd_lib_update_ready()) delay(20);
        }
    else
        if (millis() - lastSerialCommandTime < SERIAL_CONTROL_TIMEOUT)
            {
                /*  if (!serialScreenShown)
                  {
                      lcd_lib_clear();
                      lcd_lib_draw_string_centerP(20, PSTR("Printing with USB..."));
                	lcd_lib_show_message(40);

                      serialScreenShown = true;
                  }*/
                if (printing_state == PRINT_STATE_HEATING || printing_state == PRINT_STATE_HEATING_BED || printing_state == PRINT_STATE_HOMING)
                    lastSerialCommandTime = millis();
                lcd_lib_update_screen();
                while (!lcd_lib_update_ready()) delay(20);

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
	if (led_mode == LED_MODE_ALWAYS_ON)
		analogWrite(LED_PIN, int(led_glow * 2 * led_brightness_level / 100));
	
	while (!lcd_lib_update_ready()) delay (20);
    lcd_lib_update_screen();
	
    if (led_glow_dir || lcd_lib_button_pressed)
        {
            if (led_mode == LED_MODE_ALWAYS_ON)
                analogWrite(LED_PIN, 255 * led_brightness_level / 100);
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
                    lcd_lib_clear();
// 			lcd_lib_draw_string_center(10,"UM" SQUARED_SYMBOL " - Nerd fork");
// 			lcd_lib_draw_string_center(30,STRING_CONFIG_H_AUTHOR);
// 			lcd_lib_draw_string_center(40,STRING_VERSION_CONFIG_H);
                    lcd_lib_led_color(255,255,255,false);
                    lcd_lib_update_screen();

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
	lcd_cache_new.getData(LCD_CACHE::NO_MODE);

    if (LED_DIM_TIME>0 && (millis() -  last_user_interaction> LED_DIM_TIME*MILLISECONDS_PER_MINUTE))
        {
            lcd_main_screensaver();
            delay(35);
            return;
        }

    lcd_triple_menu_low(PSTR("PRINT"), PSTR("FILA"), PSTR("SYSTEM"));

//    lcd_triple_X_menu_low(MainMenuLlist,5);
    LED_GLOW();
    if (lcd_lib_button_pressed)
        {
	//	IS_SELECTED_MAIN


            if (IS_SELECTED_MAIN(MAIN_MENU_PRINT))
                {
					lcd_cache_new.getData(LCD_CACHE::NO_MODE);
					card.release();
                    lcd_change_to_menu(lcd_sd_filemenu_doAction, SCROLL_MENU_ITEM_POS(0));
                }
            else
                if (IS_SELECTED_MAIN(MAIN_MENU_FILAMENT))
                    lcd_change_to_menu(lcd_menu_material);
                else
                    if (IS_SELECTED_MAIN(MAIN_MENU_SYSTEM))
                        lcd_change_to_menu(lcd_menu_maintenance);
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

    char buffer[24];
    memset (buffer,0,24);
    char* c;
    c = buffer;
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

//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

#define BREAKOUT_PADDLE_WIDTH 21
//Use the lcd_cache memory to store breakout data, so we do not waste memory.

// really?  8 bytes are worth this complexity ?

#define ball_x  ( lcd_cache_new.getData(LCD_CACHE::BREAKOUT).breakout.ball_x)
#define ball_y  ( lcd_cache_new.getData(LCD_CACHE::BREAKOUT).breakout.ball_y)
#define ball_dx ( lcd_cache_new.getData(LCD_CACHE::BREAKOUT).breakout.ball_dx)
#define ball_dy ( lcd_cache_new.getData(LCD_CACHE::BREAKOUT).breakout.ball_dy)
static void lcd_menu_breakout()
{
	byte * rawstorage = lcd_cache_new.getData(LCD_CACHE::BREAKOUT).breakout.brick_data;
    if (lcd_lib_encoder_pos == ENCODER_NO_SELECTION)
        {
            lcd_lib_encoder_pos = (128 - BREAKOUT_PADDLE_WIDTH) / 2 / 2;
            for(uint8_t y=0; y<3; y++)
                for(uint8_t x=0; x<5; x++)
                    rawstorage[x+y*5] = 3;
            ball_x = 0;
            ball_y = 57 << 8;
            ball_dx = 0;
            ball_dy = 0;
        }

    if (lcd_lib_encoder_pos < 0) lcd_lib_encoder_pos = 0;
    if (lcd_lib_encoder_pos * 2 > 128 - BREAKOUT_PADDLE_WIDTH - 1) lcd_lib_encoder_pos = (128 - BREAKOUT_PADDLE_WIDTH - 1) / 2;
    ball_x += ball_dx;
    ball_y += ball_dy;
    if (ball_x < 1 << 8) ball_dx = abs(ball_dx);
    if (ball_x > 124 << 8) ball_dx = -abs(ball_dx);
    if (ball_y < (1 << 8)) ball_dy = abs(ball_dy);
    if (ball_y < (3 * 10) << 8)
        {
            uint8_t x = (ball_x >> 8) / 25;
            uint8_t y = (ball_y >> 8) / 10;
            if (rawstorage[x+y*5])
                {
                    rawstorage[x+y*5]--;
                    ball_dy = abs(ball_dy);
                    for(y=0; y<3; y++)
                        {
                            for(x=0; x<5; x++)
                                if (rawstorage[x+y*5])
                                    break;
                            if (x != 5)
                                break;
                        }
                    if (x==5 && y==3)
                        {
                            for(y=0; y<3; y++)
                                for(x=0; x<5; x++)
                                    rawstorage[x+y*5] = 3;
                        }
                }
        }
    if (ball_y > (58 << 8))
        {
            if (ball_x < (lcd_lib_encoder_pos * 2 - 2) << 8 || ball_x > (lcd_lib_encoder_pos * 2 + BREAKOUT_PADDLE_WIDTH) << 8)
                lcd_change_to_menu(lcd_menu_main);
            ball_dx += (ball_x - ((lcd_lib_encoder_pos * 2 + BREAKOUT_PADDLE_WIDTH / 2) * 256)) / 64;
            ball_dy = -512 + abs(ball_dx);
        }
    if (ball_dy == 0)
        {
            ball_y = 57 << 8;
            ball_x = (lcd_lib_encoder_pos * 2 + BREAKOUT_PADDLE_WIDTH / 2) << 8;
            if (lcd_lib_button_pressed)
                {
                    ball_dx = -256 + lcd_lib_encoder_pos * 8;
                    ball_dy = -512 + abs(ball_dx);
                }
        }

    lcd_lib_clear();

    for(uint8_t y=0; y<3; y++)
        for(uint8_t x=0; x<5; x++)
            {
                if (rawstorage[x+y*5])
                    lcd_lib_clear(3 + x*25, 2 + y * 10, 23 + x*25, 10 + y * 10);
                if (rawstorage[x+y*5] == 2)
                    lcd_lib_draw_shade(4 + x*25, 3 + y * 10, 22 + x*25, 9 + y * 10);
                if (rawstorage[x+y*5] == 3)
                    lcd_lib_set(4 + x*25, 3 + y * 10, 22 + x*25, 9 + y * 10);
            }

    lcd_lib_draw_box(ball_x >> 8, ball_y >> 8, (ball_x >> 8) + 2, (ball_y >> 8) + 2);
    lcd_lib_draw_box(lcd_lib_encoder_pos * 2, 60, lcd_lib_encoder_pos * 2 + BREAKOUT_PADDLE_WIDTH, 63);
    lcd_lib_update_screen();
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
/* Warning: This function is called from interrupt context */
void lcd_buttons_update()
{
    lcd_lib_buttons_update_interrupt();
}
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------


#define SS_MAX_SPED 3.0
#define SS_MAX_ACC 0.25
#define SS_GRAVITY 0.02
#define SS_MARGIN 10



//-----------------------------------------------------------------------------------------------------------------
void lcd_main_screensaver()
{
    static float pos_x = LCD_GFX_WIDTH/2;
    static float pos_y = LCD_GFX_HEIGHT/2;
    static float dx = (random(-SS_MAX_SPED*100,SS_MAX_SPED*100)) / 100.0;
    static float dy = (random(-SS_MAX_SPED*100,SS_MAX_SPED*100)) / 100.0;
    static float ax = (random(-SS_MAX_ACC*1000,SS_MAX_ACC*1000)) / 1000.0;
    static float ay = (random(-SS_MAX_ACC*1000,SS_MAX_ACC*1000)) / 1000.0;
    if (random(100) ==50 || abs(dx)>=SS_MAX_SPED || abs(dy) >= SS_MAX_SPED)
        {
            ax =  (random(-SS_MAX_ACC*1000,SS_MAX_ACC*1000)) / 1000.0;
            ay =  (random(-SS_MAX_ACC*1000,SS_MAX_ACC*1000)) / 1000.0;
        }
    if (random(12)==1)
        ax += (random(-SS_MAX_ACC*1000,SS_MAX_ACC*1000)) / 3000.0;
    if (random(12)==1)
        ay += (random(-SS_MAX_ACC*1000,SS_MAX_ACC*1000)) / 3000.0;
    if (pos_x > LCD_GFX_WIDTH/2 ) ax-=SS_GRAVITY;
    if (pos_x < LCD_GFX_WIDTH/2 ) ax+=SS_GRAVITY;
    if (pos_y > LCD_GFX_HEIGHT/2 ) ay-=SS_GRAVITY;
    if (pos_y < LCD_GFX_HEIGHT/2 ) ay+=SS_GRAVITY;
    constrain(ax,(-SS_MAX_ACC),SS_MAX_ACC);
    constrain(ay,(-SS_MAX_ACC),SS_MAX_ACC);
    dx*=0.98;
    dy*=0.98;
    dx+=ax;
    dy+=ay;
    constrain(dx,(-SS_MAX_SPED),SS_MAX_SPED);
    constrain(dy,(-SS_MAX_SPED),SS_MAX_SPED);
    pos_x+= dx;
    pos_y+= dy;
    if (pos_x >= LCD_GFX_WIDTH-SS_MARGIN || pos_x < SS_MARGIN ) dx = -dx;
    if (pos_y >=LCD_GFX_HEIGHT-SS_MARGIN || pos_y < SS_MARGIN ) dy = -dy;
    constrain(pos_x,SS_MARGIN, (LCD_GFX_WIDTH-SS_MARGIN ));
    constrain(pos_y,SS_MARGIN, (LCD_GFX_HEIGHT-SS_MARGIN));
    lcd_lib_draw_string(pos_x-3,pos_y-3,"o");
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
