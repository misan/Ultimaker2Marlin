#include <avr/pgmspace.h>

#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
#include "Marlin.h"
#include "cardreader.h"
#include "temperature.h"
#include "lifetime_stats.h"
#include "UltiLCD2.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_menu_print.h"
#include "UltiLCD2_menu_material.h"


// #pragma GCC diagnostic push 
#pragma GCC diagnostic ignored "-Wstrict-aliasing" // Code that causes warning goes here #pragma GCC diagnostic pop

uint8_t lcd_cache[LCD_CACHE_SIZE];
#define LCD_CACHE_NR_OF_FILES() lcd_cache[(LCD_CACHE_COUNT*(LONG_FILENAME_LENGTH+2))]
#define LCD_CACHE_ID(n) lcd_cache[(n)]
#define LCD_CACHE_FILENAME(n) ((char*)&lcd_cache[2*LCD_CACHE_COUNT + (n) * LONG_FILENAME_LENGTH])
#define LCD_CACHE_TYPE(n) lcd_cache[LCD_CACHE_COUNT + (n)]
#define LCD_DETAIL_CACHE_START ((LCD_CACHE_COUNT*(LONG_FILENAME_LENGTH+2))+1)
#define LCD_DETAIL_CACHE_ID() lcd_cache[LCD_DETAIL_CACHE_START]
#define LCD_DETAIL_CACHE_TIME() (*(uint32_t*)&lcd_cache[LCD_DETAIL_CACHE_START+1])
#define LCD_DETAIL_CACHE_MATERIAL(n) (*(uint32_t*)&lcd_cache[LCD_DETAIL_CACHE_START+5+4*n])

// #pragma GCC diagnostic pop 

void doCooldown();//TODO
static void lcd_menu_print_heatup();
static void lcd_menu_print_printing();
static void lcd_menu_print_error();
static void lcd_menu_print_classic_warning();
static void lcd_menu_print_abort();
static void lcd_menu_print_ready();
static void lcd_menu_print_tune();
static void lcd_menu_print_tune_retraction();

float estimated_filament_length_in_m =0;
//  filament diameter of pi * r^2
// nominal 2.85mm filament -- will be recalculated at StartPrint each time
float PI_R2 = 2.0306;

extern long position[4];

// introduce a short delay before reading file details so director listings are more responsive...
#define FILE_READ_DELAY 100
int file_read_delay_counter = FILE_READ_DELAY;
unsigned long estimatedTime=0;

char temp_history[HISTORY_SIZE];
 char bed_history[HISTORY_SIZE];


// smoothing for the print speed display values
// low pass filter constant, from 0.0 to 1.0 -- Higher numbers mean more smoothing, less responsiveness.
// 0.0 would be completely disabled, 1.0 would ignore any changes
#define LOW_PASS_SMOOTHING 0.9

void calculateSpeeds(float & xy_speed, float & e_smoothed_speed);

// integer square root approximation.  faster
unsigned isqrt(unsigned long val)
	{
	unsigned long temp, g=0, b = 0x8000, bshft = 15;
	do
		{
		if (val >= (temp = (((g << 1) + b)<<bshft--)))
			{
			g += b;
			val -= temp;
			}
		}
		while (b >>= 1);
		return g;
	}




void lcd_clear_cache()
{
    for(uint8_t n=0; n<LCD_CACHE_COUNT; n++)
        LCD_CACHE_ID(n) = 0xFF;
    LCD_DETAIL_CACHE_ID() = 0;
    LCD_CACHE_NR_OF_FILES() = 0xFF;
}
//-----------------------------------------------------------------------------------------------------------------
extern float final_e_position;

static void abortPrint()
{
    /// stop any printing that's in the queue -- either from planner or the serial buffer
    plan_discard_all_blocks();
    clear_command_queue();
    char buffer[32];
    card.sdprinting = false;
	final_e_position = true_e_position + current_position[E_AXIS];

    // set up the end of print retraction
    sprintf_P(buffer, PSTR("G92 E%i"), int(((float)END_OF_PRINT_RETRACTION) / volume_to_filament_length[active_extruder]));
    enquecommand(buffer);
    // perform the retraction at the standard retract speed
    sprintf_P(buffer, PSTR("G1 F%i E0"), int(retract_feedrate));
    enquecommand(buffer);

    enquecommand_P(PSTR("G28"));
    enquecommand_P(PSTR("M84"));
    lcd_lib_beep_ext(220,150);
    postMenuCheck = NULL;
    lifetime_stats_print_end();
    doCooldown();

    stoptime = millis();
}


static void checkPrintFinished()
{
    if (!card.sdprinting && !is_command_queued())
        {
            abortPrint();
            currentMenu = lcd_menu_print_ready;

            SELECT_MAIN_MENU_ITEM(0);
            lcd_lib_beep_ext(440,250);

        }
    if (card.errorCode())
        {
            abortPrint();
            lcd_lib_beep_ext(110,250);
            currentMenu = lcd_menu_print_error;
            SELECT_MAIN_MENU_ITEM(0);
        }
}


static void doStartPrint()
{
    PI_R2 =((PI*((material[0].diameter/2)*(material[0].diameter/2))));
    current_position[E_AXIS] = 0.0;
    plan_set_e_position(0);
    // since we are going to prime the nozzle, forget about any G10/G11 retractions that happened at end of previous print
    retracted = false;
#ifdef RAISE_BED_ON_START
    current_position[Z_AXIS] = 20.0;
#endif
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS], 0);

    for(uint8_t e = 0; e<EXTRUDERS; e++)
        {
            if (!LCD_DETAIL_CACHE_MATERIAL(e))
                {
                    // don't prime the extruder if it isn't used in the (Ulti)gcode
                    // traditional gcode files typically won't have the Material lines at start, so we won't prime for those
                    continue;
                }
            active_extruder = e;


            // undo the end-of-print retraction
            plan_set_e_position((0.0 - END_OF_PRINT_RETRACTION) / volume_to_filament_length[e]);
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], END_OF_PRINT_RECOVERY_SPEED, e);

            // perform additional priming
            plan_set_e_position(-PRIMING_MM3);
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], (PRIMING_MM3_PER_SEC * volume_to_filament_length[e]), e);

            // for extruders other than the first one, perform end of print retraction
#if EXTRUDERS>1
			if (e > 0)
                {
                    plan_set_e_position((END_OF_PRINT_RETRACTION) / volume_to_filament_length[e]);
                    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], retract_feedrate/60, e);
                }
#endif
        }
    active_extruder = 0;

    postMenuCheck = checkPrintFinished;
    lcd_setstatusP(PSTR("HERE WE GO!"));
    lcd_lib_beep_ext(440,100);
    lcd_lib_beep_ext(660,150);
    lcd_lib_beep_ext(880,150);
    card.startFileprint();
    last_user_interaction=starttime = millis();
    stoptime=0;
    lifetime_stats_print_start();
    starttime = millis();
	true_e_position=0;

}

static void cardUpdir()
{
    card.updir();
}

static char* lcd_sd_menu_filename_callback(uint8_t nr)
{
    //This code uses the card.longFilename as buffer to store the filename, to save memory.
    if (nr == 0)
        {
            if (card.atRoot())
                {
                    strcpy_P(card.longFilename, PSTR("< RETURN"));
                }
            else
                {
                    strcpy_P(card.longFilename, PSTR("< BACK"));
                }
        }
    else
        {
            card.longFilename[0] = '\0';
            for(uint8_t idx=0; idx<LCD_CACHE_COUNT; idx++)
                {
                    if (LCD_CACHE_ID(idx) == nr)
                        strcpy(card.longFilename, LCD_CACHE_FILENAME(idx));
                }
            if (card.longFilename[0] == '\0')
                {
                    card.getfilename(nr - 1);
                    if (!card.longFilename[0])
                        strcpy(card.longFilename, card.filename);
                    if (!card.filenameIsDir)
                        {
                            if (strchr(card.longFilename, '.')) strrchr(card.longFilename, '.')[0] = '\0';
                        }

                    uint8_t idx = nr % LCD_CACHE_COUNT;
                    LCD_CACHE_ID(idx) = nr;
                    strcpy(LCD_CACHE_FILENAME(idx), card.longFilename);
                    LCD_CACHE_TYPE(idx) = card.filenameIsDir ? 1 : 0;
                    if (card.errorCode() && card.sdInserted)
                        {
                            //On a read error reset the file position and try to keep going. (not pretty, but these read errors are annoying as hell)
                            card.clearError();
                            LCD_CACHE_ID(idx) = 255;
                            card.longFilename[0] = '\0';
                        }
                }
        }
    return card.longFilename;
}

//-----------------------------------------------------------------------------------------------------------------
void updateFileDetails( uint8_t nr, char * filename);
void lcd_sd_menu_details_callback(uint8_t nr)
{
    if (nr == 0)
        {
            return;
        }
    for(uint8_t idx=0; idx<LCD_CACHE_COUNT; idx++)
        {
            if (LCD_CACHE_ID(idx) == nr)
                {
                    if (LCD_CACHE_TYPE(idx) == 1)
                        {
                            lcd_lib_draw_string_centerP(53, PSTR("Folder"));
                        }
                    else
                        {
                            char buffer[32];


                            if ( LCD_DETAIL_CACHE_ID() != nr)
                                {
                                    if (file_read_delay_counter>0)
                                        file_read_delay_counter --;
                                    if (file_read_delay_counter > 0) return;		// wait, don't read yet, we may just be scanning through the list quickly....
                                    file_read_delay_counter=FILE_READ_DELAY;						// but don't make the wait too long - we don't want them to select a file without having hit this block
                                    card.getfilename(nr - 1);
                                    if (card.errorCode())
                                        {
                                            card.clearError();
                                            return;
                                        }
                                    updateFileDetails(nr, card.filename);
                                }

                            if (LCD_DETAIL_CACHE_TIME() > 0)
                                {
                                    char* c = buffer;
                                    if (led_glow_dir)
                                        {
                                            strcpy_P(c, PSTR("Time: "));
											c += 6;
                                            c = int_to_time_string(LCD_DETAIL_CACHE_TIME(), c);
                                        }
                                    else
                                        {
                                            strcpy_P(c, PSTR("Material: "));
											c += 10;
                                            estimated_filament_length_in_m = float(LCD_DETAIL_CACHE_MATERIAL(0)) / (M_PI * (material[0].diameter / 2.0) * (material[0].diameter / 2.0)) / 1000.0;
                                            if (estimated_filament_length_in_m < 10)
                                                c = float_to_string(estimated_filament_length_in_m , c, PSTR("m"));
                                            else
                                                c = int_to_string(estimated_filament_length_in_m , c, PSTR("m"));
#if EXTRUDERS > 1
                                            if (LCD_DETAIL_CACHE_MATERIAL(1))
                                                {
                                                    *c++ = '/';
                                                    float length = float(LCD_DETAIL_CACHE_MATERIAL(1)) / (M_PI * (material[1].diameter / 2.0) * (material[1].diameter / 2.0));
                                                    if (length < 10000)
                                                        c = float_to_string(length / 1000.0, c, PSTR("m"));
                                                    else
                                                        c = int_to_string(length / 1000.0, c, PSTR("m"));
                                                }
#endif
                                        }
                                    lcd_lib_draw_string(3, 53, buffer);
                                }
                            else
                                {
                                    lcd_lib_draw_stringP(3, 53, PSTR("No info available"));
                                }
                        }
                }
        }
}

void lcd_menu_print_select()
{
    static bool beeped = false;
    if (!card.sdInserted)
        {
            // beep, but only once
            LED_GLOW_ERROR();
            if (!beeped) ERROR_BEEP();
            beeped = true;
            lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(0);
            lcd_info_screen(lcd_menu_main);
            lcd_lib_draw_string_centerP(15, PSTR("No SD-CARD!"));
            lcd_lib_draw_string_centerP(25, PSTR("Please insert card"));
            lcd_lib_update_screen();
            card.release();
            return;
        }
    if (!card.isOk())
        {
            lcd_info_screen(lcd_menu_main);
            lcd_lib_draw_string_centerP(16, PSTR("Reading card..."));
            lcd_lib_update_screen();
            lcd_clear_cache();
            card.initsd();
            return;
        }

    if (LCD_CACHE_NR_OF_FILES() == 0xFF)
        LCD_CACHE_NR_OF_FILES() = card.getnrfilenames();
    if (card.errorCode())
        {
            LED_GLOW_ERROR();
            if (!beeped) ERROR_BEEP();
            beeped = true;
            LCD_CACHE_NR_OF_FILES() = 0xFF;
            return;
        }
    LED_NORMAL();
    beeped = false;
    uint8_t nrOfFiles = LCD_CACHE_NR_OF_FILES();
    if (nrOfFiles == 0)
        {
            if (card.atRoot())
                lcd_info_screen(lcd_menu_main, NULL, PSTR("OK"));
            else
                lcd_info_screen(lcd_menu_print_select, cardUpdir, PSTR("OK"));
            lcd_lib_draw_string_centerP(25, PSTR("No files found!"));
            lcd_lib_update_screen();
            lcd_clear_cache();
            return;
        }

    if (lcd_lib_button_pressed)
        {
            uint8_t selIndex = uint16_t(SELECTED_SCROLL_MENU_ITEM());
            if (selIndex == 0)
                {
                    if (card.atRoot())
                        {
                            lcd_change_to_menu(lcd_menu_main);
                        }
                    else
                        {
                            lcd_clear_cache();
                            lcd_lib_beep();
                            card.updir();
                        }
                }
            else
                {
                    card.getfilename(selIndex - 1);
                    if (!card.filenameIsDir)
                        {
                            char buffer[64];
                            //Start print
                            updateFileDetails(selIndex-1,card.filename);
                            active_extruder = 0;
                            card.openFile(card.filename, true);

                            if (card.isFileOpen() && !is_command_queued())
                                {
                                    if (led_mode == LED_MODE_WHILE_PRINTING || led_mode == LED_MODE_BLINK_ON_DONE)
                                        analogWrite(LED_PIN, 255 * int(led_brightness_level) / 100);
                                    if (!card.longFilename[0])
                                        strcpy(card.longFilename, card.filename);
                                    card.longFilename[20] = '\0';
                                    if (strchr(card.longFilename, '.')) strchr(card.longFilename, '.')[0] = '\0';


                                    card.fgets(buffer, sizeof(buffer));
                                    buffer[sizeof(buffer)-1] = '\0';
                                    while (strlen(buffer) > 0 && buffer[strlen(buffer)-1] < ' ') buffer[strlen(buffer)-1] = '\0';
                                    if (strcmp_P(buffer, PSTR(";FLAVOR:UltiGCode")) != 0)
                                        {
                                            card.fgets(buffer, sizeof(buffer));
                                            buffer[sizeof(buffer)-1] = '\0';
                                            while (strlen(buffer) > 0 && buffer[strlen(buffer)-1] < ' ') buffer[strlen(buffer)-1] = '\0';
                                        }
                                    card.setIndex(0);
                                    if (strcmp_P(buffer, PSTR(";FLAVOR:UltiGCode")) == 0)
                                        {
                                            //New style GCode flavor without start/end code.
                                            // Temperature settings, filament settings, fan settings, start and end-code are machine controlled.
                                            target_temperature_bed = 0;
                                            fanSpeedPercent = 0;
                                            estimatedTime = LCD_DETAIL_CACHE_TIME();
                                            for(uint8_t e=0; e<EXTRUDERS; e++)
                                                {
                                                    if (LCD_DETAIL_CACHE_MATERIAL(e) < 1)
                                                        continue;
                                                    target_temperature[e] = 0;//material[e].temperature;
                                                    target_temperature_bed = max(target_temperature_bed, material[e].bed_temperature);
                                                    fanSpeedPercent = max(fanSpeedPercent, material[0].fan_speed);
                                                    volume_to_filament_length[e] = 1.0 / (M_PI * (material[e].diameter / 2.0) * (material[e].diameter / 2.0));
                                                    extrudemultiply[e] = material[e].flow;
                                                }

                                            fanSpeed = 0;
                                            enquecommand_P(PSTR("G28"));
                                            enquecommand_P(PSTR("G1 F12000 X5 Y10"));
                                            lcd_change_to_menu(lcd_menu_print_heatup);
                                        }
                                    else
                                        {
                                            estimatedTime=1000;
                                            lcd_lib_beep_ext(330,150);
                                            //Classic gcode file
                                            //Set the settings to defaults so the classic GCode has full control
                                            fanSpeedPercent = 100;
                                            for(uint8_t e=0; e<EXTRUDERS; e++)
                                                {
                                                    volume_to_filament_length[e] = 1.0;
                                                    extrudemultiply[e] = 100;
                                                }
                                            lcd_change_to_menu(lcd_menu_print_classic_warning, MAIN_MENU_ITEM_POS(0));
                                        }
                                }
                        }
                    else
                        {
                            lcd_lib_beep();
                            lcd_clear_cache();
                            card.chdir(card.filename);
                            SELECT_SCROLL_MENU_ITEM(0);
                        }
                    return;//Return so we do not continue after changing the directory or selecting a file. The nrOfFiles is invalid at this point.
                }
        }
    lcd_scroll_menu(PSTR("SD CARD"), nrOfFiles+1, lcd_sd_menu_filename_callback, lcd_sd_menu_details_callback);
}

static void lcd_menu_print_heatup()
{
bool draw_filename = true;
if (millis() - last_user_interaction < 5000)
	{
	lcd_question_screen(lcd_menu_print_tune, NULL, PSTR("TUNE"), lcd_menu_print_abort, NULL, PSTR("ABORT"));

	draw_filename = false;
	}
else
	{
	lcd_lib_encoder_pos = ENCODER_NO_SELECTION;
	lcd_lib_clear ();
	//  lcd_basic_screen();
	}
     starttime=stoptime =millis();		// kept the timers paused
    if (current_temperature_bed > target_temperature_bed - 10 && target_temperature_bed > 5)
        {
            for(uint8_t e=0; e<EXTRUDERS; e++)
                {
                    if (LCD_DETAIL_CACHE_MATERIAL(e) < 1 || target_temperature[e] > 0)
                        continue;
                    if (target_temperature[e] != material[e].temperature)
						{
						analogWrite(LED_PIN,0);
						lcd_lib_beep_ext(600,50);
						delay (100);
						lcd_lib_beep_ext(660,50);
						analogWrite(LED_PIN, 255 * led_brightness_level / 100);
						}


                    target_temperature[e] = material[e].temperature;
                }
            if (current_temperature_bed >= target_temperature_bed - TEMP_WINDOW * 2 && !is_command_queued())
                {
                    bool ready = true;
                    for(uint8_t e=0; e<EXTRUDERS; e++)
                        if (current_temperature[e] < target_temperature[e] - TEMP_WINDOW)
                            ready = false;
                    if (ready)
                        {
                            doStartPrint();
                            currentMenu = lcd_menu_print_printing;
                        }
                }
        }

    uint8_t progress = 125;
    for(uint8_t e=0; e<EXTRUDERS; e++)
        {
            if (LCD_DETAIL_CACHE_MATERIAL(e) < 1 || target_temperature[e] < 1)
                continue;
            if (current_temperature[e] > 20)
                progress = min(progress, (current_temperature[e] - 20) * 125 / (target_temperature[e] - 20 - TEMP_WINDOW));
            else
                progress = 0;
        }
    if (target_temperature_bed > BED_MINTEMP) 
		if (current_temperature_bed > 20)							//  what if we're printing without hearing the bed?
	        progress = min(progress, (current_temperature_bed - 20) * 125 / (target_temperature_bed - 20 - TEMP_WINDOW));
	    else
		    progress = 0;

    if (progress < minProgress)
        progress = minProgress;
    else
        minProgress = progress;


    char buffer[25];
    char* c;

    c = int_to_string(current_temperature[0], buffer/*, PSTR( DEGREE_C_SYMBOL )*/);
    *c++ = TEMPERATURE_SEPARATOR;
    c = int_to_string(target_temperature[0], c, PSTR( DEGREE_C_SYMBOL ),true);
    *c++ = ' ';
    *c++ = ' ';
    *c++ = ' ';
    c = int_to_string(current_temperature_bed, c/*, PSTR( DEGREE_C_SYMBOL )*/);
    *c++ = TEMPERATURE_SEPARATOR;
    c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ),true);
	*c++=0;
    lcd_lib_draw_string_center(HALF_ROW(ROW1), buffer);



    drawTempHistory (3,ROW2+3,DISPLAY_RIGHT/2-2,ROW6-3,temp_history);
    drawTempHistory (DISPLAY_RIGHT/2+2,ROW2+3,DISPLAY_RIGHT-3,ROW6-3,bed_history);

	  
	  if (draw_filename) 
		  {
		  c = buffer;
		  c = float_to_string((estimated_filament_length_in_m) ,c,PSTR ("m of "));
		  strcpy (c,material_name[0]);
		  lcd_lib_draw_string_center(ROW6, buffer);

		//  lcd_lib_draw_string_centerP(ROW6, PSTR("Heating up..."));
		  lcd_lib_draw_string_center(ROW7, card.longFilename);
		  lcd_lib_invert(3, ROW7-1, progress, DISPLAY_BOTTOM);
		  } 

//	lcd_progressbar(progress);
    lcd_lib_update_screen();
    LED_HEAT();
}



//-----------------------------------------------------------------------------------------------------------------
// Draws a bargraph of the specified size and location, filled based on the value of 0.0 to 1.0
void drawMiniBargraph( int x1,int y1,int x2,int y2,double value )
{
    lcd_lib_draw_box(x1, y1, x2, y2);
    value = constrain(value,0.0,1.0);
    if (value ==0.0) return;
    int val = value * abs(x2-x1-4);

    lcd_lib_set (x1+2,y1+2,x1+2+val,y2-2);
}




char *  printDigits( byte digits, byte places, char * buffer )
{
    if (digits < 0)
        {
            *buffer++='-';
            digits = -digits;
        }
    unsigned long zeros = 1;
    for (int a=1; a < places; a++)
        zeros*=10;
    while (places-->=0 && digits  < zeros)
        {
            *buffer++='0';
            zeros/=10;
        }
    if (digits!=0)
        buffer = int_to_string(digits,buffer);
    return buffer;

}

//-------------------------------------------------------------------------------------------------------------------
//************************************
// Method:    EchoTimeSpan
// FullName:  EchoTimeSpan
// Access:    public
// Returns:   char* predfined buffer
// Qualifier:
// Parameter: long t  time in seconds
//************************************
// returned string will be in HH:MM:SS (8 chars) or HH:MM (5 chars) format 
char *   EchoTimeSpan (unsigned long t, char * buffer, bool seconds)
{
    unsigned long sec,min,hr;
	if (!seconds) t+=29;	// to round things up
    hr = t/3600L;
    t-=hr*3600L; 
    //if (hr>99) hr=99;
    min=t/60L;
    t-=min*60L;
    min%=60L;

    sec=t%60L;
    if (hr > 24)
        {
            int days =hr / 24;
            hr -= days * (24);
            buffer = int_to_string(days,buffer);
            *buffer++='d';
            *buffer++=' ';
        }
    buffer = printDigits(hr, 2,buffer);
    *buffer++=':';
    buffer = printDigits(min, 2,buffer);
    if (seconds) 
		{ 
		*buffer++=':';
		buffer = printDigits(sec, 2,buffer);
		}
	*buffer=0;
    return buffer;
};
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
static long calculateRemainingTime( unsigned long printTimeMs, long timeLeftSec );
//-----------------------------------------------------------------------------------------------------------------
static char* drawCurrentTemp( char* c );

// ROW 1  = SPEED and FLOW and Z
// ROW2 =  TEMP and bed temp
// ROW3 = GRAPH and FAN
// ROW4 = Graph amd  BUFFER,
// ROW5 =  extrusion volume  and linear speed
// ROW6 = filament and time progress values
// ROW7 = MENU / Progress / MESSAGE OR FILENAME

static void lcd_menu_print_printing()
{
    bool draw_filename = true;
    if (millis() - last_user_interaction < 5000)
        {
            lcd_question_screen(lcd_menu_print_tune, NULL, PSTR("TUNE"), lcd_menu_print_abort, NULL, PSTR("ABORT"));
            draw_filename = false;
        }
    else
        {
            lcd_lib_encoder_pos = ENCODER_NO_SELECTION;
			lcd_lib_clear ();
          //  lcd_basic_screen();
        }
    uint8_t progress = card.getFilePos() / ((card.getFileSize() + 123) / 124);
    char buffer[40];
    char* c;
    switch(printing_state)
        {
            default:
                {
                    LED_NORMAL();
                    // these are used to maintain a simple low-pass filter on the speeds
                    static float e_smoothed_speed = 0.0;
                    static float xy_speed = 0.0;
					calculateSpeeds(xy_speed, e_smoothed_speed);

					
					drawSpeedAndFlow(buffer, c,ROW5);

					// Show the extruder temperature and target temperature:
					c = buffer/* + 3*/;
					c = drawCurrentTemp(c);
					*c++=0;
					lcd_lib_draw_string(10,ROW1, buffer);
                 
					c = buffer /*+ 2*/;	
					c = int_to_string(current_temperature_bed, c, PSTR( TEMPERATURE_SEPARATOR_S ));
					c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ));
					*c++=0;

					lcd_lib_draw_string_right(ROW1, buffer);

                    // show the extrusion rate
                    c=buffer;
					*c=32;
					if (e_smoothed_speed<10.0) c++;
                    c = float_to_string( e_smoothed_speed,c, PSTR ("mm" CUBED_SYMBOL  PER_SECOND_SYMBOL ));
                    *c++=0;
					lcd_lib_draw_string(5,ROW4, buffer);

                    // show the xy travel speed
                    c = buffer;
                    c = int_to_string( round(xy_speed),c,PSTR ("mm" PER_SECOND_SYMBOL ));		// we don't need decimal places here.
                    *c++=0;
					lcd_lib_draw_string_right(ROW4, buffer);

                    // show the fan speed
                    drawMiniBargraph (DISPLAY_RIGHT-(3+2+32),ROW2+1,DISPLAY_RIGHT,ROW3-1,(float) getFanSpeed()/ 255.0);
					strcpy_P(buffer,PSTR ("FAN"));
					lcd_lib_draw_string_right(ROW2+1,buffer,DISPLAY_RIGHT-(3+2+32));
                    drawTempHistory (3,ROW2,DISPLAY_RIGHT/2-2,ROW4-2,temp_history);
// 					lcd_lib_draw_box(DISPLAY_RIGHT/2,ROW2,DISPLAY_RIGHT/2+6,ROW4-2);
// 					lcd_lib_set (DISPLAY_RIGHT/2,ROW4-2 - ((int) (ROW4-ROW2-2) * (int) getHeaterPower(active_extruder)) >> 7,DISPLAY_RIGHT/2+6,ROW4-2);


                    // show the buffer  depth
					strcpy_P(buffer,PSTR ("BUF"));
					lcd_lib_draw_string_right(ROW3,buffer,DISPLAY_RIGHT-(3+2+32));
					drawMiniBargraph (DISPLAY_RIGHT-(3+2+32),ROW3,DISPLAY_RIGHT,ROW4-2,(float) movesplanned() / (BLOCK_BUFFER_SIZE-1));

                    // show pink or red if the movement buffer is low / dry
                    if (movesplanned() < 2)							lcd_lib_led_color(255,0,0);
                    else
                        if (movesplanned() < BLOCK_BUFFER_SIZE/4)	lcd_lib_led_color(255,0,160);
                        else
                            if (movesplanned() < BLOCK_BUFFER_SIZE/2)  lcd_lib_led_color(192,32,192);
                }
                break;
            case PRINT_STATE_WAIT_USER:
                // get the user's attention by flashing the control knob LED, clicking, and disabling the automatic LED lighting dimming
                LED_FLASH();
                if (led_glow == 128) lcd_lib_tick();
                last_user_interaction = millis();
                lcd_lib_encoder_pos = ENCODER_NO_SELECTION;
                lcd_lib_draw_string_centerP(ROW3, PSTR("Press button"));
                // show a message, if we have one.  Otherwise, use a default.
                if (!lcd_lib_show_message (ROW5, false))
                    lcd_lib_draw_string_centerP(ROW4, PSTR("to continue"));
                break;
            case PRINT_STATE_HEATING:
                LED_HEAT();
                lcd_lib_draw_string_centerP(ROW2, PSTR("Heating"));
                c = int_to_string(current_temperature[0], buffer/*, PSTR( DEGREE_C_SYMBOL )*/);
                *c++ = TEMPERATURE_SEPARATOR;
				c = drawCurrentTemp(c);
                lcd_lib_draw_string_center(ROW4, buffer);
                break;
            case PRINT_STATE_HEATING_BED:
                LED_HEAT();
                lcd_lib_draw_string_centerP(ROW2, PSTR("Heating buildplate"));
                c = int_to_string(current_temperature_bed, buffer/*, PSTR( DEGREE_C_SYMBOL )*/);
                *c++ = TEMPERATURE_SEPARATOR;
                c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ));
                lcd_lib_draw_string_center(ROW4, buffer);
                break;
        }

    lcd_lib_draw_hline(3,125,ROW6-1);
    lcd_lib_draw_dotted_hline(3,125,ROW7-1);
	lcd_lib_draw_hline(3,ROW7,DISPLAY_BOTTOM);
	lcd_lib_draw_hline(125,ROW7,DISPLAY_BOTTOM);

	if (draw_filename)
		{	// draw time and filament progress
		static long timeLeftSec =estimatedTime;
		unsigned long printTimeMs = (millis() - starttime);
		timeLeftSec = calculateRemainingTime(printTimeMs, timeLeftSec);
		float pos ;
		if (time_phase0)   
			EchoTimeSpan(round (printTimeMs / 1000),buffer);
		else
			{ 
			pos  = (true_e_position+current_position[E_AXIS]) * volume_to_filament_length[0];
			pos /=1000.0;
			c= float_to_string3(pos,buffer,PSTR("m"));
			*c++=0;
			}

		lcd_lib_draw_string(5,ROW6,buffer);

		if (time_phase0)   
			EchoTimeSpan (timeLeftSec,buffer);
		else   
			{
			c = float_to_string3((estimated_filament_length_in_m) - pos,buffer,PSTR("m"));
			*c++=0;
			}

		lcd_lib_draw_string_right(ROW6,buffer);

					// bottom  row - show any M117 GCODE messages, or if none, then  alternate between time remaining and currently printing file, switch every 2 seconds
				if (!lcd_lib_show_message (ROW7+1))
					{ 
					if (time_phase0) 
						lcd_lib_draw_string_center(ROW7+1, card.longFilename);
					else   
						{
						// show the Z height 
						buffer[0]='Z';
						buffer[1]='=';
						c = buffer + 2;
						c=float_to_string(current_position[Z_AXIS],c,PSTR ("mm"));
						*c++=0;
						lcd_lib_draw_string_center(ROW7, buffer);

						}
					lcd_lib_invert(3, ROW7-1, progress, 63);
					}
				}
    lcd_lib_update_screen();
}

static void lcd_menu_print_error()
{
    LED_GLOW_ERROR();
    ERROR_BEEP();
    lcd_info_screen(lcd_menu_main, NULL, PSTR("RETURN TO MAIN"));

    lcd_lib_draw_string_centerP(10, PSTR("Error while"));
    lcd_lib_draw_string_centerP(20, PSTR("reading"));
    lcd_lib_draw_string_centerP(30, PSTR("SD-card!"));
    char buffer[12];
    strcpy_P(buffer, PSTR("Code:"));
    int_to_string(card.errorCode(), buffer+5);
    lcd_lib_draw_string_center(40, buffer);

    lcd_lib_update_screen();
}

static void lcd_menu_print_classic_warning()
{
    lcd_question_screen(lcd_menu_print_printing, doStartPrint, PSTR("CONTINUE"), lcd_menu_print_select, NULL, PSTR("CANCEL"));

    lcd_lib_draw_string_centerP(10, PSTR("This file will"));
    lcd_lib_draw_string_centerP(20, PSTR("override machine"));
    lcd_lib_draw_string_centerP(30, PSTR("setting with setting"));
    lcd_lib_draw_string_centerP(40, PSTR("from the slicer."));

    lcd_lib_update_screen();
}

static void lcd_menu_print_abort()
{
    LED_FLASH();
    lcd_question_screen(lcd_menu_print_ready, abortPrint, PSTR("YES"), previousMenu, NULL, PSTR("NO"));

    lcd_lib_draw_string_centerP(20, PSTR("Abort the print?"));

    lcd_lib_update_screen();
}

static void postPrintReady()
{
    if (led_mode == LED_MODE_BLINK_ON_DONE)
        analogWrite(LED_PIN, 0);
}
//-----------------------------------------------------------------------------------------------------------------
static void printDoneBeep();
#define SKIP_END_SCREEN 1

static void lcd_menu_print_ready()
{
    if (stoptime ==0) stoptime = millis();
    if (stoptime <starttime) starttime = stoptime;
    last_user_interaction = millis();
    if (led_mode == LED_MODE_WHILE_PRINTING)
        analogWrite(LED_PIN, 0);
    else
        if (led_mode == LED_MODE_BLINK_ON_DONE)
            analogWrite(LED_PIN, (led_glow << 1) * int(led_brightness_level) / 100);


	if (SKIP_END_SCREEN && (millis() - stoptime > 10000))	// ten seconds to let retraction and homing complete
		{
		LED_NORMAL();
		printDoneBeep();
		lcd_change_to_menu(lcd_menu_main);
		return;
	}
    lcd_info_screen(lcd_menu_main, postPrintReady, PSTR("BACK TO MENU"));
    char buffer[24];
    char* c;
    if (time_phase1)
        {
            // Let's show the final print time....
            unsigned long printTimeSec = (stoptime-starttime)/1000;

            strcpy_P(buffer, PSTR("Done in "));
            c =EchoTimeSpan(printTimeSec,buffer+8);
            *c++=0;
            lcd_lib_draw_string_center(10, buffer);
        }
    else
        {
            strcpy_P(buffer, PSTR("Est was "));
            c =EchoTimeSpan(estimatedTime,buffer+8);
            *c++=0;
            lcd_lib_draw_string_center(10, buffer);
        }

    // changed to a comparison with prior state saved and a gap between states to avoid switching back and forth
    // at the trigger point (hysteresis)
    static bool print_is_cool = false;
    if (current_temperature_bed>44 || current_temperature[0] > 64) print_is_cool = false;
    if (current_temperature_bed<40 && current_temperature[0] < 60)
            if (!print_is_cool)
                {
					LED_NORMAL();
                    printDoneBeep();
                    print_is_cool = true;
					
                }
            if (!print_is_cool )
                {
                    LED_COOL();

                    lcd_lib_draw_string_centerP(20, PSTR("Printer cooling down"));

                    int16_t progress = 124 - max ((current_temperature[0] - 60),(current_temperature_bed-40));		// whichever is slowest (usually the bed)
                    if (progress < 0) progress = 0;
                    if (progress > 124) progress = 124;

                    if (progress < minProgress)
                        progress = minProgress;
                    else
                        minProgress = progress;

                    lcd_progressbar(progress);

                    c = buffer;
                    for(uint8_t e=0; e<EXTRUDERS; e++)
                        c = int_to_string(current_temperature[e], buffer, PSTR( DEGREE_C_SYMBOL "   "));
                    int_to_string(current_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ));
                    lcd_lib_draw_string_center(30, buffer);
                }
            else
                {
                    LED_DONE();
                    lcd_lib_draw_string_centerP(20, PSTR("Print finished"));
                    lcd_lib_draw_string_centerP(30, PSTR("You can remove"));
                    lcd_lib_draw_string_center(40, card.longFilename);
                }
            lcd_lib_update_screen();
        }

    static char* tune_item_callback(uint8_t nr)
    {
        char* c = (char*)lcd_cache;
        if (nr == 0)
            strcpy_P(c, PSTR("< RETURN"));
        else
            if (nr == 1)
                {
                    if (!card.pause)
                        {
                            if (movesplanned() > 0)
                                strcpy_P(c, PSTR("Pause"));
                            else
                                strcpy_P(c, PSTR("Can not pause"));
                        }
                    else
                        {
                            if (movesplanned() < 1)
                                strcpy_P(c, PSTR("Resume"));
                            else
                                strcpy_P(c, PSTR("Pausing..."));
                        }
                }
            else
                if (nr == 2)
                    strcpy_P(c, PSTR("Speed"));
                else
                    if (nr == 3)
                        strcpy_P(c, PSTR("Temperature"));
#if EXTRUDERS > 1
                    else
                        if (nr == 4)
                            strcpy_P(c, PSTR("Temperature 2"));
#endif
                        else
                            if (nr == 3 + EXTRUDERS)
                                strcpy_P(c, PSTR("Buildplate temp."));
                            else
                                if (nr == 4 + EXTRUDERS)
                                    strcpy_P(c, PSTR("Fan speed"));
                                else
                                    if (nr == 5 + EXTRUDERS)
                                        strcpy_P(c, PSTR("Material flow"));
#if EXTRUDERS > 1
                                    else
                                        if (nr == 6 + EXTRUDERS)
                                            strcpy_P(c, PSTR("Material flow 2"));
#endif
                                        else
                                            if (nr == 5 + EXTRUDERS * 2)
                                                strcpy_P(c, PSTR("Retraction"));
                                            else
                                                if (nr == 6 + EXTRUDERS * 2)
                                                    strcpy_P(c, PSTR("LED Brightness"));
												else 
													if (nr == 7+ EXTRUDERS * 2)
														strcpy_P(c, PSTR("Home head and resume"));
        return c;
    }

    static void tune_item_details_callback(uint8_t nr)
    {
        char* c = (char*)lcd_cache;
        if (nr == 2)
            c = int_to_string(feedmultiply, c, PSTR("%"));
        else
            if (nr == 3)
                {
                    c = int_to_string(current_temperature[0], c /*,PSTR( DEGREE_C_SYMBOL )*/);
                    *c++ = TEMPERATURE_SEPARATOR;
                    c = int_to_string(target_temperature[0], c, PSTR( DEGREE_C_SYMBOL ));
                }
#if EXTRUDERS > 1
            else
                if (nr == 4)
                    {
                        c = int_to_string(current_temperature[1], c/*, PSTR( DEGREE_C_SYMBOL )*/);
                        *c++ = TEMPERATURE_SEPARATOR;
                        c = int_to_string(target_temperature[1], c, PSTR( DEGREE_C_SYMBOL ));
                    }
#endif
                else
                    if (nr == 3 + EXTRUDERS)
                        {
                            c = int_to_string(current_temperature_bed, c/*, PSTR( DEGREE_C_SYMBOL )*/);
                            *c++ = TEMPERATURE_SEPARATOR;
                            c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ));
                        }
                    else
                        if (nr == 4 + EXTRUDERS)
                            c = int_to_string(int(getFanSpeed()) * 100 / 255, c, PSTR("%"));
                        else
                            if (nr == 5 + EXTRUDERS)
                                c = int_to_string(extrudemultiply[0], c, PSTR("%"));
#if EXTRUDERS > 1
                            else
                                if (nr == 6 + EXTRUDERS)
                                    c = int_to_string(extrudemultiply[1], c, PSTR("%"));
#endif
                                else
                                    if (nr == 7 + EXTRUDERS)
                                        {
                                            c = int_to_string(led_brightness_level, c, PSTR("%"));
                                            if (led_mode == LED_MODE_ALWAYS_ON ||  led_mode == LED_MODE_WHILE_PRINTING || led_mode == LED_MODE_BLINK_ON_DONE)
                                                analogWrite(LED_PIN, 255 * int(led_brightness_level) / 100);
                                        }
                                    else
                                        return;
        lcd_lib_draw_string(5, 53, (char*)lcd_cache);
    }

    void lcd_menu_print_tune_heatup_nozzle0()
    {
        lcd_lib_enable_encoder_acceleration(true);
        if (lcd_lib_encoder_pos /*/ ENCODER_TICKS_PER_SCROLL_MENU_ITEM */!= 0)
            {
                target_temperature[0] = int(target_temperature[0]) + (lcd_lib_encoder_pos /*/ ENCODER_TICKS_PER_SCROLL_MENU_ITEM*/);
                if (target_temperature[0] < 0)
                    target_temperature[0] = 0;
                if (target_temperature[0] > HEATER_0_MAXTEMP - 15)
                    target_temperature[0] = HEATER_0_MAXTEMP - 15;
                lcd_lib_encoder_pos = 0;
            }
        if (lcd_lib_button_pressed)
            lcd_change_to_menu(previousMenu, previousEncoderPos);

        lcd_lib_clear();
        lcd_lib_draw_string_centerP(20, PSTR("Nozzle temperature:"));
        lcd_lib_draw_string_centerP(53, PSTR("Click to return"));
        char buffer[16];
        int_to_string(int(current_temperature[0]), buffer, PSTR(/* DEGREE_C_SYMBOL "" */TEMPERATURE_SEPARATOR_S));
        int_to_string(int(target_temperature[0]), buffer+strlen(buffer), PSTR( DEGREE_C_SYMBOL ));
        lcd_lib_draw_string_center(30, buffer);
        lcd_lib_update_screen();
    }
#if EXTRUDERS > 1
    void lcd_menu_print_tune_heatup_nozzle1()
    {
        if (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM != 0)
            {
                target_temperature[1] = int(target_temperature[1]) + (lcd_lib_encoder_pos / ENCODER_TICKS_PER_SCROLL_MENU_ITEM);
                if (target_temperature[1] < 0)
                    target_temperature[1] = 0;
                if (target_temperature[1] > HEATER_0_MAXTEMP - 15)
                    target_temperature[1] = HEATER_0_MAXTEMP - 15;
                lcd_lib_encoder_pos = 0;
            }
        if (lcd_lib_button_pressed)
            lcd_change_to_menu(previousMenu, previousEncoderPos);

        lcd_lib_clear();
        lcd_lib_draw_string_centerP(20, PSTR("Nozzle2 temperature:"));
        lcd_lib_draw_string_centerP(53, PSTR("Click to return"));
        char buffer[16];
        int_to_string(int(current_temperature[1]), buffer, PSTR( /*DEGREE_C_SYMBOL*/ TEMPERATURE_SEPARATOR));
        int_to_string(int(target_temperature[1]), buffer+strlen(buffer), PSTR( DEGREE_C_SYMBOL ));
        lcd_lib_draw_string_center(30, buffer);
        lcd_lib_update_screen();
    }
#endif
    extern void lcd_menu_maintenance_advanced_bed_heatup();//TODO


	// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    static void lcd_menu_print_tune()
    {
        lcd_scroll_menu(PSTR("TUNE"), 8 + EXTRUDERS * 2, tune_item_callback, tune_item_details_callback);
        if (lcd_lib_button_pressed)
            {
                if (IS_SELECTED_SCROLL(0))
                    {
                        if (card.sdprinting)
                            lcd_change_to_menu(lcd_menu_print_printing);
                        else
                            lcd_change_to_menu(lcd_menu_print_heatup);
                    }
                else
                    if (IS_SELECTED_SCROLL(1) || IS_SELECTED_SCROLL(7 + EXTRUDERS * 2))
                        {
                            if (card.sdprinting)
                                {
                                    if (card.pause)
                                        {
                                            if (movesplanned() < 1)
                                                {
                                                    card.pause = false;
                                                    lcd_lib_beep();
													if (IS_SELECTED_SCROLL(7 + EXTRUDERS * 2))
														enquecommand_P(PSTR("G28 XY"));		// home the head again after a pause, in case it was forcibly moved or is offset
                                                }
                                        }
                                    else
                                        {
                                            if (movesplanned() > 0 && commands_queued() < BUFSIZE)
                                                {
                                                    lcd_lib_beep();
                                                    card.pause = true;
                                                    if (current_position[Z_AXIS] < 170)
                                                        enquecommand_P(PSTR("M601 X10 Y20 Z20 L30"));
                                                    else
                                                        if (current_position[Z_AXIS] < 200)
                                                            enquecommand_P(PSTR("M601 X10 Y20 Z2 L30"));
                                                        else
                                                            enquecommand_P(PSTR("M601 X10 Y20 Z0 L30"));
                                                }
                                        }
                                }
                        }
                    else
                        if (IS_SELECTED_SCROLL(2))
                            LCD_EDIT_SETTING(feedmultiply, "Print speed", "%", 10, 1000);
                        else
                            if (IS_SELECTED_SCROLL(3))
                                lcd_change_to_menu(lcd_menu_print_tune_heatup_nozzle0, 0);
#if EXTRUDERS > 1
                            else
                                if (IS_SELECTED_SCROLL(4))
                                    lcd_change_to_menu(lcd_menu_print_tune_heatup_nozzle1, 0);
#endif
                                else
                                    if (IS_SELECTED_SCROLL(3 + EXTRUDERS))
                                        lcd_change_to_menu(lcd_menu_maintenance_advanced_bed_heatup, 0);//Use the maintainace heatup menu, which shows the current temperature.
                                    else
                                        if (IS_SELECTED_SCROLL(4 + EXTRUDERS))
                                            LCD_EDIT_SETTING_FAN_OVERRIDE(fanSpeedOverride, "Fan speed", "%", 0, 100);
                                        else
                                            if (IS_SELECTED_SCROLL(5 + EXTRUDERS))
                                                LCD_EDIT_SETTING(extrudemultiply[0], "Material flow", "%", 10, 1000);
#if EXTRUDERS > 1
                                            else
                                                if (IS_SELECTED_SCROLL(6 + EXTRUDERS))
                                                    LCD_EDIT_SETTING(extrudemultiply[1], "Material flow 2", "%", 10, 1000);
#endif
                                                else
                                                    if (IS_SELECTED_SCROLL(5 + EXTRUDERS * 2))
                                                        lcd_change_to_menu(lcd_menu_print_tune_retraction);
                                                    else
                                                        if (IS_SELECTED_SCROLL(6 + EXTRUDERS * 2))
                                                            LCD_EDIT_SETTING(led_brightness_level, "Brightness", "%", 0, 100);
															
            }
    }

    static char* lcd_retraction_item(uint8_t nr)
    {
        if (nr == 0)
            strcpy_P((char*)lcd_cache, PSTR("< RETURN"));
        else
            if (nr == 1)
                strcpy_P((char*)lcd_cache, PSTR("Retract length"));
            else
                if (nr == 2)
                    strcpy_P((char*)lcd_cache, PSTR("Retract speed"));
#if EXTRUDERS > 1
                else
                    if (nr == 3)
                        strcpy_P((char*)lcd_cache, PSTR("Extruder change len"));
#endif
                    else
                        strcpy_P((char*)lcd_cache, PSTR("???"));
        return (char*)lcd_cache;
    }

    static void lcd_retraction_details(uint8_t nr)
    {
        char buffer[16];
        if (nr == 0)
            return;
        else
            if(nr == 1)
                float_to_string(retract_length, buffer, PSTR("mm"));
            else
                if(nr == 2)
                    int_to_string(retract_feedrate / 60 + 0.5, buffer, PSTR("mm" PER_SECOND_SYMBOL ));
#if EXTRUDERS > 1
                else
                    if(nr == 3)
                        int_to_string(extruder_swap_retract_length, buffer, PSTR("mm"));
#endif
        lcd_lib_draw_string(5, 53, buffer);
    }

    static void lcd_menu_print_tune_retraction()
    {
        lcd_scroll_menu(PSTR("RETRACTION"), 3 + (EXTRUDERS > 1 ? 1 : 0), lcd_retraction_item, lcd_retraction_details);
        if (lcd_lib_button_pressed)
            {
                lcd_lib_enable_encoder_acceleration(true);
                if (IS_SELECTED_SCROLL(0))
                    lcd_change_to_menu(lcd_menu_print_tune, SCROLL_MENU_ITEM_POS(6));
                else
                    if (IS_SELECTED_SCROLL(1))
                        LCD_EDIT_SETTING_FLOAT01(retract_length, "Retract length", "mm", 0, 50);
                    else
                        if (IS_SELECTED_SCROLL(2))
                            LCD_EDIT_SETTING_SPEED(retract_feedrate, "Retract speed", "mm" PER_SECOND_SYMBOL , 0, max_feedrate[E_AXIS] * 60);
#if EXTRUDERS > 1
                        else
                            if (IS_SELECTED_SCROLL(3))
                                LCD_EDIT_SETTING_FLOAT001(extruder_swap_retract_length, "Extruder change", "mm", 0, 50);
#endif
            }
    }
//-----------------------------------------------------------------------------------------------------------------
    void updateFileDetails( uint8_t nr,char * filename)
    {
        char buffer[64];
        LCD_DETAIL_CACHE_ID() = nr;
        LCD_DETAIL_CACHE_TIME() = 0;
        for(uint8_t e=0; e<EXTRUDERS; e++)
            LCD_DETAIL_CACHE_MATERIAL(e) = 0;
        card.openFile(filename, true);
        if (card.isFileOpen())
            {
                SERIAL_ECHO_START ;



                SERIAL_ECHOPGM("Reading details on ");
                SERIAL_ECHOLN(filename);
                for(uint8_t n=0; n<8; n++)
                    {
                        card.fgets(buffer, sizeof(buffer));
                        buffer[sizeof(buffer)-1] = '\0';
                        while (strlen(buffer) > 0 && buffer[strlen(buffer)-1] < ' ') buffer[strlen(buffer)-1] = '\0';
                        if (strncmp_P(buffer, PSTR(";TIME:"), 6) == 0)
                            LCD_DETAIL_CACHE_TIME() = atol(buffer + 6);
                        else
                            if (strncmp_P(buffer, PSTR(";MATERIAL:"), 10) == 0)
								{
                                LCD_DETAIL_CACHE_MATERIAL(0) = atol(buffer + 10);
								estimated_filament_length_in_m = float(LCD_DETAIL_CACHE_MATERIAL(0)) / (M_PI * (material[0].diameter / 2.0) * (material[0].diameter / 2.0)) / 1000.0; 
								}
#if EXTRUDERS > 1
                            else
                                if (strncmp_P(buffer, PSTR(";MATERIAL2:"), 11) == 0)
                                    LCD_DETAIL_CACHE_MATERIAL(1) = atol(buffer + 11);
#endif
                    }
            }
        if (card.errorCode())
            {
                //On a read error reset the file position and try to keep going. (not pretty, but these read errors are annoying as hell)
                card.clearError();
                LCD_DETAIL_CACHE_ID() = 255;
            }
        card.closefile();
    }
	//-----------------------------------------------------------------------------------------------------------------
	void printDoneBeep()
		{
		analogWrite(LED_PIN, 0);
		lcd_lib_beep_ext(480,50);
		delay (100);
		lcd_lib_beep_ext(440,90);
		analogWrite(LED_PIN, 255 * led_brightness_level / 100);
		}
	//-----------------------------------------------------------------------------------------------------------------
	char* drawSpeedAndFlow( char * buffer, char* c , byte y)
		{
		buffer[0] ='S';
 		buffer[1] ='P';
 		buffer[2] ='D';
 		buffer[3] =':';
		c = int_to_string(feedmultiply,buffer+4,PSTR("%"));
// 		*c++='%';
		*c++=0;
		lcd_lib_draw_string_right(y, buffer);

		buffer[0] ='F';
		buffer[1] ='L';
		buffer[2] ='O';
		buffer[3] =':';

		c = int_to_string(extrudemultiply[0],buffer+4,PSTR("%"));
// 		*c++='%';
		*c++=0;
		lcd_lib_draw_string(5,y, buffer);

		return c;
		}
	//-----------------------------------------------------------------------------------------------------------------
	long calculateRemainingTime( unsigned long printTimeMs, long timeLeftSec )
		{
		long printTimeSec = printTimeMs / 1000L;
		float percent_done = min(1.0,float(card.getFilePos()) / float(card.getFileSize()));
		unsigned long totalTimeMs = float(printTimeMs) / percent_done;
		static float totalTimeSmoothSec=estimatedTime;
		totalTimeSmoothSec *= 0.95;
		totalTimeSmoothSec += (totalTimeMs/1000) / 20;
		if (isinf(totalTimeSmoothSec))
			totalTimeSmoothSec = totalTimeMs;
		long totalTimeSec;

		// take a linear weighted balance between original estimate and actual time -- at the start, we trust the estimate more.  At the end, we trust the elapsed and percentage calculattion
		totalTimeSec =( (1.0 - percent_done) * estimatedTime) + (totalTimeSmoothSec*percent_done);
		//
		// 			if (printTimeSec < LCD_DETAIL_CACHE_TIME() / 2)
		// 				{
		// 				float f = float(printTimeSec) / max(1.0,float(estimatedTime / 2));
		// 				totalTimeSec = float(totalTimeSmoothSec) * f + float(estimatedTime) * (1 - f);
		// 				}
		// 			else
		// 				{
		// 					totalTimeSec = totalTimeSmoothSec;
		// 		}
		timeLeftSec = max(0l, totalTimeSec - printTimeSec);		// avoid negative time...	return timeLeftSec;
		}
	//-----------------------------------------------------------------------------------------------------------------
	char* drawCurrentTemp( char* c )
		{
		c = int_to_string(current_temperature[0], c, PSTR(TEMPERATURE_SEPARATOR_S));
		c = int_to_string(target_temperature[0], c, PSTR( DEGREE_C_SYMBOL "  "),true);	
		
		return c;
		}
	//-----------------------------------------------------------------------------------------------------------------
	void calculateSpeeds(float & xy_speed, float & e_smoothed_speed)
		{
		if (current_block!=NULL)
			{
			if (current_block->steps_x != 0 ||  current_block->steps_y != 0)
				{
				// we only want to track movements that have some xy component

				if (current_block->speed_e >= 0 && current_block->speed_e < retract_feedrate)
					// calculate live extrusion rate from e speed and filament area
						e_smoothed_speed = (e_smoothed_speed*LOW_PASS_SMOOTHING) + ( PI_R2 * current_block->speed_e *(1.0-LOW_PASS_SMOOTHING));

				xy_speed = (LOW_PASS_SMOOTHING*xy_speed) + (((1.0-LOW_PASS_SMOOTHING)) * isqrt ((unsigned long) (current_block->speed_x*current_block->speed_x)+(current_block->speed_y*current_block->speed_y)));
				// might want to replace that sqrt with a fast approximation, since accuracy isn't that important here.
				// or likely in the planner we've already got sqrt(dX*dX + dY*dY) .....somewhere....
				// idea: consider reading XY position and delta time to calculate actual movement rather than what the planner is giving us.
				//		 would that be more useful? It might lose some motion in fast zig zag or cut corners, so it would likely under-report
				//		 but I'm not certain the planner updates the current speed values with acceleration reduction -- so it may be over-reporting
				}
			else
				{
				// zero XY movement...
				xy_speed *= LOW_PASS_SMOOTHING;		// equivalent to above, but since sqrt(0) we can drop that term and calculate less
				if (current_block->steps_z == 0)		// no z-steps, must be a retract -- flash the RGB LED to show we're retracting
					lcd_lib_led_color(8,32,128);
				}
			} //  current_block !=null
		else //    it is null,
			{
			// no current block -- we're paused, buffer has run out, ISR has not yet advanced to the next block, or we're not printing
			xy_speed *= LOW_PASS_SMOOTHING;
			e_smoothed_speed *= LOW_PASS_SMOOTHING;
			}//  current_block ==null
		}

	extern byte history_position;


void drawTempHistory(byte x1, byte y1, byte x2, byte y2, char *data)
		{
		int height = y2 - y1;
		int width  = x2 - x1;
		int mid_point = (y2+y1)>>1;
		lcd_lib_draw_box(x1,y1,x2,y2);
		lcd_lib_draw_dotted_hline (x1,x2,mid_point);
		byte x = x2;
		int prev_offset = mid_point;
		int b = history_position-1;
		for (byte a=0;a<x2-x1;a++)
			{
			if (b<0) b += HISTORY_SIZE;

			int offset = data[b];
			if (offset < 120 &&  offset  > -120)
				{
					offset+= mid_point;
					if (offset < y1) offset = y1;
					if (offset > y2) offset = y2;
					lcd_lib_draw_vline(x,prev_offset,offset);
					prev_offset = offset;
					x--;
					b--;
				}
			}
		};


#endif//ENABLE_ULTILCD2


	//-----------------------------------------------------------------------------------------------------------------


