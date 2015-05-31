#include "MenuUseful.h"
#include "UltiLCD2_low_lib.h"
#include "UltiLCD2.h"
#include "temperature.h"
#include "cardreader.h"
#include "gcode.h"
#include "stringHelpers.h"
#include "UltiLCD2_menu_print.h"

unsigned long estimatedTime=0;

byte history_position =0;
bool did_beep = false;
float PI_R2 = 2.0306;
bool run_history = false;

unsigned char  LED_DIM_TIME = 30;
// 30 min

//-----------------------------------------------------------------------------------------------------------------
void updateTempHistory()
{
    if (!run_history) return;
	history_position %=HISTORY_SIZE ;
	lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.temp_history[history_position] = constrain((int) target_temperature[active_extruder]	- (int)  current_temperature[active_extruder], (int) -127,(int) 127);
    lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.bed_history[history_position]  = constrain((int) target_temperature_bed					- (int) current_temperature_bed				 , (int) -127,(int) 127);
    history_position++;
}

//-----------------------------------------------------------------------------------------------------------------
void clearHistory()
{
    history_position=0;
    memset (lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.bed_history,-127,sizeof (lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.bed_history));
    memset (lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.temp_history,-127,sizeof (lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.temp_history));
}


//-----------------------------------------------------------------------------------------------------------------
// display a message, if we have one, at a given Y coord, and decrease it's counter.
// returns TRUE if it displayed a message
bool lcd_lib_show_message(int position, bool decrement)
{
    if (message_string[0]==0) message_counter =0;
    if (message_counter > 0)
        {
            if (decrement) message_counter --;
            lcd_lib_draw_string_center(position, message_string);
        }
    return message_counter>0;
}
//-----------------------------------------------------------------------------------------------------------------
// does NOT immediately show the message, just sets it for the next update
void lcd_setstatus( const char* message )
{
    serialScreenShown=false;
    message_counter = DEFAULT_MESSAGE_DURATION;
    strncpy (message_string, message,MAX_MESSAGE_LEN);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM("LCD: " );
    SERIAL_ECHOLN(message_string);
}
//-----------------------------------------------------------------------------------------------------------------
//// does NOT immediately show the message, just sets it for the next update
void lcd_setstatusP( ppstr message )
{
    serialScreenShown=false;
    message_counter = DEFAULT_MESSAGE_DURATION;
    strncpy_P (message_string, message,MAX_MESSAGE_LEN);
    // 	SERIAL_ECHO_START;
    // 	SERIAL_ECHOPGM("LCD: " );
    // 	SERIAL_ECHOLNPGM(message_string);
}

//-----------------------------------------------------------------------------------------------------------------
// forces drawing of the status string, advancing to the next line each time
// and clearing the screen at 0  -- simple "log view"
void forceMessage ()
{
    static int position =0;
    if (position == 0 )
        lcd_lib_clear();
    lcd_lib_draw_string_center(position*10, message_string);
    position++;
    if (position>=6) position = 0;
    lcd_lib_update_screen();
    while (!lcd_lib_update_ready()) delay(20);
}

//-----------------------------------------------------------------------------------------------------------------
void drawTempHistory( byte x1, byte y1, byte x2, byte y2, char *data )
{
//     int height = y2 - y1;
//     int width  = x2 - x1;
    int mid_point = (y2+y1)>>1;
    lcd_lib_draw_box(x1,y1,x2,y2);
    lcd_lib_draw_dotted_hline (x1,x2,mid_point);
    byte x = x2;
    int prev_offset = mid_point;
    int b = history_position-1;
    for (byte a=0; a<x2-x1; a++)
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
}


//-----------------------------------------------------------------------------------------------------------------
// fpbbsdnsi
// floating_point_buffer_because_sprintf_does_not_support_it
void Nudge( unsigned char axis, float offset )
{
    {
        char fpbbsdnsi[8];
        float z = current_position[axis];
        float_to_string(  z+offset,fpbbsdnsi);
        char buffer[24];
        memset (buffer,0,sizeof(buffer));

        sprintf_P(buffer, PSTR("G92 %c%s"),axis_codes[axis],fpbbsdnsi);
        while (commands_queued() > BUFSIZE-3) manageBuffer();
        enquecommand( buffer);
        float_to_string(z,fpbbsdnsi);
        float f1 = homing_feedrate[X_AXIS];
        if (axis < 2) f1 = retract_feedrate;

        int f = (int) f1;
        sprintf_P(buffer, PSTR("G1 %c%s F%i"),axis_codes[axis],fpbbsdnsi,f);
        enquecommand( buffer);
    }
}

//-----------------------------------------------------------------------------------------------------------------
void togglePausePrinting(bool home_first)
{
    if (!card.sdprinting) return;
    if (card.pause)
        {
            if (movesplanned() < 1)
                {
                    lcd_lib_beep();
                    card.pause = false;
                    if (home_first)
                        {
                            while (commands_queued() > BUFSIZE-3) manageBuffer();
                            enquecommand_P(PSTR("G28 X0 Y0"));		// home the head again after a pause, in case it was forcibly moved or is offset
                        }
                    enquecommand_P(PSTR("M603 L30"));	// UNpause
                }
        }
    else
        {
            while (commands_queued() > BUFSIZE-3) manageBuffer();		// we're gonna want some room in the command buffer to queue up a command or two
            lcd_lib_beep();
            card.pause = true;
            char buffer[32];
            memset (buffer,0,sizeof(buffer));
            sprintf_P(buffer, PSTR("M602 X%i Y%i Z%i L30"),PAUSE_X_POS,PAUSE_Y_POS,PAUSE_HEIGHT_DROP);
            enquecommand (buffer);
        }
}




//-----------------------------------------------------------------------------------------------------------------
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



//-----------------------------------------------------------------------------------------------------------------
void calculateSpeeds( float & xy_speed, float & e_smoothed_speed )
{
    if (e_smoothed_speed < 0) e_smoothed_speed =0;
    if (xy_speed < 0 ) xy_speed = 0;
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
//-----------------------------------------------------------------------------------------------------------------
char* drawCurrentBedTemp( char* c )
{
    c = int_to_string(current_temperature_bed, c, PSTR(TEMPERATURE_SEPARATOR_S));
    c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL "  "),true);

    return c;
}
//-----------------------------------------------------------------------------------------------------------------
char* drawCurrentTemp( char* c , byte e )
{
    c = int_to_string(current_temperature[e], c, PSTR(TEMPERATURE_SEPARATOR_S));
    c = int_to_string(target_temperature[e], c, PSTR( DEGREE_C_SYMBOL "  "),true);

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
    return  max(0l, totalTimeSec - printTimeSec);		// avoid negative time...	return timeLeftSec;
}
//-----------------------------------------------------------------------------------------------------------------
char* drawSpeedAndFlow( char * buffer, char* c , byte y )
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
    buffer[3] ='W';
    buffer[4] =':';

    c = int_to_string(extrudemultiply[active_extruder],buffer+5,PSTR("%"));
    // 		*c++='%';
    *c++=0;
    lcd_lib_draw_string(5,y, buffer);

    return c;
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
// t in seconds
char * EchoTimeSpan( unsigned long t, char * buffer, bool seconds )
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
}

//-----------------------------------------------------------------------------------------------------------------
char * printDigits( byte digits, byte places, char * buffer )
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
//-----------------------------------------------------------------------------------------------------------------
void drawMiniBargraph( int x1,int y1,int x2,int y2,double value )
{
    lcd_lib_draw_box(x1, y1, x2, y2);
    value = constrain(value,0.0,1.0);
    if (value ==0.0) return;
    int val = value * abs(x2-x1-4);

    lcd_lib_set (x1+2,y1+2,x1+2+val,y2-2);
}

//-----------------------------------------------------------------------------------------------------------------
void processLongFilename()
{
    if (! card.longFilename[0])		// is it non zerto length? copy iy to our buffer
        strcpy (card.longFilename,card.filename);
    if (!card.filenameIsDir)				// end the filename at the FIRST period -- should be the LAST period.
        {
            if (strrchr(card.longFilename, '.')) strrchr(card.longFilename, '.')[0] = '\0';
        }

    // for names longer than 20 chars, put an elipsis and show the last 4 characters because often it's important to see the END of the filename to differentiate them
    // for example:  THISISAVERYLONGFILENAME_VER1.GCO and THISISAVERYLONGFILENAME_VER2.GCO
    // we need to see the VER1 and VER2 part of the name

    /*
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM ("FILE_ENTRY: ")
    SERIAL_ECHO(card.longFilename);
    SERIAL_ECHOPGM (" 8.3: ");
    SERIAL_ECHO(card.filename);
    SERIAL_ECHOLNPGM (" ");*/

    byte x = strlen(card.longFilename);
    if (x > MAX_DISPLAY_FILENAME_LEN)
        {
            card.longFilename[MAX_DISPLAY_FILENAME_LEN-4] = card.longFilename[x-4];
            card.longFilename[MAX_DISPLAY_FILENAME_LEN-3] = card.longFilename[x-3];
            card.longFilename[MAX_DISPLAY_FILENAME_LEN-2] = card.longFilename[x-2];
            card.longFilename[MAX_DISPLAY_FILENAME_LEN-1] = card.longFilename[x-1];
            card.longFilename[MAX_DISPLAY_FILENAME_LEN-5] = ELIPSIS_SYMBOL;// ELIPSIS_SYMBOL;
        }
    card.longFilename[MAX_DISPLAY_FILENAME_LEN] = '\0';

    /*SERIAL_ECHOPGM ("TRIMMED NAME: ")
    SERIAL_ECHO(card.longFilename);
    SERIAL_ECHOLNPGM (" ");
    */
}