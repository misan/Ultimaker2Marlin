#pragma once


#include "Marlin.h"
#include "UltiLCD2_low_lib.h"
#include "UltiLCD2.h"

// smoothing for the print speed display values
// low pass filter constant, from 0.0 to 1.0 -- Higher numbers mean more smoothing, less responsiveness.
// 0.0 would be completely disabled, 1.0 would ignore any changes
#define LOW_PASS_SMOOTHING 0.9

FORCE_INLINE void ERROR_BEEP() 
	{
	lcd_lib_beep_ext(110,400);
	}

//  filament diameter of pi * r^2
// nominal 2.85mm filament -- will be recalculated at StartPrint each time
extern float PI_R2;

//-----------------------------------------------------------------------------------------------------------------
void clearHistory();
void togglePausePrinting();
//-----------------------------------------------------------------------------------------------------------------
extern bool did_beep;
void Nudge( unsigned char axis, float offset );
void lcd_setstatus(const char* message);
void lcd_setstatusP(ppstr message);

#define DEFAULT_MESSAGE_DURATION 500
#define MAX_MESSAGE_LEN 20
extern char message_string [MAX_MESSAGE_LEN+1];
extern int message_counter ;

extern bool run_history ;


FORCE_INLINE bool is_message_shown () { return message_counter > 0; };
FORCE_INLINE void clear_message() { message_counter = 0; message_string[0] = 0;};

#define LCD_MESSAGEPGM(x) {lcd_setstatusP(x);lcd_lib_update_screen();}
#define LCD_ALERTMESSAGEPGM(x) { lcd_setstatusP(x); lcd_lib_update_screen(); lcd_lib_beep_ext(200,500);}; 

extern unsigned long estimatedTime;

// these are used by the menus to time slot fields
#define time_phase1 ((millis() >> 11) & 1)
#define time_phase0 (!time_phase1)

#define time_phase_a(spd)  (((millis() >> (15-spd)) & 3)==0)
#define time_phase_b(spd)  (((millis() >> (15-spd)) & 3)==1)
#define time_phase_c(spd)  (((millis() >> (15-spd)) & 3)==2)
#define time_phase_d(spd)  (((millis() >> (15-spd)) & 3)==3)
#define ROW_HEIGHT 9
#define ROW1 1
#define ROW2 (ROW1+ROW_HEIGHT)		// 9
#define ROW3 (ROW2+ROW_HEIGHT)		// 18
#define ROW4 (ROW3+ROW_HEIGHT)		//27
#define ROW5 (ROW4+ROW_HEIGHT)		// 36
#define ROW6 (ROW5+ROW_HEIGHT)		// 45
#define ROW7 (ROW6+ROW_HEIGHT)		// 54
#define DISPLAY_BOTTOM 63
#define DISPLAY_RIGHT 127
#define HALF_ROW(a) (a+ROW_HEIGHT/2)
#define CHARS_PER_ROW (DISPLAY_RIGHT/6)

 

extern unsigned char LED_DIM_TIME;
bool lcd_lib_show_message(int position, bool decrement = true);
void forceMessage ();


//-----------------------------------------------------------------------------------------------------------------
void Nudge( unsigned char axis, float offset );


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawTempHistory(byte x1, byte y1, byte x2, byte y2, char *data);;


//-----------------------------------------------------------------------------------------------------------------
void printDoneBeep();

//-----------------------------------------------------------------------------------------------------------------
char* drawSpeedAndFlow( char * buffer, char* c , byte y);

//-----------------------------------------------------------------------------------------------------------------
long calculateRemainingTime( unsigned long printTimeMs, long  timeLeftSec );

//-----------------------------------------------------------------------------------------------------------------
char* drawCurrentTemp( char* c , byte e=active_extruder);

//-----------------------------------------------------------------------------------------------------------------
char* drawCurrentBedTemp( char* c );

//-----------------------------------------------------------------------------------------------------------------
void calculateSpeeds(float & xy_speed, float & e_smoothed_speed);

extern byte history_position;



//-----------------------------------------------------------------------------------------------------------------

void togglePausePrinting(bool home_first=false);


//-----------------------------------------------------------------------------------------------------------------
// Draws a bargraph of the specified size and location, filled based on the value of 0.0 to 1.0
void drawMiniBargraph( int x1,int y1,int x2,int y2,double value );




//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
char *  printDigits( byte digits, byte places, char * buffer );

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
char *   EchoTimeSpan (unsigned long t, char * buffer, bool seconds = true);;
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
long calculateRemainingTime( unsigned long printTimeMs, long  timeLeftSec );
//-----------------------------------------------------------------------------------------------------------------

