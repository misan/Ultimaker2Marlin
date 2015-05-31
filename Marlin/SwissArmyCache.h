#pragma once

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
//Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif
#include "Configuration.h"
#include "ScreenSaver.h"


#ifdef  __AVR_ATmega2560__
#define LCD_CACHE_MAX_FILES_CACHED 12		// we can't show more than 6, but having a bigger buffer makes scrolling through menus faster
#else
#define LCD_CACHE_MAX_FILES_CACHED 6
#endif



#define MAX_DISPLAY_FILENAME_LEN 25


struct LCDCACHE_FILEINFO
	{
	byte is_folder:1;
	byte id;
	char filename[MAX_DISPLAY_FILENAME_LEN+2];
	};

struct LCDCACHE_FILEDETAIL
	{
	byte id;
	long estimated_print_time;
	long material[EXTRUDERS];
	unsigned int total_layers;
	unsigned int timestamp;
	unsigned int datestamp;
	};

struct LCDCACHE_FILELIST
	{
	byte number_of_entries;
	LCDCACHE_FILEINFO fileinfo[LCD_CACHE_MAX_FILES_CACHED];
	};

// the biggest of the union elements -- just so we have some idea of our byte arrays in other forms
#define CACHE_SIZE ((LCD_CACHE_MAX_FILES_CACHED*sizeof (LCDCACHE_FILEINFO)) )

struct LCDCACHE_BREAKOUT
	{
	int ball_x;
	int ball_y;
	int ball_dx;
	int ball_dy;
	byte brick_data[CACHE_SIZE-16];
	};




struct LCDCACHE_SCREENSAVER
	{
		ball balls[SCREENSAVER_BALLS];
	};

#define HISTORY_SIZE (64)


struct LCD_CACHE_TEMP_HISTORY
	{

	char temp_history[HISTORY_SIZE];
	char bed_history[HISTORY_SIZE];
	};

union LCD_CACHE_SHARED_DATA
	{
	char rawstring[CACHE_SIZE+5];
	byte rawbytes[CACHE_SIZE+5];
	LCDCACHE_FILELIST filelist;
	LCDCACHE_BREAKOUT breakout;
	LCD_CACHE_TEMP_HISTORY temphist;
	LCDCACHE_SCREENSAVER ss;
	};



#define MAGIC_TOKEN 8675309

class LCD_CACHE
	{

	public:
		enum ACCESS_MODE
			{
			NO_MODE,
			FILELIST,
			BREAKOUT,
			RAWSTRING,
			RAWDATA,
			TEMPERATURE_HISTORY,
			SCREENSAVER,
			MAX_ACCESS_MODE
			};

		LCD_CACHE_SHARED_DATA & getData(ACCESS_MODE mode) ;
		void clear (byte value = 0xff);
		void  init();
		LCD_CACHE() { current_mode= MAX_ACCESS_MODE;};
		bool verify ();
		ACCESS_MODE getMode () { return current_mode;};
	private:
		ACCESS_MODE current_mode;
		unsigned long overwrite_check;
		LCD_CACHE_SHARED_DATA c_data;
		unsigned long overwrite_check2;
	} ;
extern LCD_CACHE lcd_cache_new;
extern LCDCACHE_FILEDETAIL filedetail;


#define LCD_CACHE_FI(n)  lcd_cache_new.getData(LCD_CACHE::FILELIST).filelist.fileinfo[n]
#define LCD_CACHE_DETAIL /*lcd_cache_new.getData(LCD_CACHE::FILELIST).filelist.*/filedetail

#define LCD_CACHE_NR_OF_FILES()		 lcd_cache_new.getData(LCD_CACHE::FILELIST).filelist.number_of_entries
#define LCD_CACHE_ID(n)				 LCD_CACHE_FI(n).id
#define LCD_CACHE_TYPE(n)			 LCD_CACHE_FI(n).is_folder
#define LCD_CACHE_FILENAME(n)		 LCD_CACHE_FI(n).filename
#define LCD_DETAIL_CACHE_MATERIAL(n) LCD_CACHE_DETAIL.material[n]
#define LCD_DETAIL_CACHE_TIME()		 LCD_CACHE_DETAIL.estimated_print_time
#define LCD_DETAIL_CACHE_ID()		 LCD_CACHE_DETAIL.id


