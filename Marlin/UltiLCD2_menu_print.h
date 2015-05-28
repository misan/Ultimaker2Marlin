#ifndef ULTI_LCD2_MENU_PRINT_H
#define ULTI_LCD2_MENU_PRINT_H

#include "cardreader.h"

#ifdef  __AVR_ATmega2560__
#define LCD_CACHE_MAX_FILES_CACHED 12		// we can't show more than 6, but having a bigger buffer makes scrolling through menus faster
#else
#define LCD_CACHE_MAX_FILES_CACHED 6
#endif

#define MAX_DISPLAY_FILENAME_LEN 21


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
    LCDCACHE_FILEDETAIL filedetail;

};

#define CACHE_SIZE ((LCD_CACHE_MAX_FILES_CACHED*sizeof (LCDCACHE_FILEINFO)) + sizeof (LCDCACHE_FILEDETAIL))

struct LCDCACHE_BREAKOUT
{
    int ball_x;
    int ball_y;
    int ball_dx;
    int ball_dy;
    byte brick_data[CACHE_SIZE-16];
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


//#define LCD_DETAIL_CACHE_SIZE (5+4*EXTRUDERS)
//#define LCD_CACHE_SIZE (1 + (2 + LONG_FILENAME_LENGTH) * LCD_CACHE_MAX_FILES_CACHED + LCD_DETAIL_CACHE_SIZE)

// extern uint8_t lcd_cache[LCD_CACHE_SIZE];

void lcd_sd_filemenu_doAction();

void doCancelPrint();
void lcd_menu_maintenance_advanced_bed_heatup();

//-----------------------------------------------------------------------------------------------------------------
extern char last_print_name[LONG_FILENAME_LENGTH];
extern unsigned char nozzle_adjust_id;
void lcd_menu_print_tune_heatup_nozzle();
void lcd_menu_retraction_getDetails(uint8_t nr);
char* lcd_menu_retraction_getString(uint8_t nr);
void lcd_menu_retraction_doAction();
#endif//ULTI_LCD2_MENU_PRINT_H
