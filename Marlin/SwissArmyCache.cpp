#include "SwissArmyCache.h"
#include "Marlin.h"

#define DEBUG_INFO

#pragma GCC diagnostic ignored "-Wswitch" // Code that causes warning goes here #pragma GCC diagnostic pop


//-----------------------------------------------------------------------------------------------------------------

LCDCACHE_FILEDETAIL filedetail;


LCD_CACHE lcd_cache_new;
extern byte history_position;

LCD_CACHE_SHARED_DATA & LCD_CACHE::getData( ACCESS_MODE mode )
	{
	if (mode ==  current_mode) return c_data;
	if (current_mode!=MAX_ACCESS_MODE) verify();
	clear();
	current_mode = mode;

#ifdef DEBUG_INFO
	SERIAL_ECHO_START ;
	SERIAL_ECHOPGM(("SWITCHING CACHE MODE TO "));
	SERIAL_ECHOLN (mode);
#endif
	// do any other init when the cache switches context
	switch (current_mode)
		{
		case RAWSTRING: c_data.rawstring[0] = 0; break;
		case TEMPERATURE_HISTORY: history_position = 0; break;
			
		case FILELIST:
			for (int a=0; a<LCD_CACHE_MAX_FILES_CACHED; a++)
				LCD_CACHE_FILENAME(a)[0]=0;
			break;
		}
	verify();

	return c_data;
	}

//-----------------------------------------------------------------------------------------------------------------
void LCD_CACHE::clear (byte value)
	{
	//     for (int a=0; a<sizeof(c_data); a++)
	//         c_data.rawbytes[a] = value;

	memset ((void *) &c_data.rawbytes[0], 0xff, sizeof (c_data));

	}



//-----------------------------------------------------------------------------------------------------------------
void LCD_CACHE::init()
	{
	current_mode = NO_MODE;
	overwrite_check2=MAGIC_TOKEN;
	overwrite_check=MAGIC_TOKEN;
	clear();
	}

//-----------------------------------------------------------------------------------------------------------------
// does some sanity checking in debug mode to make sure nothing was accidentallty overwritten
bool LCD_CACHE::verify()
	{
#ifndef DEBUG_INFO
	return true;
#endif
	bool ok = true;
	switch (current_mode)
		{
		case RAWSTRING:
			if (strlen(c_data.rawstring) > sizeof (c_data) )
				{
				SERIAL_ECHO_START ;
				SERIAL_ECHOPGM(("String mode overrun: "));
				c_data.rawstring[20] = 0;
				SERIAL_ECHOLN((c_data.rawstring));
				ok = false;
				}
			break;
		case FILELIST:
			for (int a=0; a<LCD_CACHE_MAX_FILES_CACHED; a++)
				if (LCD_CACHE_FILENAME(a)[MAX_DISPLAY_FILENAME_LEN+1]!=(char) 0xff)
					{
					SERIAL_ECHO_START ;
					SERIAL_ECHOPGM(("Filename overrun on "));
					SERIAL_ECHO(a);
					SERIAL_ECHO(" ");
					SERIAL_ECHO(LCD_CACHE_FILENAME(a));
					SERIAL_ECHO("=");
					SERIAL_ECHOLN((unsigned int) LCD_CACHE_FILENAME(a)[MAX_DISPLAY_FILENAME_LEN+1]);
					ok = false;
					}
				break;
		}
	if ((overwrite_check==MAGIC_TOKEN) && (overwrite_check2==MAGIC_TOKEN)) return ok;
	SERIAL_ECHO_START ;
	SERIAL_ECHOPGM(("Overwrite! "));
	SERIAL_ECHOLN ( sizeof (c_data));
	delay(50);
	overwrite_check=MAGIC_TOKEN;
	overwrite_check2=MAGIC_TOKEN;
	return false;
	}




