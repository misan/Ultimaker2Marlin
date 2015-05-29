#include "ScreenSaver.h"
#include "SwissArmyCache.h"
#include "UltiLCD2_low_lib.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2.h"

#pragma GCC diagnostic ignored "-Wunused-value"  // bogus warning about constrain



//-----------------------------------------------------------------------------------------------------------------
void ball::init()
	{
	 pos_x = LCD_GFX_WIDTH/2;
	 pos_y = LCD_GFX_HEIGHT/2;
	 dx = (random(-SS_MAX_SPED*100,SS_MAX_SPED*100)) / 100.0;
	 dy = (random(-SS_MAX_SPED*100,SS_MAX_SPED*100)) / 100.0;
	 ax = (random(-SS_MAX_ACC*1000,SS_MAX_ACC*1000)) / 1000.0;
	 ay = (random(-SS_MAX_ACC*1000,SS_MAX_ACC*1000)) / 1000.0;
	}

//-----------------------------------------------------------------------------------------------------------------
void ball::update()
	{
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
void lcd_menu_breakout()
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
//----
