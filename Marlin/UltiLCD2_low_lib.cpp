#include "Configuration.h"
#include "pins.h"
#include "UltiLCD2_low_lib.h"
#include "UltiLCD2_hi_lib.h"
#include "stringHelpers.h"
#include "FONT_Arial5x6.h"
#include "FONT_small4x6.h"
#include "FONT_Standard5x7.h"
#include "FONT_smaller.h"
#ifdef ENABLE_ULTILCD2
/**
 * Implementation of the LCD display routines for a SSD1309 OLED graphical display connected with i2c.
 **/



#define LCD_RESET_PIN 5
#define LCD_CS_PIN    6
#define I2C_SDA_PIN   20
#define I2C_SCL_PIN   21

#define I2C_FREQ ((unsigned long) (400000*I2C_OVERCLOCK))
//#define I2C_FREQ 1000000

//The TWI interrupt routine conflicts with an interrupt already defined by Arduino, if you are using the Arduino IDE.
// Not running the screen update from interrupts causes a 25ms delay each screen refresh. Which will cause issues during printing.
// I recommend against using the Arduino IDE and setup a proper development environment.
#define USE_TWI_INTERRUPT 1

#define I2C_WRITE   0x00
#define I2C_READ    0x01

#define I2C_LED_ADDRESS 0b1100000

#define I2C_LCD_ADDRESS 0b0111100
#define I2C_LCD_SEND_COMMAND 0x00
#define I2C_LCD_SEND_DATA    0x40

#define LCD_COMMAND_CONTRAST                0x81
#define LCD_COMMAND_FULL_DISPLAY_ON_DISABLE 0xA4
#define LCD_COMMAND_FULL_DISPLAY_ON_ENABLE  0xA5
#define LCD_COMMAND_INVERT_DISABLE          0xA6
#define LCD_COMMAND_INVERT_ENABLE           0xA7
#define LCD_COMMAND_DISPLAY_OFF             0xAE
#define LCD_COMMAND_DISPLAY_ON              0xAF
#define LCD_COMMAND_NOP                     0xE3
#define LCD_COMMAND_LOCK_COMMANDS           0xFD	// 0xDF  

#define LCD_COMMAND_SET_ADDRESSING_MODE     0x20

/** Backbuffer for LCD */
uint8_t lcd_buffer[LCD_GFX_WIDTH * (LCD_GFX_HEIGHT+1) / 8];		// add one row of guard buffer... should never hit it, but....just in case...
uint8_t led_r, led_g, led_b;



/**
 * i2c communiation low level functions.
 */
static inline void i2c_start()
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
}

static inline void i2c_restart()
{
    while (!(TWCR & (1<<TWINT))) {}
    i2c_start();
}

static inline void i2c_send_raw(uint8_t data)
{
    while (!(TWCR & (1<<TWINT))) {}
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
}

static inline void i2c_end()
{
    while (!(TWCR & (1<<TWINT))) {}
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

static void i2c_led_write(uint8_t addr, uint8_t data)
{
    i2c_start();
    i2c_send_raw(I2C_LED_ADDRESS << 1 | I2C_WRITE);
    i2c_send_raw(addr);
    i2c_send_raw(data);
    i2c_end();
}
uint8_t * LCD_BITMAP_END;


#if USE_TWI_INTERRUPT
uint16_t lcd_update_pos = 0;
ISR(TWI_vect)
	{
	if (lcd_update_pos == LCD_GFX_WIDTH*LCD_GFX_HEIGHT/8)
		{
		i2c_end();
		}
	else
		{
		i2c_send_raw(lcd_buffer[lcd_update_pos]);
		TWCR |= _BV(TWIE);
		lcd_update_pos++;
		}
	}
#endif



void lcd_lib_init()
{
    SET_OUTPUT(LCD_CS_PIN);
    SET_OUTPUT(LCD_RESET_PIN);

    SET_OUTPUT(I2C_SDA_PIN);
    SET_OUTPUT(I2C_SCL_PIN);

    //Set the beeper as output.
    SET_OUTPUT(BEEPER);

    //Set the encoder bits and encoder button as inputs with pullup
    SET_INPUT(BTN_EN1);
    SET_INPUT(BTN_EN2);
    SET_INPUT(BTN_ENC);
    WRITE(BTN_EN1, 1);
    WRITE(BTN_EN2, 1);
    WRITE(BTN_ENC, 1);

    SET_INPUT(SDCARDDETECT);
    WRITE(SDCARDDETECT, HIGH);

    WRITE(LCD_CS_PIN, 0);
    WRITE(I2C_SDA_PIN, 1);
    WRITE(I2C_SCL_PIN, 1);

    WRITE(LCD_RESET_PIN, 0);
    _delay_ms(1);
    WRITE(LCD_RESET_PIN, 1);
    _delay_ms(1);

    //ClockFreq = (F_CPU) / (16 + 2*TWBR * 4^TWPS)
    //TWBR = ((F_CPU / ClockFreq) - 16)/2*4^TWPS
    TWBR = ((F_CPU / I2C_FREQ) - 16)/2*1;
    TWSR = 0x00;

    i2c_led_write(0, 0x80);//MODE1
    i2c_led_write(1, 0x1C);//MODE2
    i2c_led_write(2, led_r);//PWM0
    i2c_led_write(3, led_g);//PWM1
    i2c_led_write(4, led_b);//PWM2
    i2c_led_write(5, 0x00);//PWM3
    i2c_led_write(6, 0xFF);//GRPPWM
    i2c_led_write(7, 0x00);//GRPFREQ
    i2c_led_write(8, 0xAA);//LEDOUT

    i2c_start();
    i2c_send_raw(I2C_LCD_ADDRESS << 1 | I2C_WRITE);
    i2c_send_raw(I2C_LCD_SEND_COMMAND);

    i2c_send_raw(LCD_COMMAND_LOCK_COMMANDS);
    i2c_send_raw(0x12);

    i2c_send_raw(LCD_COMMAND_DISPLAY_OFF);

    i2c_send_raw(0xD5);//Display clock divider/freq
    i2c_send_raw(0xA0);

    i2c_send_raw(0xA8);//Multiplex ratio
    i2c_send_raw(0x3F);

    i2c_send_raw(0xD3);//Display offset
    i2c_send_raw(0x00);

    i2c_send_raw(0x40);//Set start line

    i2c_send_raw(0xA1);//Segment remap

    i2c_send_raw(0xC8);//COM scan output direction
    i2c_send_raw(0xDA);//COM pins hardware configuration
    i2c_send_raw(0x12);

    i2c_send_raw(LCD_COMMAND_CONTRAST);
    i2c_send_raw(0xDF);

    i2c_send_raw(0xD9);//Pre charge period
    i2c_send_raw(0x82);

    i2c_send_raw(0xDB);//VCOMH Deslect level
    i2c_send_raw(0x34);

    i2c_send_raw(LCD_COMMAND_SET_ADDRESSING_MODE);

    i2c_send_raw(LCD_COMMAND_FULL_DISPLAY_ON_DISABLE);

    i2c_send_raw(LCD_COMMAND_DISPLAY_ON);
    i2c_end();

    LCD_BITMAP_END =  lcd_buffer + (LCD_GFX_WIDTH * (LCD_GFX_HEIGHT) / 8);

    lcd_lib_buttons_update_interrupt();
    lcd_lib_buttons_update();
    lcd_lib_encoder_pos = 0;
    lcd_lib_update_screen();
	lcd_update_pos = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
void lcd_lib_update_screen()
{
    lcd_lib_update_RGB_LED();

    i2c_start();
    i2c_send_raw(I2C_LCD_ADDRESS << 1 | I2C_WRITE);
    //Set the drawin position to 0,0
    i2c_send_raw(I2C_LCD_SEND_COMMAND);
    i2c_send_raw(0x00 | (0 & 0x0F));
    i2c_send_raw(0x10 | (0 >> 4));
    i2c_send_raw(0xB0 | 0);

    i2c_restart();
    i2c_send_raw(I2C_LCD_ADDRESS << 1 | I2C_WRITE);
    i2c_send_raw(I2C_LCD_SEND_DATA);
#if USE_TWI_INTERRUPT
    lcd_update_pos = 0;
    TWCR |= _BV(TWIE);
#else
    for(uint16_t n=0; n<LCD_GFX_WIDTH*LCD_GFX_HEIGHT/8; n++)
        {
            i2c_send_raw(lcd_buffer[n]);
        }
    i2c_end();
#endif
}

bool lcd_lib_update_ready()
{
#if USE_TWI_INTERRUPT
    return !(TWCR & _BV(TWIE));
#else
    return true;
#endif
}

bool led_forced=false;
// if forced, then subsequent color requests will be ignored unless also forced.
// setting a forced 0,0,0 will clear the forced state and go back to normal operation
// this is to support user/ gcode setting of the rgb as an override and not
// letting the normal colors show.
void lcd_lib_led_color(uint8_t r, uint8_t g, uint8_t b, bool forced)
{
    led_forced = forced;
    if (led_forced)
        if (r==0 && g==0 && b==0) led_forced = false;
    if (led_forced && !forced) return;
    led_r = r;
    led_g = g;
    led_b = b;
}

// the baseline ASCII->font table offset.  Was 32 (space) but shifted down by four as I added four custom characters to the font
// had to add them to the start of the table, because we're using char, which are signed and top out at 128, which is already the max table value.
#define FONT_BASE_CHAR 32



//-----------------------------------------------------------------------------------------------------------------
void drawSmallChar( uint8_t y, uint8_t index, bool clear, uint8_t* &dst, uint8_t* &dst2 )
{
    if (dst>LCD_BITMAP_END) return;

    constrain (index,(uint8_t) ((uint16_t)pgm_read_word(&f_04b036ptFontInfo[1])),(uint8_t)((uint16_t)pgm_read_word(&f_04b036ptFontInfo[2])));

    index -= FONT_BASE_CHAR;;
    uint8_t yshift = y % 8;
    uint8_t yshift2 = 8 - yshift;
    uint16_t offset = (uint16_t)pgm_read_word(&f_04b036ptDescriptors[index][2]);
    uint8_t width = (uint16_t)pgm_read_word(&f_04b036ptDescriptors[index][0]);
    if (width < 1) return;
    if (width > 8) return;
    const uint8_t* src = f_04b036ptBitmaps + offset;
    for(uint8_t i = 0; i < width; i++)
        {
            if (dst>LCD_BITMAP_END) continue;
            if (clear)
                {*dst = (*dst) &~(pgm_read_byte(src++) << yshift); dst++;}
            else
                {*dst = (*dst) | pgm_read_byte(src++) << yshift; dst++;}
        }

    if (yshift != 0)
        {
			if (dst2>LCD_BITMAP_END) return;
            src = f_04b036ptBitmaps + offset;
            for(uint8_t i = 0; i < width; i++)
                {
                    if (dst2>LCD_BITMAP_END) continue;
                    if (clear)
                        {*dst2 = (*dst2) &~(pgm_read_byte(src++) >> yshift2); dst2++;}
                    else
                        {*dst2 = (*dst2) | pgm_read_byte(src++) >> yshift2; dst2++;}
                }
            dst2++;
        }
    dst++;

}


#define FONT_WIDTH 5

//-----------------------------------------------------------------------------------------------------------------
void drawMedlChar( uint8_t y, uint8_t index, bool clear, uint8_t* &dst, uint8_t* &dst2 )
{
    if (dst>LCD_BITMAP_END) return;

    constrain (index,(uint8_t) FONT_BASE_CHAR,FONT_BASE_CHAR+sizeof(lcd_font_7x5)/FONT_WIDTH);

    index -= FONT_BASE_CHAR;;
    uint8_t yshift = y % 8;
    uint8_t yshift2 = 8 - yshift;
    uint8_t width =FONT_WIDTH;
    const uint8_t* src = lcd_font_7x5 + (index * FONT_WIDTH);
    for(uint8_t i = 0; i < width; i++)
        {
            if (dst>LCD_BITMAP_END) continue;
            if (clear)
                {*dst = (*dst) &~(pgm_read_byte(src++) << yshift); dst++;}
            else
                {*dst = (*dst) | pgm_read_byte(src++) << yshift; dst++;}

        }

    if (yshift != 0)
        {
		if (dst2>LCD_BITMAP_END) return;
            src =  lcd_font_7x5 + (index * FONT_WIDTH);
            for(uint8_t i = 0; i < width; i++)
                {
                    if (dst2>LCD_BITMAP_END) continue;
                    if (clear)
                        {*dst2 = (*dst2) &~(pgm_read_byte(src++) >> yshift2); dst2++;}
                    else
                        {*dst2 = (*dst2) | pgm_read_byte(src++) >> yshift2; dst2++;}
                }
            dst2++;

        }
    dst++;
}


void lcd_lib_draw_small_stringP(uint8_t x, uint8_t y, const char* pstr, bool clear)
{
    uint8_t* dst = lcd_buffer + x + (y / 8) * LCD_GFX_WIDTH;
    uint8_t* dst2 = lcd_buffer + x + (y / 8) * LCD_GFX_WIDTH + LCD_GFX_WIDTH;
  //  uint8_t index;
    for(char c = pgm_read_byte(pstr); c; c = pgm_read_byte(++pstr))
        {
            drawSmallChar(y, c, clear, dst, dst2);
        }
}


void lcd_lib_draw_small_string(uint8_t x, uint8_t y, const char* str, bool clear)
{
    uint8_t* dst = lcd_buffer + x + (y / 8) * LCD_GFX_WIDTH;
    uint8_t* dst2 = lcd_buffer + x + (y / 8) * LCD_GFX_WIDTH + LCD_GFX_WIDTH;
//    uint8_t index;
    while(*str)
        {
            drawSmallChar(y, *str , clear, dst, dst2);
            str++;
        }
}




void lcd_lib_draw_string(uint8_t x, uint8_t y, const char* str)
{
    lcd_lib_draw_small_string(x,y,str,false);
}

void lcd_lib_clear_string(uint8_t x, uint8_t y, const char* str)
{
    lcd_lib_draw_small_string(x,y,str,true);
}



void lcd_lib_draw_stringP(uint8_t x, uint8_t y, const char* pstr, bool clear)
{
    lcd_lib_draw_small_stringP(x,y,pstr,false);
}

void lcd_lib_clear_stringP(uint8_t x, uint8_t y, const char* pstr)
{
    lcd_lib_draw_small_stringP(x,y,pstr,true);
    return;

    lcd_lib_draw_stringP (x,y,pstr, true);
    return;


}


byte lcd_lib_get_string_lengthP (const char* pstr)
{
    byte length = 0;

    for(char c = pgm_read_byte(pstr); c; c = pgm_read_byte(++pstr))
        {
            int index = c;
            constrain (index,(uint8_t) ((uint16_t)pgm_read_word(&f_04b036ptFontInfo[1])),(uint8_t)((uint16_t)pgm_read_word(&f_04b036ptFontInfo[2])));
            index -= FONT_BASE_CHAR;
			int w = 1+(uint16_t)pgm_read_word(&f_04b036ptDescriptors[index][0]);
			length +=constrain (w,1,8);
        }
    return length;
}


byte lcd_lib_get_string_length (const char* str)
{
    byte length = 0;

    while(*str)
        {
			int index = *str;
			constrain (index,(uint8_t) ((uint16_t)pgm_read_word(&f_04b036ptFontInfo[1])),(uint8_t)((uint16_t)pgm_read_word(&f_04b036ptFontInfo[2])));
    		index -= FONT_BASE_CHAR;//FONT_BASE_CHAR
            int w = 1+(uint16_t)pgm_read_word(&f_04b036ptDescriptors[index][0]);
            length +=constrain (w,1,8);
            str++;
        }
    return length;
}

void lcd_lib_draw_string_right(uint8_t y, const char* str, byte startpos)
{
    lcd_lib_draw_string(startpos - lcd_lib_get_string_length(str) , y, str);		// move 2 pixels in fom extreme right side
}


void lcd_lib_draw_string_center(uint8_t y, const char* str)
{
    lcd_lib_draw_string(64 - lcd_lib_get_string_length(str)/2, y, str);
}

void lcd_lib_clear_string_center(uint8_t y, const char* str)
{
    lcd_lib_clear_string(64 - lcd_lib_get_string_length(str)/2, y, str);
}

void lcd_lib_draw_string_centerP(uint8_t y, const char* pstr)
{
    lcd_lib_draw_stringP(64 - lcd_lib_get_string_lengthP(pstr)/2, y, pstr);
}

void lcd_lib_clear_string_centerP(uint8_t y, const char* pstr)
{
    lcd_lib_clear_stringP(64 - lcd_lib_get_string_lengthP(pstr)/2, y, pstr);
}

void lcd_lib_draw_string_center_atP(uint8_t x, uint8_t y, const char* pstr)
{
    const char* split = strchr_P(pstr, '|');
    if (split)
        {
            char buf[24];
            strncpy_P(buf, pstr, split - pstr);
            buf[split - pstr] = '\0';
            lcd_lib_draw_string(x - lcd_lib_get_string_lengthP(buf)/2, y - 5, buf);
            lcd_lib_draw_stringP(x - lcd_lib_get_string_lengthP(split+1) /2, y + 5, split+1);
        }
    else
        {
            lcd_lib_draw_stringP(x - lcd_lib_get_string_lengthP(pstr) /2, y, pstr);
        }
}

void lcd_lib_clear_string_center_atP(uint8_t x, uint8_t y, const char* pstr)
{
    const char* split = strchr_P(pstr, '|');
    if (split)
        {
            char buf[24];
            strncpy_P(buf, pstr, split - pstr);
            buf[split - pstr] = '\0';
            lcd_lib_clear_string(x - lcd_lib_get_string_lengthP(buf) /2, y - 5, buf);
            lcd_lib_clear_stringP(x - lcd_lib_get_string_lengthP(split+1) /2, y + 5, split+1);
        }
    else
        {
            lcd_lib_clear_stringP(x - lcd_lib_get_string_lengthP(pstr) /2 , y, pstr);
        }
}

void lcd_lib_draw_hline(uint8_t x0, uint8_t x1, uint8_t y)
{
    if (x1 < x0) XORSWAP(x0,x1);
    uint8_t* dst = lcd_buffer + x0 + (y / 8) * LCD_GFX_WIDTH;
    uint8_t mask = 0x01 << (y % 8);

    while(x0 <= x1)
        {
            *dst++ |= mask;
            x0 ++;
            if (dst>LCD_BITMAP_END) return;
        }

}

void lcd_lib_draw_dotted_hline(uint8_t x0, uint8_t x1, uint8_t y)
{
    if (x1 < x0) XORSWAP(x0,x1);
    byte a;
    for (a=x0; a<x1; a+=2)
        lcd_lib_draw_hline(a,a,y);

}


void lcd_lib_draw_vline(uint8_t x, uint8_t y0, uint8_t y1)
{
    if (y1 < y0) XORSWAP(y0,y1);
    uint8_t* dst0 = lcd_buffer + x + (y0 / 8) * LCD_GFX_WIDTH;
    uint8_t* dst1 = lcd_buffer + x + (y1 / 8) * LCD_GFX_WIDTH;
    if (dst0 == dst1)
        {
            *dst0 |= (0xFF << (y0 % 8)) & (0xFF >> (7 - (y1 % 8)));
            if (dst0>LCD_BITMAP_END) return;

        }
    else
        {
            *dst0 |= 0xFF << (y0 % 8);
            dst0 += LCD_GFX_WIDTH;
            while(dst0 != dst1)
                {
                    *dst0 = 0xFF;
                    dst0 += LCD_GFX_WIDTH;
                }
            *dst1 |= 0xFF >> (7 - (y1 % 8));
            if (dst0>LCD_BITMAP_END|| dst1>LCD_BITMAP_END) return;

        }
}

void lcd_lib_draw_box(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    if (y1 < y0) XORSWAP(y0,y1);
    lcd_lib_draw_vline(x0, y0+1, y1-1);
    lcd_lib_draw_vline(x1, y0+1, y1-1);
    lcd_lib_draw_hline(x0+1, x1-1, y0);
    lcd_lib_draw_hline(x0+1, x1-1, y1);
}

void lcd_lib_draw_shade(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t* dst0 = lcd_buffer + x0 + (y0 / 8) * LCD_GFX_WIDTH;
    uint8_t* dst1 = lcd_buffer + x0 + (y1 / 8) * LCD_GFX_WIDTH;
    if (dst0 == dst1)
        {
            //uint8_t mask = (0xFF << (y0 % 8)) & (0xFF >> (7 - (y1 % 8)));
            //*dstA0 |= (mask & 0xEE);
        }
    else
        {
            uint8_t mask = 0xFF << (y0 % 8);
            uint8_t* dst = dst0;
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst++ |= mask & ((x & 1) ? 0xAA : 0x55);
                    if (dst>LCD_BITMAP_END) return;
                }


            dst0 += LCD_GFX_WIDTH;
            while(dst0 != dst1)
                {
                    dst = dst0;
                    for(uint8_t x=x0; x<=x1; x++)
                        {
                            *dst++ |= (x & 1) ? 0xAA : 0x55;
                            if (dst>LCD_BITMAP_END) return;
                        }
                    dst0 += LCD_GFX_WIDTH;
                }
            dst = dst1;
            mask = 0xFF >> (7 - (y1 % 8));
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst++ |= mask & ((x & 1) ? 0xAA : 0x55);
                    if (dst>LCD_BITMAP_END) return;
                }

        }
}

void lcd_lib_clear()
{
    memset( lcd_buffer, 0, sizeof(lcd_buffer));
}

void lcd_lib_set()
{
    memset(lcd_buffer, 0xFF, sizeof(lcd_buffer));
}

void lcd_lib_clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t* dst0 = lcd_buffer + x0 + (y0 / 8) * LCD_GFX_WIDTH;
    uint8_t* dst1 = lcd_buffer + x0 + (y1 / 8) * LCD_GFX_WIDTH;
    if (dst0 == dst1)
        {
            uint8_t mask = (0xFF << (y0 % 8)) & (0xFF >> (7 - (y1 % 8)));
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst0++ &=~mask;
                    if (dst0>LCD_BITMAP_END) return;
                }
        }
    else
        {
            uint8_t mask = 0xFF << (y0 % 8);
            uint8_t* dst = dst0;
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst++ &=~mask;
                    if (dst>LCD_BITMAP_END) return;
                }
            dst0 += LCD_GFX_WIDTH;
            while(dst0 != dst1)
                {
                    dst = dst0;
                    for(uint8_t x=x0; x<=x1; x++)
                        *dst++ = 0x00;
                    dst0 += LCD_GFX_WIDTH;
                    if (dst0>LCD_BITMAP_END) return;
                }
            dst = dst1;
            mask = 0xFF >> (7 - (y1 % 8));
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst++ &=~mask;
                    if (dst>LCD_BITMAP_END) return;
                }
        }
}

void lcd_lib_invert(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t* dst0 = lcd_buffer + x0 + (y0 / 8) * LCD_GFX_WIDTH;
    uint8_t* dst1 = lcd_buffer + x0 + (y1 / 8) * LCD_GFX_WIDTH;
    if (dst0 == dst1)
        {
            uint8_t mask = (0xFF << (y0 % 8)) & (0xFF >> (7 - (y1 % 8)));
            for(uint8_t x=x0; x<=x1; x++)
                {

                    *dst0++ ^= mask;
                    if (dst0>LCD_BITMAP_END) return;
                }
        }
    else
        {
            uint8_t mask = 0xFF << (y0 % 8);
            uint8_t* dst = dst0;
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst++ ^= mask;
                    if (dst>LCD_BITMAP_END) return;
                }

            dst0 += LCD_GFX_WIDTH;
            while(dst0 != dst1)
                {
                    dst = dst0;
                    for(uint8_t x=x0; x<=x1; x++)
                        {
                            *dst++ ^= 0xFF;
                            if (dst>LCD_BITMAP_END) return;
                        }
                    dst0 += LCD_GFX_WIDTH;
                    if (dst0>LCD_BITMAP_END) return;
                }
            dst = dst1;
            mask = 0xFF >> (7 - (y1 % 8));
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst++ ^= mask;
                    if (dst>LCD_BITMAP_END) return;
                }
        }
}

void lcd_lib_set(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t* dst0 = lcd_buffer + x0 + (y0 / 8) * LCD_GFX_WIDTH;
    uint8_t* dst1 = lcd_buffer + x0 + (y1 / 8) * LCD_GFX_WIDTH;
    if (dst0 == dst1)
        {
            uint8_t mask = (0xFF << (y0 % 8)) & (0xFF >> (7 - (y1 % 8)));
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst0++ |= mask;
                    if (dst0>LCD_BITMAP_END) return;
                }
        }
    else
        {
            uint8_t mask = 0xFF << (y0 % 8);
            uint8_t* dst = dst0;
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst++ |= mask;
                    if (dst>LCD_BITMAP_END) return;
                }
            dst0 += LCD_GFX_WIDTH;
            while(dst0 != dst1)
                {
                    dst = dst0;
                    for(uint8_t x=x0; x<=x1; x++)
                        {
                            *dst++ = 0xFF;
                            if (dst>LCD_BITMAP_END) return;
                        }
                    dst0 += LCD_GFX_WIDTH;
                    if (dst0>LCD_BITMAP_END) return;
                }
            dst = dst1;
            mask = 0xFF >> (7 - (y1 % 8));
            for(uint8_t x=x0; x<=x1; x++)
                {
                    *dst++ |= mask;
                    if (dst>LCD_BITMAP_END) return;
                }
        }
}



void lcd_lib_draw_gfx(uint8_t x, uint8_t y, const uint8_t* gfx)
{
    uint8_t w = pgm_read_byte(gfx++);
    uint8_t h = (pgm_read_byte(gfx++) + 7) / 8;
    uint8_t shift = y % 8;
    uint8_t shift2 = 8 - shift;
    y /= 8;

    for(; h; h--)
        {
            if (y >= LCD_GFX_HEIGHT / 8) break;

            uint8_t* dst0 = lcd_buffer + x + y * LCD_GFX_WIDTH;
            uint8_t* dst1 = lcd_buffer + x + y * LCD_GFX_WIDTH + LCD_GFX_WIDTH;
            for(uint8_t _w = w; _w; _w--)
                {
                    uint8_t c = pgm_read_byte(gfx++);
                    *dst0++ |= c << shift;
                    if (dst0>LCD_BITMAP_END) return;
                    if (shift && y < 7)
                        *dst1++ |= c >> shift2;
                    if (dst1>LCD_BITMAP_END) return;
                }
            y++;
        }
}

void lcd_lib_clear_gfx(uint8_t x, uint8_t y, const uint8_t* gfx)
{
    uint8_t w = pgm_read_byte(gfx++);
    uint8_t h = (pgm_read_byte(gfx++) + 7) / 8;
    uint8_t shift = y % 8;
    uint8_t shift2 = 8 - shift;
    y /= 8;

    for(; h; h--)
        {
            if (y >= LCD_GFX_HEIGHT / 8) break;

            uint8_t* dst0 = lcd_buffer + x + y * LCD_GFX_WIDTH;
            uint8_t* dst1 = lcd_buffer + x + y * LCD_GFX_WIDTH + LCD_GFX_WIDTH;
            for(uint8_t _w = w; _w; _w--)
                {
                    uint8_t c = pgm_read_byte(gfx++);
                    *dst0++ &=~(c << shift);
                    if (dst0>LCD_BITMAP_END) return;
                    if (shift && y < 7)
                        *dst1++ &=~(c >> shift2);
                    if (dst1>LCD_BITMAP_END) return;
                }
            y++;
        }
}

void lcd_lib_beep()
{
#define _BEEP(c, n) for(int8_t _i=0;_i<c;_i++) { WRITE(BEEPER, HIGH); _delay_us(n); WRITE(BEEPER, LOW); _delay_us(n); }
    _BEEP(20, 366);
    _BEEP(10, 150);
#undef _BEEP
}


//-----------------------------------------------------------------------------------------------------------------
// very short tick for UI feedback -- 1 millisecond  long
void lcd_lib_tick( )
{
    LED_WHITE() ;
#if EXTENDED_BEEP
    for (int a =0; a<10; a++)
        {
            WRITE(BEEPER,0);
            _delay_us (50);
            WRITE(BEEPER,1);
            _delay_us(50);
        }
    WRITE(BEEPER,0);
#endif
    LED_NORMAL();


}
//-----------------------------------------------------------------------------------------------------------------
// freq in Hz, duration in milliseconds -- note that there will be a
// minimum time of one cycle.  ie: specifying a freq of 100Hz
// would mean one cycle takes 10ms, and that is the
// minimum time for the duration.
//
// and if it isn't obvious, THIS IS A BLOCKING CALL!
// don't try to be playing fancy chip tunes when time is important!  ;)
void lcd_lib_beep_ext( unsigned int freq, unsigned int dur )
{
#if EXTENDED_BEEP
    freq = 500000UL/freq;
    unsigned long start_time = millis();
    while (millis() - start_time < dur)
        {
            WRITE(BEEPER,0);
            _delay_us (freq);
            WRITE(BEEPER,1);
            _delay_us(freq);
        }
    WRITE(BEEPER,0);
#endif
}

int8_t lcd_lib_encoder_pos_interrupt = 0;
int16_t lcd_lib_encoder_pos = 0;
bool lcd_lib_button_was_pressed = false;
bool lcd_lib_button_down;

#define ENCODER_ROTARY_BIT_0 _BV(0)
#define ENCODER_ROTARY_BIT_1 _BV(1)
/* Warning: This function is called from interrupt context */
void lcd_lib_buttons_update_interrupt()
{
    static uint8_t lastEncBits = 0;

    uint8_t encBits = 0;
    if(!READ(BTN_EN1)) encBits |= ENCODER_ROTARY_BIT_0;
    if(!READ(BTN_EN2)) encBits |= ENCODER_ROTARY_BIT_1;

    if(encBits != lastEncBits)
        {
            switch(encBits)
                {
                    case encrot0:
                        if(lastEncBits==encrot3)
                            lcd_lib_encoder_pos_interrupt++;
                        else
                            if(lastEncBits==encrot1)
                                lcd_lib_encoder_pos_interrupt--;
                        break;
                    case encrot1:
                        if(lastEncBits==encrot0)
                            lcd_lib_encoder_pos_interrupt++;
                        else
                            if(lastEncBits==encrot2)
                                lcd_lib_encoder_pos_interrupt--;
                        break;
                    case encrot2:
                        if(lastEncBits==encrot1)
                            lcd_lib_encoder_pos_interrupt++;
                        else
                            if(lastEncBits==encrot3)
                                lcd_lib_encoder_pos_interrupt--;
                        break;
                    case encrot3:
                        if(lastEncBits==encrot2)
                            lcd_lib_encoder_pos_interrupt++;
                        else
                            if(lastEncBits==encrot0)
                                lcd_lib_encoder_pos_interrupt--;
                        break;
                }
            lastEncBits = encBits;
        }
}

// this is changed by the various menus to enable or disable encoder acceleration for that time.
bool allow_encoder_acceleration = false;
bool clear_button_press = true;
void lcd_lib_buttons_update()
{


// Added encoder acceleration (Lars Jun 2014)
// if we detect we're moving the encoder the same direction for repeated frames, we increase our step size (up to a maximum)
// if we stop, or change direction, set the step size back to +/- 1
// we only want this for SOME things, like changing a value, and not for other things, like a menu.
// so we have an enable bit
    static char encoder_accel = 0;
    if (lcd_lib_encoder_pos_interrupt > 0)		// positive -- were we already going positive last time?  If so, increase our accel
        {
            if (encoder_accel > 0 )
                encoder_accel ++;
            else
                encoder_accel = 1;
#if EXTENDED_BEEP
            // if we're using acceleration, beep with a pitch changing tone based on the accel value
            if (allow_encoder_acceleration) lcd_lib_beep_ext (500+encoder_accel*25,10);
#endif
        }
    if (lcd_lib_encoder_pos_interrupt <0)	// negative -- decrease our negative accel
        {
            if (encoder_accel < 0 )
                encoder_accel --;
            else
                encoder_accel = -1;
#if EXTENDED_BEEP
            if (allow_encoder_acceleration) lcd_lib_beep_ext (400+encoder_accel*25,10);
#endif
        }
    if (/*lcd_lib_encoder_pos_interrupt ==0 ||*/ (millis() - last_user_interaction > 300) )						// no movement --  back to 0 acceleration
        {
            encoder_accel=0;
        }
    if (encoder_accel <-MAX_ENCODER_ACCELERATION) encoder_accel = -MAX_ENCODER_ACCELERATION;
    if (encoder_accel > MAX_ENCODER_ACCELERATION) encoder_accel = MAX_ENCODER_ACCELERATION;
    if (!allow_encoder_acceleration) encoder_accel=1;

    lcd_lib_encoder_pos += abs(encoder_accel) * lcd_lib_encoder_pos_interrupt;

	if (clear_button_press)
		{
		lcd_lib_button_was_pressed= false;
		clear_button_press = false;
		} 
    uint8_t buttonState = !READ(BTN_ENC);
    if (buttonState && !lcd_lib_button_down) 
		lcd_lib_button_was_pressed= true;
    lcd_lib_button_down = buttonState;

    if  (lcd_lib_button_down || lcd_lib_encoder_pos_interrupt!=0 ) last_user_interaction=millis();
    lcd_lib_encoder_pos_interrupt = 0;
}


// this saves the pressed state until something reads it

bool lcd_lib_button_pressed()
	{
	 bool rv = lcd_lib_button_was_pressed;
	 if (rv && (millis() - last_user_interaction > 2000) ) rv = false;		// it was more than 2 seconds ago, don't respond to outdated presses!
	 if (rv) lcd_lib_beep_ext (1200,20);
	 clear_button_press = true;
	
	 return rv;
	}


//-----------------------------------------------------------------------------------------------------------------
void lcd_lib_update_RGB_LED()
{
    i2c_led_write(2, led_r);//PWM0
    i2c_led_write(3, led_g);//PWM1
    i2c_led_write(4, led_b);//PWM2
}
#endif//ENABLE_ULTILCD2
