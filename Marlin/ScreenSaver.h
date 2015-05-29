#pragma once
#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
//Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#define SCREENSAVER_BALLS 4


#define SS_MAX_SPED 3.0
#define SS_MAX_ACC 0.25
#define SS_GRAVITY 0.02
#define SS_MARGIN 10

class ball
{
        float pos_x ;
        float pos_y ;
        float dx ;
        float dy ;
        float ax ;
        float ay ;
public:
		void init();
        void update();
};


void lcd_menu_breakout();

