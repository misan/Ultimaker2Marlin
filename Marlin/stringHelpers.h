#pragma once

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
//Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

 

char* int_to_string(int i, char* temp_buffer, const char* p_postfix = NULL, bool use_OFF = false);
char* int_to_time_string(unsigned long i, char* temp_buffer);
char* float_to_string(float f, char* temp_buffer, const char* p_postfix = NULL);
char* float_to_string1(float f, char* temp_buffer, const char* p_postfix= NULL);
char* float_to_string3(float f, char* temp_buffer, const char* p_postfix= NULL);

