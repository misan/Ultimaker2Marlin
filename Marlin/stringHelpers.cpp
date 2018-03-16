#include "stringHelpers.h"


//-----------------------------------------------------------------------------------------------------------------
char* int_to_string( int i, char* temp_buffer, const char* p_postfix, bool use_OFF )
	{
	char* c = temp_buffer;
	if (use_OFF && i==0)
		{
		strcpy_P(c, PSTR ("OFF"));
		c += 3;
		return c;
		}

	if (i < 0)
		{
		*c++ = '-';
		i = -i;
		}
	if (i >= 10000)
		*c++ = ((i/10000)%10)+'0';
	if (i >= 1000)
		*c++ = ((i/1000)%10)+'0';
	if (i >= 100)
		*c++ = ((i/100)%10)+'0';
	if (i >= 10)
		*c++ = ((i/10)%10)+'0';
	*c++ = ((i)%10)+'0';
	*c = '\0';
	if (p_postfix)
		{
		strcpy_P(c, p_postfix);
		c += strlen_P(p_postfix);
		}
	return c;
	}

//-----------------------------------------------------------------------------------------------------------------
char* int_to_time_string( unsigned long i, char* temp_buffer )
	{
	char* c = temp_buffer;
	uint8_t hours = (i / 60 / 60) % 60;
	uint8_t mins = (i / 60) % 60;
	uint8_t secs = i % 60;

	if (hours > 0)
		{
		if (hours > 99)
			*c++ = '0' + hours / 100;
		if (hours > 9)
			*c++ = '0' + (hours / 10) % 10;
		*c++ = '0' + hours % 10;
		// Lars changes -- let's add a decimal point for hours less than 10 -- so we can tell is that 1.8 or 1.1 hours remaining
		// comment: decimal hours is a little weird, as we're used to thinking in terms of hours and minutes, but we don't have much screen space
		// and we are using human-readable text ("hours") -- if we switched to a digital clock	 display HH:MM:SS we could be much
		// more detailed, although the accuracy of our time predictions isn't that great anyway, so it's probably a false precision.
		if (hours < 10)
			{
			*c++ = '.';
			*c++ = '0' + (10*mins/60) % 10;
			}
		// end changes
		if (hours > 1)
			{
			strcpy_P(c, PSTR(" hours"));
			return c + 6;
			}
		strcpy_P(c, PSTR(" hour"));
		return c + 5;
		}
	if (mins > 0)
		{
		if (mins > 9)
			*c++ = '0' + (mins / 10) % 10;
		*c++ = '0' + mins % 10;
		strcpy_P(c, PSTR(" min"));
		return c + 4;
		}
	if (secs > 9)
		*c++ = '0' + secs / 10;
	*c++ = '0' + secs % 10;
	strcpy_P(c, PSTR(" sec"));
	return c + 4;
	/*
	if (hours > 99)
	*c++ = '0' + hours / 100;
	*c++ = '0' + (hours / 10) % 10;
	*c++ = '0' + hours % 10;
	*c++ = ':';
	*c++ = '0' + mins / 10;
	*c++ = '0' + mins % 10;
	*c++ = ':';
	*c++ = '0' + secs / 10;
	*c++ = '0' + secs % 10;
	*c = '\0';
	return c;
	*/
	}

//-----------------------------------------------------------------------------------------------------------------
char* float_to_string( float f, char* temp_buffer, const char* p_postfix )
	{
	int32_t i = f * 100.0 + 0.5;
	char* c = temp_buffer;
	if (i < 0)
		{
		*c++ = '-';
		i = -i;
		}
	if (i >= 10000)
		*c++ = ((i/10000)%10)+'0';
	if (i >= 1000)
		*c++ = ((i/1000)%10)+'0';
	*c++ = ((i/100)%10)+'0';
	*c++ = '.';
	// if (i >= 10)				ALWAYS PUT THE LEADING ZERO AFTER THE DECIMAL POINT!!!!
	*c++ = ((i/10)%10)+'0';
	*c++ = ((i)%10)+'0';
	*c = '\0';
	if (p_postfix)
		{
		strcpy_P(c, p_postfix);
		c += strlen_P(p_postfix);
		}
	return c;
	}

//-----------------------------------------------------------------------------------------------------------------
char* float_to_string1( float f, char* temp_buffer, const char* p_postfix )
	{
	int32_t i = f * 10.0 + 0.5;
	char* c = temp_buffer;
	if (i < 0)
		{
		*c++ = '-';
		i = -i;
		}
	if (i >= 1000)
		*c++ = ((i/1000)%10)+'0';
	if (i >= 100)
		*c++ = ((i/100)%10)+'0';
	*c++ = ((i/10)%10)+'0';
	*c++ = '.';
	// if (i >= 10)				ALWAYS PUT THE LEADING ZERO AFTER THE DECIMAL POINT!!!!
	*c++ = ((i)%10)+'0';
	*c = '\0';
	if (p_postfix)
		{
		strcpy_P(c, p_postfix);
		c += strlen_P(p_postfix);
		}
	return c;
	}

//-----------------------------------------------------------------------------------------------------------------
char* float_to_string3( float f, char* temp_buffer, const char* p_postfix )
	{
	int32_t i = f * 1000.0 + 0.5;
	char* c = temp_buffer;
	if (i < 0)
		{
		*c++ = '-';
		i = -i;
		}
	if (i >= 10000)
		*c++ = ((i/10000)%10)+'0';
	*c++ = ((i/1000)%10)+'0';
	*c++ = '.';
	*c++ = ((i/100)%10)+'0';
	*c++ = ((i/10)%10)+'0';
	*c++ = ((i)%10)+'0';
	*c = '\0';
	if (p_postfix)
		{
		strcpy_P(c, p_postfix);
		c += strlen_P(p_postfix);
		}
	return c;
	}
