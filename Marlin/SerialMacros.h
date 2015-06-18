#pragma once


// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Serial Communication: 
#define SERIAL_PROTOCOL(x) MYSERIAL.print(x);
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y);
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x));
#define SERIAL_PROTOCOLLN(x) do {MYSERIAL.print(x);MYSERIAL.write('\n');} while(0)
#define SERIAL_PROTOCOLLNPGM(x) do{serialprintPGM(PSTR(x));MYSERIAL.write('\n');} while(0)
const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";
#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x) 

#define SERIAL_ECHO_START serialprintPGM(echomagic);
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

// void serial_echopair_P(const char *s_P, float v);
// void serial_echopair_P(const char *s_P, double v);
// void serial_echopair_P(const char *s_P, unsigned long v);


//things to write to serial from Program memory. saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}



inline void serial_echopair_P(const char *s_P, float v)
	{ serialprintPGM(s_P); SERIAL_ECHO(v); }
inline void serial_echopair_P(const char *s_P, double v)
	{ serialprintPGM(s_P); SERIAL_ECHO(v); }
inline void serial_echopair_P(const char *s_P, unsigned long v)
	{ serialprintPGM(s_P); SERIAL_ECHO(v); }
inline void serial_echopair_P(const char *s_P, int  v)
	{ serialprintPGM(s_P); SERIAL_ECHO(v); }
inline void serial_echopair_P(const char *s_P, byte  v)
	{ serialprintPGM(s_P); SERIAL_ECHO(v); }
inline void serial_echopair_P(const char *s_P, long  v)
	{ serialprintPGM(s_P); SERIAL_ECHO(v); }

