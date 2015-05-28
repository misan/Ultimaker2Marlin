#pragma once

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
//Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif



extern  char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];

extern int bufindr ;
extern int bufindw ;
extern int buflen ;

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// command queue management: 
extern int buflen;

inline bool is_command_queued() 	{ 	return buflen > 0;	}
inline  bool is_command_queue_full () 	{	return  buflen>=BUFSIZE;	}
inline uint8_t commands_queued()	{	return buflen;	}
void waitBuffer();
void get_command();
void clear_command_queue();
void enquecommand(const char *cmd,bool usePmem = false); //put an ascii command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ascii command at the end of the current buffer, read from flash

void manageBuffer();
void manage_inactivity();

void initQueue();
bool is_command_from_sd(int a);
