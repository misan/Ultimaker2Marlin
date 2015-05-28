#include "Marlin.h"
#include "CommandQ.h"
#include "language.h"
#include "cardreader.h"
#include "MenuUseful.h"
#include "gcode.h"



char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
bool fromsd[BUFSIZE];
int bufindr = 0;
int bufindw = 0;
int buflen = 0;
//static int i = 0;
char serial_char;
int serial_count = 0;
boolean comment_mode = false;

extern unsigned long lastSerialCommandTime;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Clear all the commands in the ASCII command buffer, to make sure we have room for abort commands.
void clear_command_queue()
{
    if (is_command_queued())
        {
            bufindw = (bufindr + 1)%BUFSIZE;
            buflen = 1;
        }
}


//-----------------------------------------------------------------------------------------------------------------
void waitBuffer()
{
    while (is_command_queue_full())
        {
            SERIAL_ECHO_START;
            SERIAL_ECHOPGM("waiting to enque");
            delay(100);							// can this even work?  we aren't multithreaded and the commands are not processed on an interrupt, so if we block here., we'll never process commands...
// added manageBuffer() here
            manageBuffer();
        }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd, bool usePmem)
{
    if (is_command_queue_full())
        waitBuffer();

    if (!is_command_queue_full())
        {
            //this is dangerous if a mixing of serial and this happsens
            if (usePmem)
                strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
            else
                strcpy(&(cmdbuffer[bufindw][0]),cmd);
            SERIAL_ECHO_START;
            SERIAL_ECHOPGM("enqueing \"");
            SERIAL_ECHO(cmdbuffer[bufindw]);
            SERIAL_ECHOLNPGM("\"");
            bufindw= (bufindw + 1)%BUFSIZE;
            buflen += 1;
        }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void enquecommand_P(const char *cmd)
{
    enquecommand(cmd,true);
}

extern char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc


//-----------------------------------------------------------------------------------------------------------------
void getCommandSerial()
{
    {
        serial_char = MYSERIAL.read();
        if(serial_char == '\n' ||
                serial_char == '\r' ||
                (serial_char == ':' && comment_mode == false) ||
                serial_count >= (MAX_CMD_SIZE - 1) )
            {
                if(!serial_count)   //if empty line
                    {
                        comment_mode = false; //for new command
                        return;
                    }
                cmdbuffer[bufindw][serial_count] = 0; //terminate string
                if(!comment_mode)
                    {
                        comment_mode = false; //for new command
                        fromsd[bufindw] = false;
                        if(strchr(cmdbuffer[bufindw], 'N') != NULL)
                            {
                                strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
                                gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
                                if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) )
                                    {
                                        SERIAL_ERROR_START;
                                        SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
                                        SERIAL_ERRORLN(gcode_LastN);
                                        //Serial.println(gcode_N);
                                        FlushSerialRequestResend();
                                        serial_count = 0;
                                        return;
                                    }

                                if(strchr(cmdbuffer[bufindw], '*') != NULL)
                                    {
                                        byte checksum = 0;
                                        byte count = 0;
                                        while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
                                        strchr_pointer = strchr(cmdbuffer[bufindw], '*');

                                        if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum)
                                            {
                                                SERIAL_ERROR_START;
                                                SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
                                                SERIAL_ERRORLN(gcode_LastN);
                                                FlushSerialRequestResend();
                                                serial_count = 0;
                                                return;
                                            }
                                        //if no errors, continue parsing
                                    }
                                else
                                    {
                                        SERIAL_ERROR_START;
                                        SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
                                        SERIAL_ERRORLN(gcode_LastN);
                                        FlushSerialRequestResend();
                                        serial_count = 0;
                                        return;
                                    }

                                gcode_LastN = gcode_N;
                                //if no errors, continue parsing
                            }
                        else  // if we don't receive 'N' but still see '*'
                            {
                                if((strchr(cmdbuffer[bufindw], '*') != NULL))
                                    {
                                        SERIAL_ERROR_START;
                                        SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
                                        SERIAL_ERRORLN(gcode_LastN);
                                        serial_count = 0;
                                        return;
                                    }
                            }
                        if((strchr(cmdbuffer[bufindw], 'G') != NULL))
                            {
                                strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
                                switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL))))
                                    {
                                        case 0:
                                        case 1:
                                        case 2:
                                        case 3:
                                            if(Stopped == false)   // If printer is stopped by an error the G[0-3] codes are ignored.
                                                {
#ifdef SDSUPPORT
                                                    if(card.saving)
                                                        break;
#endif //SDSUPPORT
                                                    SERIAL_PROTOCOLLNPGM(MSG_OK);
                                                }
                                            else
                                                {
                                                    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
                                                    LCD_MESSAGEPGM(MSG_STOPPED);
                                                }
                                            break;
                                        default:
                                            break;
                                    }

                            }
#ifdef ENABLE_ULTILCD2
                        strchr_pointer = strchr(cmdbuffer[bufindw], 'M');
                        if (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10) != 105)
                            lastSerialCommandTime = millis();
#endif
                        bufindw = (bufindw + 1)%BUFSIZE;
                        buflen += 1;
                    }
                serial_count = 0; //clear buffer
            }
        else
            {
                if(serial_char == ';') comment_mode = true;
                if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
            }
    }
}



#define MAX_COMMENT_LENGTH 20
unsigned long current_layer= 0;

//-----------------------------------------------------------------------------------------------------------------
// do whatyever fancy schmancy stuff we might want to do with a comment here:
void processComment( char * comment )
{
    SERIAL_ECHO_START
    SERIAL_ECHOLN (comment) ;

    String str(comment);
    int pos =str.indexOf("Layer:");
    if (pos >= 0)
        {
            current_layer = str.substring(pos).toInt();
            SERIAL_ECHO_START
            String str2 = "Layer " + current_layer;// + " / " + total_layers;

            serial_echopair_P("Starting layer ", current_layer);
            if (card.sdprinting)  serial_echopair_P("File position = ", card.getFilePos());
            lcd_setstatus((char *) str2.c_str());
            SERIAL_ECHOLN("");
        }
}
//-----------------------------------------------------------------------------------------------------------------
void getCommandSDCard(  )
{
    {
        static uint32_t endOfLineFilePosition = 0;
        char comment[MAX_COMMENT_LENGTH];
        memset (comment,0,MAX_COMMENT_LENGTH);
        byte comment_pos=0;

        int16_t n=card.get();
        if (card.errorCode())
            {
                if (!card.sdInserted)
                    {
                        card.release();
                        serial_count = 0;
                        return;
                    }

                //On an error, reset the error, reset the file position and try again.
                card.clearError();
                serial_count = 0;
                //Screw it, if we are near the end of a file with an error, act if the file is finished. Hopefully preventing the hang at the end.
                if (endOfLineFilePosition > card.getFileSize() - 512)
                    card.sdprinting = false;
                else
                    card.setIndex(endOfLineFilePosition);
                return;
            }

        serial_char = (char)n;
        if(serial_char == '\n' ||
                serial_char == '\r' ||
                (serial_char == ':' && comment_mode == false) ||
                serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
            {
                if(card.eof() || n==-1)
                    {
                        SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
                        stoptime=millis();
                        //         unsigned long t=(stoptime-starttime)/1000;
                        card.printingHasFinished();
                        card.checkautostart(true);
                    }
                if(!serial_count)
                    {
                        comment_mode = false; //for new command
                        return; //if empty line
                    }
                cmdbuffer[bufindw][serial_count] = 0; //terminate string
                //      if(!comment_mode){
                fromsd[bufindw] = true;
                buflen += 1;
                bufindw = (bufindw + 1)%BUFSIZE;
                //      }
                comment_mode = false; //for new command
                serial_count = 0; //clear buffer
                endOfLineFilePosition = card.getFilePos();
            }
        else
            {
                if(serial_char == ';')
                    {
                        comment_mode = true;
                        comment_pos = serial_count;
                    }
                if(!comment_mode)
                    cmdbuffer[bufindw][serial_count++] = serial_char;
				//                 else
					{
						// todo --- come back to this, it doesn't work.
					}
//                     if (serial_count-comment_pos < MAX_COMMENT_LENGTH-1) comment[serial_count++] = serial_char;
            }

//         processComment (comment) ;
    }
}






// *******************************************
void get_command()
{
    while( MYSERIAL.available() > 0  && !is_command_queue_full())
        {
            getCommandSerial();
            return;
        }
#ifdef SDSUPPORT
    if(!card.sdprinting || serial_count!=0)
        {
            return;
        }
    if (card.pause)
        {
            return;
        }

    while( !card.eof()  && (!is_command_queue_full()))
        getCommandSDCard();
    return;

#endif //SDSUPPORT

}



//-----------------------------------------------------------------------------------------------------------------
// check the command queue and process commands in it
void manageBuffer()
{
    if(is_command_queued())
        {
#ifdef SDSUPPORT
            if(card.saving)
                {
                    if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
                        {
                            card.write_command(cmdbuffer[bufindr]);
                            if(card.logging)
                                {
                                    process_commands();
                                }
                            else
                                {
                                    SERIAL_PROTOCOLLNPGM(MSG_OK);
                                }
                        }
                    else
                        {
                            card.closefile();
                            SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
                        }
                }
            else
                {
                    process_commands();
                }
#else
            process_commands();
#endif //SDSUPPORT
            if (buflen > 0)
                {
                    buflen = (buflen-1);
                    bufindr = (bufindr + 1)%BUFSIZE;
                }
        }
    else delay(10)			;
}

//-----------------------------------------------------------------------------------------------------------------
void initQueue()
{

    for(int8_t i = 0; i < BUFSIZE; i++)
        {
            fromsd[i] = false;
        }

}

//-----------------------------------------------------------------------------------------------------------------
bool is_command_from_sd( int a )
{
    return fromsd[a];
}



