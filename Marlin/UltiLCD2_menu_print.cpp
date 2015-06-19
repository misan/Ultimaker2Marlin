#include <avr/pgmspace.h>

#include "Configuration.h"
#ifdef ENABLE_ULTILCD2
#include "Marlin.h"
#include "cardreader.h"
#include "temperature.h"
#include "lifetime_stats.h"
#include "UltiLCD2.h"
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_menu_print.h"
#include "UltiLCD2_menu_material.h"
#include "MenuUseful.h"
#include "gcode.h"
#include "stringHelpers.h"
#include "UltiLCD2_menu_maintenance.h"

// #define DEBUG_INFO


// #pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing" // Code that causes warning goes here #pragma GCC diagnostic pop


// how long the TUNE / ABORT stays on the bottom of the printing screen before showing the progress bar
#define PRINT_TUNE_MENU_TIMEOUT 5000

// #pragma GCC diagnostic pop

unsigned int total_layers = 1;

float old_zlift;
float old_retraction ;

float estimated_filament_length_in_m;

char last_print_name[LONG_FILENAME_LENGTH];

extern unsigned char soft_pwm_bed;
extern long position[4];

// introduce a short delay before reading file details so director listings are more responsive...
#define FILE_READ_DELAY 50
int file_read_delay_counter = FILE_READ_DELAY;


//-----------------------------------------------------------------------------------------------------------------//-----------------------------------------------------------------------------------------------------------------
extern float final_e_position;

void abortPrint()
{
#ifdef DEBUG_INFO
    SERIAL_ECHO_START ;
    SERIAL_ECHOLNPGM(("ABORTING"));
#endif

    /// stop any printing that's in the queue -- either from planner or the serial buffer
    plan_discard_all_blocks();
    clear_command_queue();
    stoptime = millis();
    delay (20);
    char buffer[30];
    memset (buffer, 0, sizeof(buffer));
    card.pause = false;		// otherwise abort when pause will hang
    card.sdprinting = false;
    final_e_position = true_e_position + extruderPositionInMM( current_position[E_AXIS]);
    true_e_position = 0;

    // set up the end of print retraction
#ifdef DEBUG_INFO
    SERIAL_ECHOLNPGM(("ABORT: SENDING MOTION COMMANDS"));
#endif
    sprintf_P(buffer, PSTR("G92 E%i"), int(((float)END_OF_PRINT_RETRACTION) / volume_to_filament_length[active_extruder]));
    enquecommand(buffer);
    // perform the retraction at the standard retract speed
//	delay (50);
    sprintf_P(buffer, PSTR("G1 F%i E0"), int(retract_feedrate));
    enquecommand(buffer);//
//	delay (50);
    enquecommand_P(PSTR("G28"));
    enquecommand_P(PSTR("M84"));
#ifdef DEBUG_INFO
    SERIAL_ECHOLNPGM(("ABORT: MOTION COMMANDS SENT"));
#endif
    postMenuCheck = NULL;
    lifetime_stats_print_end();
    doCooldown();
    enquecommand_P(PSTR("M300 S220 P150"));
}


void checkPrintFinished()
{
    if (!card.sdprinting && !is_command_queued())
    {
#ifdef DEBUG_INFO
        SERIAL_ECHO_START ;
        SERIAL_ECHOLNPGM(("SD CARD DONE, NO COMMANDS IN QUEUE"));
#endif
        abortPrint();
        lcd_change_to_menu(lcd_menu_print_ready, ENCODER_NO_SELECTION, false);
        SELECT_MAIN_MENU_ITEM(0);
        enquecommand_P(PSTR("M300 S440 P250"));
    }
    if (card.errorCode())
    {
#ifdef DEBUG_INFO
        SERIAL_ECHO_START ;
        SERIAL_ECHOLNPGM(("SD CARD ERROR"));
#endif
        abortPrint();
        enquecommand_P(PSTR("M300 S110 P250"));
        lcd_change_to_menu(lcd_menu_print_error, ENCODER_NO_SELECTION, false);

        SELECT_MAIN_MENU_ITEM(0);
    }
}


void doStartPrint()
{
#ifdef DEBUG_INFO
    SERIAL_ECHO_START ;
    SERIAL_ECHOLNPGM(("doSTART"));
#endif
    if (card.sdprinting ) return;
    PI_R2 = ((PI * ((material[0].diameter / 2) * (material[0].diameter / 2))));
    current_position[E_AXIS] = 0.0;
    plan_set_e_position(0);

    // since we are going to prime the nozzle, forget about any G10/G11 retractions that happened at end of previous print
    retracted = false;
#ifdef RAISE_BED_ON_START
    current_position[Z_AXIS] = 20.0;
#endif
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[Z_AXIS], 0);

    for (uint8_t e = 0; e < EXTRUDERS; e++)
    {
        if (!LCD_DETAIL_CACHE_MATERIAL(e))
        {
            // don't prime the extruder if it isn't used in the (Ulti)gcode
            // traditional gcode files typically won't have the Material lines at start, so we won't prime for those
            continue;
        }
        active_extruder = e;


        // undo the end-of-print retraction
        plan_set_e_position((0.0 - END_OF_PRINT_RETRACTION) / volume_to_filament_length[e]);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], END_OF_PRINT_RECOVERY_SPEED, e);

        // perform additional priming
        plan_set_e_position(-PRIMING_MM3);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], (PRIMING_MM3_PER_SEC * volume_to_filament_length[e]), e);

        // for extruders other than the first one, perform end of print retraction
#if EXTRUDERS>1
        if (e > 0)
        {

            plan_set_e_position((0.0 - END_OF_PRINT_RETRACTION) / volume_to_filament_length[e]);
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], END_OF_PRINT_RECOVERY_SPEED, e);



            //    plan_set_e_position((END_OF_PRINT_RETRACTION) / volume_to_filament_length[e]);
            //   plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], retract_feedrate/60, e);
        }
#endif
    }
    active_extruder = 0;

    postMenuCheck = checkPrintFinished;
    LCD_MESSAGEPGM(PSTR("HERE WE GO!"));
    // lcd_update();
    enquecommand_P(PSTR("M300 S440 P100"));
    enquecommand_P(PSTR("M300 S660 P150"));
    enquecommand_P(PSTR("M300 S880 P150"));
//     lcd_lib_beep_ext(440,100);
//     lcd_lib_beep_ext(660,150);
//     lcd_lib_beep_ext(880,150);
    card.startFileprint();
    last_user_interaction = starttime = millis();
    stoptime = 0;
    lifetime_stats_print_start();
    starttime = millis();
    final_e_position = 0;
    true_e_position = 0;
#ifdef DEBUG_INFO
    SERIAL_ECHO_START ;
    SERIAL_ECHOLNPGM(("do START done"));
#endif


}

void cardUpdir()
{
    card.updir();
}

char* lcd_sd_menu_filename_callbackX(uint8_t nr)
{
    //This code uses the card.longFilename as buffer to store the filename, to save memory.
    if (nr == 0)
    {
        if (card.atRoot())
            strcpy_P(card.longFilename, PSTR("< RETURN"));
        else
            strcpy_P(card.longFilename, PSTR("< BACK"));
    }
    else
    {
        card.longFilename[0] = '\0';
        for (uint8_t idx = 0; idx < LCD_CACHE_MAX_FILES_CACHED; idx++)
        {
            if (LCD_CACHE_ID(idx) == nr)
            {
                strncpy(card.longFilename, LCD_CACHE_FILENAME(idx), MAX_DISPLAY_FILENAME_LEN);
                card.longFilename[MAX_DISPLAY_FILENAME_LEN] = 0;
            }
        }
        if (card.longFilename[0] == '\0')
        {
            card.getfilename(nr - 1);
            if (!card.longFilename[0])
                strcpy(card.longFilename, card.filename);
            if (!card.filenameIsDir)
            {
                if (strchr(card.longFilename, '.')) strrchr(card.longFilename, '.')[0] = '\0';
            }

            uint8_t idx = nr % LCD_CACHE_MAX_FILES_CACHED;
            LCD_CACHE_ID(idx) = nr;
            strncpy(LCD_CACHE_FILENAME(idx), card.longFilename, MAX_DISPLAY_FILENAME_LEN);
            LCD_CACHE_FILENAME(idx)[MAX_DISPLAY_FILENAME_LEN - 1] = 0;
            LCD_CACHE_TYPE(idx) = card.filenameIsDir ? 1 : 0;
            if (card.errorCode() && card.sdInserted)
            {
                //On a read error reset the file position and try to keep going. (not pretty, but these read errors are annoying as hell)
                card.clearError();
                LCD_CACHE_ID(idx) = 255;
                card.longFilename[0] = '\0';
            }
        }
    }
    return card.longFilename;
}
//-----------------------------------------------------------------------------------------------------------------

char* lcd_sd_filemenu_getString(uint8_t nr)
{
    run_history = false;
    if (nr == 0)
    {
        if (card.atRoot())
            strcpy_P(card.longFilename, PSTR("< RETURN"));
        else
            strcpy_P(card.longFilename, PSTR("< BACK"));
        return card.longFilename;
    }
    card.longFilename[0] = '\0';
    for (uint8_t idx = 0; idx < LCD_CACHE_MAX_FILES_CACHED; idx++)		// check for a cached filename... do we have it in our cache?
    {
        if (LCD_CACHE_ID(idx) == nr)					// we do!
        {
            strcpy(card.longFilename, LCD_CACHE_FILENAME(idx));			// copy it
            card.longFilename[MAX_DISPLAY_FILENAME_LEN] = 0;
            return card.longFilename;
        }
    }

    // OK, lets grab the filename and cache it for later.
    card.getfilename(nr - 1);
    processLongFilename();

    uint8_t idx = nr % LCD_CACHE_MAX_FILES_CACHED;
    LCD_CACHE_ID(idx) = nr;
    strncpy(LCD_CACHE_FILENAME(idx), card.longFilename, MAX_DISPLAY_FILENAME_LEN);
    LCD_CACHE_FILENAME(idx)[MAX_DISPLAY_FILENAME_LEN - 1] = 0;
    LCD_CACHE_TYPE(idx) = card.filenameIsDir ? 1 : 0;
    if (card.errorCode() && card.sdInserted)
    {
        //On a read error reset the file position and try to keep going. (not pretty, but these read errors are annoying as hell)
        card.clearError();
        LCD_CACHE_ID(idx) = 255;
        card.longFilename[0] = '\0';
    }

    return card.longFilename;
}

//-----------------------------------------------------------------------------------------------------------------
void updateFileDetails( uint8_t nr, char* filename);

void lcd_sd_filemenu_getDetails(uint8_t nr)
{
    run_history = false;
    if (nr == 0)
        return;
    for (uint8_t idx = 0; idx < LCD_CACHE_MAX_FILES_CACHED; idx++)
    {
        if (LCD_CACHE_ID(idx) == nr)
        {
            if (LCD_CACHE_TYPE(idx) == 1)
                lcd_lib_draw_string_centerP(53, PSTR("Folder"));
            else
            {
                if ( LCD_DETAIL_CACHE_ID() != nr)
                {
                    if (file_read_delay_counter > 0)
                        file_read_delay_counter --;
                    if (file_read_delay_counter > 0) return;		// wait, don't read yet, we may just be scanning through the list quickly....
                    file_read_delay_counter = FILE_READ_DELAY;						// but don't make the wait too long - we don't want them to select a file without having hit this block
                    card.getfilename(nr - 1);
                    if (card.errorCode())
                    {
                        card.clearError();
                        return;
                    }
                    updateFileDetails(nr, card.filename);
                }

                if (LCD_DETAIL_CACHE_TIME() > 0)
                {
                    char buffer[32];
                    memset (buffer, 0, sizeof(buffer));
                    char* c = buffer;
                    if (time_phase_a(4))
                    {
                        strcpy_P(c, PSTR("Time: "));
                        c += 6;
                        c = int_to_time_string(LCD_DETAIL_CACHE_TIME(), c);
                    }
                    if (time_phase_c(4))
                    {
                        strcpy_P(c, PSTR("Total layers: "));
                        c += 14;
                        c = int_to_string(LCD_CACHE_DETAIL.total_layers , c);
                    }
                    if (time_phase_d(4))
                    {
// 										strcpy_P(c, PSTR("Date:"));
// 										c += 5;
                        c = int_to_string(FAT_MONTH(LCD_CACHE_DETAIL.datestamp ), c, PSTR("/"));
                        c = int_to_string(FAT_DAY(LCD_CACHE_DETAIL.datestamp ), c, PSTR("/"));
                        c = int_to_string(FAT_YEAR(LCD_CACHE_DETAIL.datestamp ), c, PSTR(" "));

                        char hour = FAT_HOUR(LCD_CACHE_DETAIL.timestamp);
#if USE_12_HOUR_CLOCK_FORMAT

                        bool pm = hour >= 12;
                        if (hour > 12) hour -= 12;
                        if (hour == 0) hour = 12;
#endif
                        if (hour < 10) *c++ = '0';
                        c = int_to_string(hour, c, PSTR(":"));
                        if (FAT_MINUTE(LCD_CACHE_DETAIL.timestamp) < 10) *c++ = '0';
                        c = int_to_string(FAT_MINUTE(LCD_CACHE_DETAIL.timestamp ), c, PSTR(":"));
                        if (FAT_SECOND(LCD_CACHE_DETAIL.timestamp) < 10) *c++ = '0';
                        c = int_to_string(FAT_SECOND(LCD_CACHE_DETAIL.timestamp ), c);
#if USE_12_HOUR_CLOCK_FORMAT
                        if (pm)
                            *c++ = 'P';
                        else
                            *c++ = 'A';
                        *c++ = 'M';
#endif
                        *c++ = 0;

                    }

                    if (time_phase_b(4))
                    {
                        strcpy_P(c, PSTR("Material: "));
                        c += 10;
                        estimated_filament_length_in_m = float(LCD_DETAIL_CACHE_MATERIAL(0)) / (M_PI * (material[0].diameter / 2.0) * (material[0].diameter / 2.0)) / 1000.0;
                        if (estimated_filament_length_in_m < 10)
                            c = float_to_string(estimated_filament_length_in_m , c, PSTR("m"));
                        else
                            c = int_to_string(estimated_filament_length_in_m , c, PSTR("m"));
#if EXTRUDERS > 1
                        if (LCD_DETAIL_CACHE_MATERIAL(1))
                        {
                            *c++ = '/';
                            float length = float(LCD_DETAIL_CACHE_MATERIAL(1)) / (M_PI * (material[1].diameter / 2.0) * (material[1].diameter / 2.0));
                            if (length < 10000)
                                c = float_to_string(length / 1000.0, c, PSTR("m"));
                            else
                                c = int_to_string(length / 1000.0, c, PSTR("m"));
                        }
#endif
                    }
                    lcd_lib_draw_string(3, 53, buffer);
                }
                else
                    lcd_lib_draw_stringP(3, 53, PSTR("No info available"));
            }
        }
    }
}
//-----------------------------------------------------------------------------------------------------------------

const byte LINES_IN_ULTICODE_HEADER = 8;

//-----------------------------------------------------------------------------------------------------------------
void updateFileDetails( uint8_t nr, char* filename)
{
    char buffer[48];
    memset (buffer, 0, sizeof(buffer));

    LCD_DETAIL_CACHE_ID() = nr;
    LCD_DETAIL_CACHE_TIME() = 0;
    for (uint8_t e = 0; e < EXTRUDERS; e++)
        LCD_DETAIL_CACHE_MATERIAL(e) = 0;

#ifdef DEBUG_INFO
    SERIAL_ECHO_START ;
    SERIAL_ECHOPGM("Pre-Reading details on ");
    SERIAL_ECHO(filename);
    SERIAL_ECHOPGM(" longname:");
    SERIAL_ECHO(card.longFilename);
    SERIAL_ECHOLN("");
#endif
    strcpy (buffer, card.longFilename);

    card.openFile(filename, true);		// for SOME reason, this overwrites longfilename with the wrong value!
    strcpy (card.longFilename, buffer);
    if (card.isFileOpen())
    {

        LCD_CACHE_DETAIL.datestamp = card.datestamp;
        LCD_CACHE_DETAIL.timestamp = card.timestamp;


// 		if (! card.longFilename[0])		// is it non zerto length? copy iy to our buffer
// 			strcpy (card.longFilename,card.filename);

#ifdef DEBUG_INFO

        SERIAL_ECHO_START ;
        SERIAL_ECHOPGM("Reading details on ");
        SERIAL_ECHO(filename);
        SERIAL_ECHOPGM(" longname:");
        SERIAL_ECHO(card.longFilename);
        SERIAL_ECHOLN("");
#endif
        // read tghe first 8 LINES
        for (uint8_t n = 0; n < LINES_IN_ULTICODE_HEADER; n++)
        {
            card.fgets(buffer, sizeof(buffer));
            buffer[sizeof(buffer) - 1] = '\0';
            while (strlen(buffer) > 0 && buffer[strlen(buffer) - 1] < ' ') buffer[strlen(buffer) - 1] = '\0';
            if (strncmp_P(buffer, PSTR(";Layer count: "), 14) == 0)
                LCD_CACHE_DETAIL.total_layers = atol(buffer + 14);
            if (strncmp_P(buffer, PSTR(";TIME:"), 6) == 0)
                LCD_CACHE_DETAIL.estimated_print_time = atol(buffer + 6);
            else
                if (strncmp_P(buffer, PSTR(";MATERIAL:"), 10) == 0)
                {
                    LCD_CACHE_DETAIL.material[0] = atol(buffer + 10);
                    estimated_filament_length_in_m = float(LCD_CACHE_DETAIL.material[0]) / (M_PI * (material[0].diameter / 2.0) * (material[0].diameter / 2.0)) / 1000.0;
                }
#if EXTRUDERS > 1
                else
                    if (strncmp_P(buffer, PSTR(";MATERIAL2:"), 11) == 0)
                        LCD_DETAIL_CACHE_MATERIAL(1) = atol(buffer + 11);
#endif
        }
    }
    if (card.errorCode())
    {
        //On a read error reset the file position and try to keep going. (not pretty, but these read errors are annoying as hell)
        card.clearError();
        LCD_DETAIL_CACHE_ID() = 255;
    }
    card.closefile();
}


void lcd_sd_filemenu_doAction()
{
    if (!lcd_lib_update_ready()) return;
    static bool beeped = false;
    if (!card.sdInserted)
    {
        // beep, but only once
        LED_GLOW_ERROR();
        if (!beeped) ERROR_BEEP();
        beeped = true;
        lcd_lib_encoder_pos = MAIN_MENU_ITEM_POS(0);
        lcd_info_screen(lcd_menu_main);
        lcd_lib_draw_string_centerP(15, PSTR("No SD-CARD!"));
        lcd_lib_draw_string_centerP(25, PSTR("Please insert card"));
        lcd_lib_update_screen();
        card.release();
        return;
    }
    if (!card.isOk())
    {
        lcd_info_screen(lcd_menu_main);
        lcd_lib_draw_string_centerP(16, PSTR("Reading card..."));
        lcd_lib_update_screen();
        lcd_cache_new.clear();
        LED_CLEAR_ERROR();
        card.initsd();
        return;
    }

    if (LCD_CACHE_NR_OF_FILES() == 0xFF)
        LCD_CACHE_NR_OF_FILES() = card.getnrfilenames();
    if (card.errorCode())
    {
        LED_GLOW_ERROR();
        if (!beeped) ERROR_BEEP();
        beeped = true;
        LCD_CACHE_NR_OF_FILES() = 0xFF;
        return;
    }
    LED_NORMAL();
    beeped = false;
    uint8_t nrOfFiles = LCD_CACHE_NR_OF_FILES();
    if (nrOfFiles == 0)
    {
        if (card.atRoot())
            lcd_info_screen(lcd_menu_main, NULL, PSTR("OK"));
        else
            lcd_info_screen(lcd_sd_filemenu_doAction, cardUpdir, PSTR("OK"));
        lcd_lib_draw_string_centerP(25, PSTR("No files found!"));
        lcd_lib_update_screen();
        lcd_cache_new.clear();
        return;
    }

    if (lcd_lib_button_pressed())
    {
        uint8_t selIndex = uint16_t(SELECTED_SCROLL_MENU_ITEM());
        if (selIndex == 0)
        {
            if (card.atRoot())
            {
                lcd_change_to_menu(lcd_menu_main);
                return;
            }
            else
            {
                SELECT_SCROLL_MENU_ITEM(0);
                lcd_cache_new.clear();
                lcd_lib_beep();
                card.updir();
            }
        }
        else
        {
            card.getfilename(selIndex - 1);
            if (!card.filenameIsDir)
            {
                char buffer[48];
                //Start print
                updateFileDetails(selIndex - 1, card.filename);
                active_extruder = 0;
                strcpy (buffer, card.longFilename);

                card.openFile(card.filename, true);		// for SOME reason, this overwrites longfilename with the wrong value!
                strcpy (card.longFilename, buffer);



                if (card.isFileOpen() && !is_command_queued())
                {
                    if (led_mode == LED_MODE_WHILE_PRINTING || led_mode == LED_MODE_BLINK_ON_DONE)
                        analogWrite(LED_PIN, 255 * int(led_brightness_level) / 100);

                    processLongFilename();

                    strcpy (last_print_name, card.longFilename);
                    card.fgets(buffer, sizeof(buffer));
                    buffer[sizeof(buffer) - 1] = '\0';
                    while (strlen(buffer) > 0 && buffer[strlen(buffer) - 1] < ' ') buffer[strlen(buffer) - 1] = '\0';
                    if (strcmp_P(buffer, PSTR(";FLAVOR:UltiGCode")) != 0)
                    {
                        card.fgets(buffer, sizeof(buffer));
                        buffer[sizeof(buffer) - 1] = '\0';
                        while (strlen(buffer) > 0 && buffer[strlen(buffer) - 1] < ' ') buffer[strlen(buffer) - 1] = '\0';
                    }
                    card.setIndex(0);
                    clearHistory();
                    run_history = true;
                    if (strcmp_P(buffer, PSTR(";FLAVOR:UltiGCode")) == 0)
                    {
                        prepareToPrintUltiGCode();
                        lcd_change_to_menu(lcd_menu_print_heatup);
                    }
                    else
                    {
                        estimatedTime = 1000;
                        lcd_lib_beep_ext(330, 150);
                        //Classic gcode file
                        //Set the settings to defaults so the classic GCode has full control
                        fanSpeedPercent = 100;
                        for (uint8_t e = 0; e < EXTRUDERS; e++)
                        {
                            volume_to_filament_length[e] = 1.0;
                            extrudemultiply[e] = 100;
                        }
                        lcd_change_to_menu(lcd_menu_print_classic_warning, MAIN_MENU_ITEM_POS(0));
                        //     strcpy (last_print_name,card.longFilename);
                    }
                }
            }
            else
            {
                lcd_lib_beep();
                lcd_cache_new.clear();
                card.chdir(card.filename);
                SELECT_SCROLL_MENU_ITEM(0);
            }
            return;//Return so we do not continue after changing the directory or selecting a file. The nrOfFiles is invalid at this point.
        }
    }
    lcd_scroll_menu(PSTR("SD CARD"), nrOfFiles + 1, lcd_sd_filemenu_getString, lcd_sd_filemenu_getDetails);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_print_heatup()
{
    if (!lcd_lib_update_ready()) return;
    bool draw_filename = true;
    if (millis() - last_user_interaction < PRINT_TUNE_MENU_TIMEOUT)
    {
        lcd_question_screen(lcd_menu_print_tune_doAction, NULL, PSTR("TUNE"), lcd_menu_print_abort, NULL, PSTR("ABORT"));
        draw_filename = false;
    }
    else
    {
        lcd_lib_encoder_pos = ENCODER_NO_SELECTION;
        lcd_lib_clear ();
        //  lcd_basic_screen();
    }
    starttime = stoptime = millis();		// kept the timers paused
    if (current_temperature_bed > target_temperature_bed - 10 && target_temperature_bed > 5)
    {
        for (uint8_t e = 0; e < EXTRUDERS; e++)
        {
            if (LCD_DETAIL_CACHE_MATERIAL(e) < 1 || target_temperature[e] > 0)
                continue;
            if (target_temperature[e] != material[e].temperature)
            {
                analogWrite(LED_PIN, 0);
                lcd_lib_beep_ext(600, 50);
                delay (100);
                lcd_lib_beep_ext(660, 50);
                analogWrite(LED_PIN, 255 * led_brightness_level / 100);
                LCD_MESSAGEPGM(PSTR("HEATING EXTRUDER"));
            }
            target_temperature[e] = material[e].temperature;
        }
        if (current_temperature_bed >= target_temperature_bed - TEMP_WINDOW * 2 && !is_command_queued())
        {
            bool ready = true;
            for (uint8_t e = 0; e < EXTRUDERS; e++)
                if (current_temperature[e] < target_temperature[e] - TEMP_WINDOW)
                    ready = false;
            if (ready)
            {
                doStartPrint();
                lcd_change_to_menu(lcd_menu_print_printing, ENCODER_NO_SELECTION, false);
#ifdef DEBUG_INFO
                SERIAL_ECHO_START ;
                SERIAL_ECHOLNPGM(("Is Ready"));
#endif
                return;
            }
        }
    }

    uint8_t progress = 125;
    for (uint8_t e = 0; e < EXTRUDERS; e++)
    {
        if (LCD_DETAIL_CACHE_MATERIAL(e) < 1 || target_temperature[e] < 1)
            continue;
        if (current_temperature[e] > 20)
            progress = min(progress, (current_temperature[e] - 20) * 125 / (target_temperature[e] - 20 - TEMP_WINDOW));
        else
            progress = 0;
    }
    if (target_temperature_bed > BED_MINTEMP)
    {
        if (current_temperature_bed > 20)							//  what if we're printing without hearing the bed?
            progress = min(progress, (current_temperature_bed - 20) * 125 / (target_temperature_bed - 20 - TEMP_WINDOW));
        else
            progress = 0;
    }
    if (progress < minProgress)
        progress = minProgress;
    else
        minProgress = progress;

    char buffer[25];
    char* c;

    c = int_to_string(current_temperature[active_extruder], buffer/*, PSTR( DEGREE_C_SYMBOL )*/);
    *c++ = TEMPERATURE_SEPARATOR;
    c = int_to_string(target_temperature[active_extruder], c, PSTR( DEGREE_C_SYMBOL ), true);
    *c++ = ' ';
    *c++ = ' ';
    *c++ = ' ';
    c = int_to_string(current_temperature_bed, c/*, PSTR( DEGREE_C_SYMBOL )*/);
    *c++ = TEMPERATURE_SEPARATOR;
    c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ), true);
    *c++ = 0;
    lcd_lib_draw_string_center(HALF_ROW(ROW1), buffer);

    run_history = true;
    drawTempHistory (15 + 3, ROW2 + 3, DISPLAY_RIGHT / 3 + 3 + 15, ROW6 - 3, lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.temp_history);
    drawTempHistory (2 * DISPLAY_RIGHT / 3 - 3 - 15, ROW2 + 3, DISPLAY_RIGHT - 3 - 15, ROW6 - 3, lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.bed_history);

    if (draw_filename)
    {
        c = buffer;
        c = float_to_string((estimated_filament_length_in_m) , c, PSTR ("m of "));
        strcpy (c, material_name[active_extruder]);
        lcd_lib_draw_string_center(ROW6, buffer);

        //  lcd_lib_draw_string_centerP(ROW6, PSTR("Heating up..."));
        lcd_lib_draw_string_center(ROW7, last_print_name);
        lcd_lib_invert(3, ROW7 - 1, progress, DISPLAY_BOTTOM);
    }

//	lcd_progressbar(progress);
    lcd_lib_update_screen();
    LED_HEAT();
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//			LEFT SIDE					RIGHT SIDE
// ROW1 =     EXT0 TEMP					 bed temp
// ROW2 =     Temp GRAPH				  FAN
// ROW3 =     Temp Graph    Heater power		MOVE BUFFER
// ROW4 =   extrusion volume				 linear speed
// ROW5  =   FLOW  percent				Speed percent
// if there is no menu shown, then:
// ROW6 =   filament / time used				filament / time to go
// ROW7 =  Progress bar overlaid with MESSAGE OR FILENAME  or Z Height
//
// ROW7 will show a message (if there is a recent one) or alternate between the filename and Zheight




//-----------------------------------------------------------------------------------------------------------------
// if we've got a slow buffer, don't draw the UI as often -- but still don't block it completely
// buffer under half, draw 1/4 of the time
// under one quarter, draw 1/8 of the time
// almost empty, draw if  1/16 of the time.
#define SKIP true
#define DONTSKIP false

bool skipDrawingUI()
{
// return DONTSKIP;
    static byte slow_buffer_counter = 0;
    if (lcd_lib_button_pressed()) return DONTSKIP;
    if (card.pause) return DONTSKIP;
	byte moves = movesplanned() ;
    // show pink or red if the movement buffer is low / dry
    if (current_block == NULL || moves < 1) { analogWrite(LED_PIN, 0); return SKIP ; }
    if (moves < 2) { slow_buffer_counter += 1; lcd_lib_led_color(255, 0, 0); lcd_lib_update_RGB_LED();}
    else
        if (moves < BLOCK_BUFFER_SIZE / 4) { slow_buffer_counter += 2; lcd_lib_led_color(255, 0, 160); lcd_lib_update_RGB_LED();}
        else
            if (moves < BLOCK_BUFFER_SIZE / 2) {  slow_buffer_counter += 4;  lcd_lib_led_color(192, 32, 192); lcd_lib_update_RGB_LED();}
			   else return DONTSKIP;
    if (slow_buffer_counter < 16) return SKIP;
    slow_buffer_counter = 0;
    return DONTSKIP;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_print_printing()
{
    if (!lcd_lib_update_ready()) return;
    if (skipDrawingUI()) return;
    run_history = true;
    bool draw_filename = true;
    if (millis() - last_user_interaction < PRINT_TUNE_MENU_TIMEOUT)
    {
        lcd_question_screen(lcd_menu_print_tune_doAction, NULL, PSTR("TUNE"), lcd_menu_print_abort, NULL, PSTR("ABORT"));
        draw_filename = false;
    }
    else
    {
        lcd_lib_encoder_pos = ENCODER_NO_SELECTION;
        lcd_lib_clear ();
        //  lcd_basic_screen();
    }
    uint8_t progress = card.getFilePos() / ((card.getFileSize() + 123) / 124);
    char buffer[40];
    memset(buffer, 0, sizeof(buffer));
    char* c = buffer;
    switch (printing_state)
    {
        default:
            {
                LED_NORMAL();
                // these are used to maintain a simple low-pass filter on the speeds
                static float e_smoothed_speed = 0.0;
                static float xy_speed = 0.0;
                calculateSpeeds(xy_speed, e_smoothed_speed);

                drawSpeedAndFlow(buffer, c, ROW5);

                // Show the extruder temperature and target temperature:
                c = buffer/* + 3*/;
                c = drawCurrentTemp(c);
                *c++ = 0;
                lcd_lib_draw_string(3, ROW3, buffer);


                c = buffer;
                if (soft_pwm_bed == 0) *c++ = 'x';
                else
                {
                    if (time_phase0) *c++ = 'o';
                    else *c++ = 'O';
                }
                *c++ = ' ';

                c = int_to_string(current_temperature_bed, c, PSTR( TEMPERATURE_SEPARATOR_S ));
                c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ));
                *c++ = 0;

                lcd_lib_draw_string_right(ROW2, buffer);

                // show the extrusion rate
                c = buffer;
                *c = 32;
                if (e_smoothed_speed < 10.0) c++;
                c = float_to_string( e_smoothed_speed, c, PSTR ("mm" CUBED_SYMBOL  PER_SECOND_SYMBOL ));
                *c++ = 0;
                lcd_lib_draw_string(5, ROW4, buffer);

                // show the xy travel speed
                c = buffer;
                c = int_to_string( round(xy_speed), c, PSTR ("mm" PER_SECOND_SYMBOL ));		// we don't need decimal places here.
                *c++ = 0;
                lcd_lib_draw_string_right(ROW4, buffer);

                // fan speed
                drawMiniBargraph (DISPLAY_RIGHT - (3 + 2 + 32), ROW1 + 1, DISPLAY_RIGHT, ROW2 - 2, (float) getFanSpeed() / 255.0);
                if (fanSpeedOverride >= 1)
                    strcpy_P(buffer, PSTR ("OVR"));
                else
                    strcpy_P(buffer, PSTR ("FAN"));
                // HISTORY AND HEATER POWER GRAPHS
                lcd_lib_draw_string_right(ROW1 + 1, buffer, DISPLAY_RIGHT - (3 + 2 + 32));
                drawTempHistory (3, ROW1, DISPLAY_RIGHT / 3 + 3, ROW3 - 2, lcd_cache_new.getData(LCD_CACHE::TEMPERATURE_HISTORY).temphist.temp_history);
                lcd_lib_draw_box(DISPLAY_RIGHT / 2 - 10, ROW1, DISPLAY_RIGHT / 2 + 6 - 10, ROW3 - 2);
                int g =  getHeaterPower(active_extruder);
                g *= (ROW_HEIGHT * 2) - 2;
                g >>= 7;
                lcd_lib_set (DISPLAY_RIGHT / 2 + 2 - 10, ROW3 - g , DISPLAY_RIGHT / 2 + 4 - 10, ROW3 - 2);

                // the movement buffer  depth
                strcpy_P(buffer, PSTR ("BUF"));
                lcd_lib_draw_string_right(ROW3, buffer, DISPLAY_RIGHT - (3 + 2 + 32));
                drawMiniBargraph (DISPLAY_RIGHT - (3 + 2 + 32), ROW3, DISPLAY_RIGHT, ROW4 - 2, (float) movesplanned() / (BLOCK_BUFFER_SIZE - 1));


            }
            break;
        case PRINT_STATE_WAIT_USER:
            // get the user's attention by flashing the control knob LED, clicking, and disabling the automatic LED lighting dimming
            LED_FLASH();
            if (led_glow == 128) lcd_lib_tick();
            last_user_interaction = millis();
            lcd_lib_encoder_pos = ENCODER_NO_SELECTION;
            lcd_lib_draw_string_centerP(ROW3, PSTR("Press button"));
            // show a message, if we have one.  Otherwise, use a default.
            if (!lcd_lib_show_message (ROW5, false))
                lcd_lib_draw_string_centerP(ROW4, PSTR("to continue"));
            break;
        case PRINT_STATE_HEATING:
            LED_HEAT();
            lcd_lib_draw_string_centerP(ROW2, PSTR("Heating"));
//                 c = int_to_string(current_temperature[0], buffer/*, PSTR( DEGREE_C_SYMBOL )*/);
//                 *c++ = TEMPERATURE_SEPARATOR;
            c = buffer;
            c = drawCurrentTemp(c);
            lcd_lib_draw_string_center(ROW4, buffer);
            break;
        case PRINT_STATE_HEATING_BED:
            LED_HEAT();
            lcd_lib_draw_string_centerP(ROW2, PSTR("Heating buildplate"));
            c = buffer;
            c = drawCurrentBedTemp(c);
            lcd_lib_draw_string_center(ROW4, buffer);
            break;
    }

    static long timeLeftSec = estimatedTime;
    unsigned long printTimeMs = (millis() - starttime);
    timeLeftSec = calculateRemainingTime(printTimeMs, timeLeftSec);


    if (draw_filename)
    {
        // draw time and filament progress
        lcd_lib_draw_dotted_hline(3, 125, ROW6 - 1);
        lcd_lib_draw_hline (3, 125, ROW7 - 1);
        lcd_lib_draw_vline(3, ROW7, DISPLAY_BOTTOM);
        lcd_lib_draw_vline(125, ROW7, DISPLAY_BOTTOM);

#pragma GCC diagnostic ignored "-Wmaybe-uninitialized" // Code that causes warning goes here 
        float pos ;
        if (time_phase0)
            EchoTimeSpan(round (printTimeMs / 1000), buffer);
        else
        {
            pos  = true_e_position + extruderPositionInMM(current_position[E_AXIS]);
            pos /= 1000.0;
            c = float_to_string3(pos, buffer, PSTR("m"));
            *c++ = 0;
        }

        lcd_lib_draw_string(5, ROW6, buffer);

        if (time_phase0)
            EchoTimeSpan (timeLeftSec, buffer);
        else
        {
            c = float_to_string3((estimated_filament_length_in_m) - pos, buffer, PSTR("m"));
            *c++ = 0;
        }
#pragma GCC diagnostic pop

        lcd_lib_draw_string_right(ROW6, buffer);

        // bottom  row - show any M117 GCODE messages, or if none, then  alternate between time remaining and currently printing file, switch every 2 seconds
        if (!lcd_lib_show_message (ROW7 + 1))
        {
            if (time_phase0)
                lcd_lib_draw_string_center(ROW7 + 1, last_print_name);
            else
            {
                // show the Z height
                buffer[0] = 'Z';
                buffer[1] = '=';
                c = buffer + 2;
                c = float_to_string(current_position[Z_AXIS], c, PSTR ("mm"));
                *c++ = 0;
                lcd_lib_draw_string_center(ROW7 + 1, buffer);

            }
            lcd_lib_invert(3, ROW7 - 1, progress, 63);
        }
    }
    lcd_lib_update_screen();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_print_error()
{
    lcd_lib_wait_for_screen_ready();
    LED_GLOW_ERROR();
    if (!did_beep) ERROR_BEEP();
    did_beep = true;
    lcd_info_screen(lcd_menu_main, NULL, PSTR("RETURN TO MAIN"));

    lcd_lib_draw_string_centerP(10, PSTR("Error while"));
    lcd_lib_draw_string_centerP(20, PSTR("reading"));
    lcd_lib_draw_string_centerP(30, PSTR("SD-card!"));
    char buffer[12];
    strcpy_P(buffer, PSTR("Code:"));
    char*   c = int_to_string(card.errorCode(), buffer + 5);
    *c++ = 0;
    lcd_lib_draw_string_center(40, buffer);
#ifdef DEBUG_INFO
    SERIAL_ERROR_START ;
    SERIAL_ECHOLNPGM(("SD CARD ERROR"));
#endif

    lcd_lib_update_screen();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_print_classic_warning()
{
    lcd_question_screen(lcd_menu_print_printing, doStartPrint, PSTR("CONTINUE"), lcd_sd_filemenu_doAction, NULL, PSTR("CANCEL"));
#ifdef DEBUG_INFO
    SERIAL_ECHO_START ;
    SERIAL_ECHOLNPGM(("CLASSIC GCODE WARNING"));
#endif
    lcd_lib_draw_string_centerP(10, PSTR("This file will"));
    lcd_lib_draw_string_centerP(20, PSTR("override machine"));
    lcd_lib_draw_string_centerP(30, PSTR("setting with setting"));
    lcd_lib_draw_string_centerP(40, PSTR("from the slicer."));

    lcd_lib_update_screen();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_print_abort()
{
    if (!lcd_lib_update_ready()) return;

    LED_FLASH();
    lcd_question_screen(lcd_menu_print_ready, abortPrint, PSTR("YES"), NULL, NULL, PSTR("NO"));
    lcd_lib_draw_string_centerP(20, PSTR("Abort the print?"));
#ifdef DEBUG_INFO
    SERIAL_ECHO_START ;
    SERIAL_ECHOLNPGM(("MANUAL ABORT!"));
#endif
    lcd_lib_update_screen();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void postPrintReady()
{
    if (led_mode == LED_MODE_BLINK_ON_DONE)
        analogWrite(LED_PIN, 0);
}
//-----------------------------------------------------------------------------------------------------------------

#define SKIP_END_SCREEN 1

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_print_ready()
{
    if (!lcd_lib_update_ready()) return;

    run_history = true;
    if (stoptime == 0) stoptime = millis();
    if (stoptime < starttime) starttime = stoptime;
    last_user_interaction = millis();
    if (led_mode == LED_MODE_WHILE_PRINTING)
        analogWrite(LED_PIN, 0);
    else
        if (led_mode == LED_MODE_BLINK_ON_DONE)
            analogWrite(LED_PIN, (led_glow << 1) * int(led_brightness_level) / 100);

    if (SKIP_END_SCREEN && (millis() - stoptime > 10000))	// ten seconds to let retraction and homing , etc
    {
        LED_NORMAL();
        printDoneBeep();
        lcd_change_to_menu(lcd_menu_main);
        message_counter = 0;
        return;
    }
    else
        lcd_info_screen(lcd_menu_main, postPrintReady, PSTR("BACK TO MENU"));
    char buffer[20];
    char* c;
    if (time_phase1)
    {
        // Let's show the final print time....
        unsigned long printTimeSec = (stoptime - starttime) / 1000;

        strcpy_P(buffer, PSTR("Done in  "));
        c = EchoTimeSpan(printTimeSec, buffer + 8);
        *c++ = 0;
        lcd_lib_draw_string_center(10, buffer);
    }
    else
    {
        strcpy_P(buffer, PSTR("Est was  "));
        c = EchoTimeSpan(estimatedTime, buffer + 8);
        *c++ = 0;
        lcd_lib_draw_string_center(10, buffer);
    }

    // changed to a comparison with prior state saved and a gap between states to avoid switching back and forth
    // at the trigger point (hysteresis)
    static bool print_is_cool = false;
    if (current_temperature_bed > 44 || current_temperature[0] > 64) print_is_cool = false;
    if (current_temperature_bed < 40 && current_temperature[0] < 60)
        if (!print_is_cool)
        {
            LED_NORMAL();
            printDoneBeep();
            print_is_cool = true;
#ifdef DEBUG_INFO
            SERIAL_ECHO_START ;
            SERIAL_ECHOLNPGM(("PRINT COOLED"));
#endif

        }
    if (!print_is_cool )
    {
        LED_COOL();

        lcd_lib_draw_string_centerP(20, PSTR("Printer cooling down"));

        int16_t progress = 124 - max ((current_temperature[0] - 60), (current_temperature_bed - 40));		// whichever is slowest (usually the bed)
        if (progress < 0) progress = 0;
        if (progress > 124) progress = 124;

        if (progress < minProgress)
            progress = minProgress;
        else
            minProgress = progress;

        lcd_progressbar(progress);

        c = buffer;
        for (uint8_t e = 0; e < EXTRUDERS; e++)
            c = int_to_string(current_temperature[e], buffer, PSTR( DEGREE_C_SYMBOL "   "));
        int_to_string(current_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ));
        lcd_lib_draw_string_center(30, buffer);
    }
    else
    {
        LED_DONE();
        lcd_lib_draw_string_centerP(20, PSTR("Print finished"));
        lcd_lib_draw_string_centerP(30, PSTR("You can remove"));
        lcd_lib_draw_string_center(40, last_print_name);
    }
    lcd_lib_update_screen();
}

enum MENU_ORDER
{
    MENU_PRINT_TUNE_RETURN = 0,
    MENU_PRINT_TUNE_PAUSE,
    MENU_PRINT_TUNE_HOME,
    MENU_PRINT_TUNE_CENTER_HEAD,
    MENU_PRINT_TUNE_FAN_SPEED,
    MENU_PRINT_TUNE_SPEED,
    MENU_PRINT_TUNE_FLOW,
#if EXTRUDERS > 1
    MENU_PRINT_TUNE_FLOW1,
#if EXTRUDERS > 2
    MENU_PRINT_TUNE_FLOW2,
#endif
#endif
    MENU_PRINT_TUNE_TEMP,
#if EXTRUDERS > 1
    MENU_PRINT_TUNE_TEMP1,
#if EXTRUDERS > 2
    MENU_PRINT_TUNE_TEMP2,
#endif
#endif

    MENU_PRINT_TUNE_PUSH_MATERIAL,
    MENU_PRINT_TUNE_NUDGE_UP,
    MENU_PRINT_TUNE_NUDGE_DOWN,
    MENU_PRINT_TUNE_BED_TEMP,
    MENU_PRINT_TUNE_DISABLE_RETRACTION,
    MENU_PRINT_TUNE_RETRACTION,
    MENU_PRINT_TUNE_DISABLE_ZLIFT,
    MENU_PRINT_TUNE_ADJUST_ZLIFT,
    MENU_PRINT_TUNE_MOTION,
    MENU_PRINT_TUNE_LEDS,
    MENU_PRINT_TUNE_MAX

};

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
char* lcd_menu_print_tune_getString(uint8_t nr)
{
    char* c = lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring	;

    switch (nr)
    {
        case MENU_PRINT_TUNE_RETURN:
            strcpy_P(c, PSTR("< RETURN"));
            break;
        case MENU_PRINT_TUNE_PAUSE:
            if (!card.pause)
            {
                if (movesplanned() > 0)
                    strcpy_P(c, PSTR("Pause"));
                else
                    strcpy_P(c, PSTR("--Can't Pause"));
            }
            else
            {
                if (movesplanned() < 1)
                    strcpy_P(c, PSTR("Resume"));
                else
                    strcpy_P(c, PSTR("Pausing...wait..."));
            }
            break;
        case MENU_PRINT_TUNE_SPEED:
            strcpy_P(c, PSTR("Speed"));
            break;
        case  MENU_PRINT_TUNE_TEMP:
            strcpy_P(c, PSTR("Temperature"));
            break;

#if EXTRUDERS > 1
        case MENU_PRINT_TUNE_TEMP1:
            strcpy_P(c, PSTR("Temperature 2"));
            break;
#if EXTRUDERS > 2
        case MENU_PRINT_TUNE_TEMP2:
            strcpy_P(c, PSTR("Temperature 2"));
            break;
#endif
#if EXTRUDERS > 2
        case MENU_PRINT_TUNE_FLOW2:
            strcpy_P(c, PSTR("Material flow 2"));
            break;
#endif
        case MENU_PRINT_TUNE_FLOW1:
            strcpy_P(c, PSTR("Material flow 2"));
            break;
#endif
        case MENU_PRINT_TUNE_BED_TEMP:
            strcpy_P(c, PSTR("Bed Temperature"));
            break;
        case MENU_PRINT_TUNE_FAN_SPEED:
            strcpy_P(c, PSTR("Fan Speed Override"));
            break;
        case MENU_PRINT_TUNE_FLOW:
            strcpy_P(c, PSTR("Material Flow"));
            break;
        case MENU_PRINT_TUNE_RETRACTION:
            strcpy_P(c, PSTR("Retraction"));
            break;
        case MENU_PRINT_TUNE_ADJUST_ZLIFT:
            strcpy_P(c, PSTR("Z-Lift"));
            break;
        case MENU_PRINT_TUNE_LEDS:
            strcpy_P(c, PSTR("LED Brightness"));
            break;
        case MENU_PRINT_TUNE_HOME:
            if (!card.pause)
                strcpy_P(c, PSTR("--Can't Home"));
            else
                strcpy_P(c, PSTR("Home Head & Resume"));
            break;
        case MENU_PRINT_TUNE_NUDGE_UP:
            strcpy_P(c, PSTR("Nudge Bed Up 0.1mm"));
            break;
        case MENU_PRINT_TUNE_NUDGE_DOWN:
            strcpy_P(c, PSTR("Nudge Bed Down 0.1mm"));
            break;
        case MENU_PRINT_TUNE_DISABLE_RETRACTION:
            if (old_retraction == 0.0)
                strcpy_P(c, PSTR("Disable Retraction"));
            else
                strcpy_P(c, PSTR("Enable  Retraction"));
            break;
        case MENU_PRINT_TUNE_DISABLE_ZLIFT:
            if (old_zlift == 0.0 )
                strcpy_P(c, PSTR("Disable Z-Lift"));
            else
                strcpy_P(c, PSTR("Enable  Z-Lift"));
            break;
        case MENU_PRINT_TUNE_PUSH_MATERIAL:
            if (!card.pause)
                strcpy_P(c, PSTR("--Can't Extrude"));
            else
                strcpy_P(c, PSTR("Extrude 10mm"));
            break;
        default:
            strcpy_P(c, PSTR("???"));
            break;

        case MENU_PRINT_TUNE_CENTER_HEAD:
            if (!card.pause)
                strcpy_P(c, PSTR("--Can't Center"));
            else
                strcpy_P(c, PSTR("Center head"));
            break;
        case MENU_PRINT_TUNE_MOTION:
            strcpy_P(c, PSTR("Motion settings"));
            break;

    }
    return c;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_print_tune_getDetails(uint8_t nr)
{
    char buffer[30];
    memset (buffer, 0, sizeof(buffer));
    char* c = buffer;
    byte index = 0;
    switch (nr)
    {
        case  MENU_PRINT_TUNE_SPEED:
            c = int_to_string(feedmultiply, c, PSTR("%"));
            *c++ = 0;
            break;
#if EXTRUDERS > 2
        case MENU_PRINT_TUNE_TEMP2:
            index++;
#endif
#if EXTRUDERS > 1
        case MENU_PRINT_TUNE_TEMP1:
            index++;
#endif
        case MENU_PRINT_TUNE_TEMP:
            c = int_to_string(current_temperature[index], c /*,PSTR( DEGREE_C_SYMBOL )*/);
            *c++ = TEMPERATURE_SEPARATOR;
            c = int_to_string(target_temperature[index], c, PSTR( DEGREE_C_SYMBOL ));
            break;
        case MENU_PRINT_TUNE_BED_TEMP:
            c = int_to_string(current_temperature_bed, c/*, PSTR( DEGREE_C_SYMBOL )*/);
            *c++ = TEMPERATURE_SEPARATOR;
            c = int_to_string(target_temperature_bed, c, PSTR( DEGREE_C_SYMBOL ));
            *c++ = 0;
            break;
        case MENU_PRINT_TUNE_FAN_SPEED:
            *c++ = 0;
            c = int_to_string(int(getFanSpeed()) * 100 / 255, c, PSTR("%"));
            break;
#if EXTRUDERS > 2
        case MENU_PRINT_TUNE_FLOW2:
            index++;
#endif
#if EXTRUDERS > 1
        case MENU_PRINT_TUNE_FLOW1:
            index++;
#endif
        case MENU_PRINT_TUNE_FLOW:
            c = int_to_string(extrudemultiply[index], c, PSTR("%"));
            *c++ = 0;
            break;
        case MENU_PRINT_TUNE_LEDS:
            c = int_to_string(led_brightness_level, c, PSTR("%"));
            *c++ = 0;
            if (led_mode == LED_MODE_ALWAYS_ON ||  led_mode == LED_MODE_WHILE_PRINTING || led_mode == LED_MODE_BLINK_ON_DONE)
                analogWrite(LED_PIN, 255 * int(led_brightness_level) / 100);
            break;
        default:
            return;
        case MENU_PRINT_TUNE_ADJUST_ZLIFT:
            c = float_to_string(retract_zlift, c, PSTR("mm"));
            *c++ = 0;
            break;
        case MENU_PRINT_TUNE_RETRACTION:
            c = float_to_string1(retract_length, c, PSTR(" @ "));
            c = int_to_string((int) retract_feedrate / 60, c, PSTR("mm" PER_SECOND_SYMBOL "="));
            c = float_to_string1(retract_length / (retract_feedrate / 60), c, PSTR("s"));
            *c++ = 0;
            break;

    }
    lcd_lib_draw_string(5, 53, buffer);
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_print_tune_doAction()
{
    if (!lcd_lib_update_ready()) return;

    lcd_scroll_menu(PSTR("TUNE"), MENU_PRINT_TUNE_MAX, lcd_menu_print_tune_getString, lcd_menu_print_tune_getDetails);
//    lcd_scroll_menu(PSTR("TUNE"), 8 + EXTRUDERS * 2, tune_item_callback, tune_item_details_callback);

    if (millis() - last_user_interaction > MENU_TIMEOUT )		// 30 seconds, idle out of menu because we won't advance from preheating to printing if we don;t
    {
        lcd_menu_go_back();
//             if (card.sdprinting)
//                 lcd_change_to_menu(lcd_menu_print_printing);
//             else
//                 lcd_change_to_menu(lcd_menu_print_heatup);
    }

    if (!lcd_lib_button_pressed()) return;
    byte index = 0;
    switch (SELECTED_SCROLL_MENU_ITEM())
    {
        case MENU_PRINT_TUNE_MOTION:

            lcd_change_to_menu (lcd_menu_maintenance_motion);
            break;

        case MENU_PRINT_TUNE_RETURN:
            lcd_menu_go_back();
//                 if (card.sdprinting)
//                     lcd_change_to_menu(lcd_menu_print_printing);
//                 else
//                     lcd_change_to_menu(lcd_menu_print_heatup);
            break;
        case MENU_PRINT_TUNE_PAUSE:
            togglePausePrinting(false);
            lcd_lib_beep_ext(1000, 250);
            delay(200);
            break;
        case MENU_PRINT_TUNE_HOME:
            if (card.pause)
                togglePausePrinting(true);
            else
                lcd_lib_beep_ext(100, 250);
            break;
        case MENU_PRINT_TUNE_SPEED:
            LCD_EDIT_SETTING_INT(feedmultiply, "Print speed", "%", 10, 1000);
            break;
#if EXTRUDERS > 2
        case MENU_PRINT_TUNE_TEMP2:
            index++;
#endif
#if EXTRUDERS > 1
        case MENU_PRINT_TUNE_TEMP1:
            index++;
#endif
        case MENU_PRINT_TUNE_TEMP:
            nozzle_adjust_id = index;
            lcd_change_to_menu(lcd_menu_print_tune_heatup_nozzle, 0);
            break;
        case MENU_PRINT_TUNE_BED_TEMP:
            lcd_change_to_menu(lcd_menu_maintenance_advanced_bed_heatup, 0);//Use the maintainace heatup menu, which shows the current temperature.
            break;
        case MENU_PRINT_TUNE_FAN_SPEED:
            LCD_EDIT_SETTING_FAN_OVERRIDE(fanSpeedOverride, "Fan speed", "%", 0, 100);
            break;
#if EXTRUDERS > 2
        case MENU_PRINT_TUNE_FLOW2:
            index++;
#endif
#if EXTRUDERS > 1
        case MENU_PRINT_TUNE_FLOW1:
            index++;
#endif
        case MENU_PRINT_TUNE_FLOW:
            LCD_EDIT_SETTING_INT(extrudemultiply[index], "Material flow", "%", 10, 1000);
            break;
        case MENU_PRINT_TUNE_RETRACTION:
            lcd_change_to_menu(lcd_menu_retraction_doAction);
            break;
        case MENU_PRINT_TUNE_LEDS:
            LCD_EDIT_SETTING_INT(led_brightness_level, "Brightness", "%", 0, 100);
            break;
        case MENU_PRINT_TUNE_NUDGE_UP:
            lcd_lib_beep();
            Nudge(Z_AXIS, 0.1);
            break;
        case MENU_PRINT_TUNE_NUDGE_DOWN:
            lcd_lib_beep();
            Nudge(Z_AXIS, -0.1);
            break;
        case MENU_PRINT_TUNE_DISABLE_RETRACTION:
            lcd_lib_beep();
            swap (old_retraction, retract_length);
            break;
        case MENU_PRINT_TUNE_DISABLE_ZLIFT:
            lcd_lib_beep();
            swap (old_zlift, retract_zlift);
            break;
        case MENU_PRINT_TUNE_ADJUST_ZLIFT:
            if (old_zlift != 0) swap (old_zlift, retract_zlift);
            old_zlift = 0;
            LCD_EDIT_SETTING_FLOATx01(retract_zlift, "Retract Z-Lift", "mm", 0, 5);
            break;

        case MENU_PRINT_TUNE_PUSH_MATERIAL:
            lcd_lib_beep();
            if (card.pause) Nudge(E_AXIS, 10);
            else
                lcd_lib_beep_ext(100, 250);
            break;
        case MENU_PRINT_TUNE_CENTER_HEAD:
            lcd_lib_beep();
            if (card.pause)
            {
                while (commands_queued() > BUFSIZE - 3) manageBuffer();
                enquecommand_P(PSTR("G1 X100 Y20 F12000"));
            }
            else
                lcd_lib_beep_ext(100, 250);

            break;

        default: break;
    }
}



void lcd_menu_draw_temp_adj_screen();


// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// no parameters to menus, so we'll use a global.
unsigned char  nozzle_adjust_id = 0;

void lcd_menu_print_tune_heatup_nozzle()
{
    if (millis() - last_user_interaction > MENU_TIMEOUT)   lcd_menu_go_back();

    unsigned int mt = HEATER_0_MAXTEMP;
#ifdef HEATER_1_MAXTEMP
    if (nozzle_adjust_id == 1) mt = HEATER_1_MAXTEMP;
#endif
#ifdef HEATER_2_MAXTEMP
    if (nozzle_adjust_id == 2) mt = HEATER_2_MAXTEMP;
#endif
    lcd_lib_enable_encoder_acceleration(true);
    if (lcd_lib_encoder_pos /*/ ENCODER_TICKS_PER_SCROLL_MENU_ITEM */ != 0)
    {
        target_temperature[nozzle_adjust_id] = int(target_temperature[nozzle_adjust_id]) + (lcd_lib_encoder_pos /*/ ENCODER_TICKS_PER_SCROLL_MENU_ITEM*/);
        if (target_temperature[nozzle_adjust_id] < 0)
            target_temperature[nozzle_adjust_id] = 0;
        if (target_temperature[nozzle_adjust_id] > mt - 15)
            target_temperature[nozzle_adjust_id] = mt - 15;
        lcd_lib_encoder_pos = 0;
    }
    if (lcd_lib_button_pressed())
        lcd_menu_go_back();
    lcd_menu_draw_temp_adj_screen();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
char* lcd_menu_retraction_getString(uint8_t nr)
{
    char* c = lcd_cache_new.getData(LCD_CACHE::RAWSTRING).rawstring;
    switch (nr)
    {
        case 0:
            strcpy_P(c, PSTR("< RETURN"));
            break;
        case 1:
            strcpy_P(c, PSTR("Retract length"));
            break;
        case 2:
            strcpy_P(c, PSTR("Retract speed"));
            break;
#if EXTRUDERS > 1
        case 3:
            strcpy_P((char*)lcd_cache, PSTR("Extruder change len"));
            break;
#endif
        default:
            strcpy_P(c, PSTR("???"));
            break;
    }
    return c;
}



//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_retraction_getDetails(uint8_t nr)
{
    char buffer[16];
    memset (buffer, 0, sizeof(buffer));

    switch (nr)
    {
        case 0:
            return;
        case 1:
            float_to_string(retract_length, buffer, PSTR("mm"));
            break;
        case 2:
            int_to_string(retract_feedrate / 60 + 0.5, buffer, PSTR("mm" PER_SECOND_SYMBOL ));
            break;
#if EXTRUDERS > 1
        case 3:
            int_to_string(extruder_swap_retract_length, buffer, PSTR("mm"));
            break;
#endif
    }

    lcd_lib_draw_string(5, 53, buffer);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_menu_retraction_doAction()
{
    if (!lcd_lib_update_ready()) return;
    if (old_retraction != 0.0) swap (old_retraction, retract_length);
    old_retraction = 0.0;
    if (millis() - last_user_interaction > MENU_TIMEOUT)   lcd_menu_go_back();
    lcd_scroll_menu(PSTR("RETRACTION"), 3 + (EXTRUDERS > 1 ? 1 : 0), lcd_menu_retraction_getString, lcd_menu_retraction_getDetails);
    if (lcd_lib_button_pressed())
    {
        switch (SELECTED_SCROLL_MENU_ITEM())
        {
            case 0:
                lcd_menu_go_back();
//                        lcd_change_to_menu(lcd_menu_print_tune_doAction, SCROLL_MENU_ITEM_POS(6));
                break;
            case 1:
                lcd_lib_enable_encoder_acceleration(true);
                LCD_EDIT_SETTING_FLOATx01(retract_length, "Retract length", "mm", 0, 15);
                break;
            case 2:
                lcd_lib_enable_encoder_acceleration(true);
                LCD_EDIT_SETTING_SPEED(retract_feedrate, "Retract speed", "mm" PER_SECOND_SYMBOL , 10, max_feedrate[E_AXIS] * 60);
                break;
#if EXTRUDERS > 1
            case 3:
                LCD_EDIT_SETTING_FLOATx001(extruder_swap_retract_length, "Extruder change", "mm", 0, 50);
                break;

#endif
        }

    }
}




//-----------------------------------------------------------------------------------------------------------------
void prepareToPrintUltiGCode()
{
    //New style GCode flavor without start/end code.
    // Temperature settings, filament settings, fan settings, start and end-code are machine controlled.
    target_temperature_bed = 0;
    fanSpeedPercent = 0;
    run_history = true;
    estimatedTime = LCD_DETAIL_CACHE_TIME();
    for (uint8_t e = 0; e < EXTRUDERS; e++)
    {
        if (LCD_DETAIL_CACHE_MATERIAL(e) < 1)
            continue;
        target_temperature[e] = 0;//material[e].temperature;
        target_temperature_bed = max(target_temperature_bed, material[e].bed_temperature);
        fanSpeedPercent = max(fanSpeedPercent, material[e].fan_speed);
        volume_to_filament_length[e] = 1.0 / (M_PI * (material[e].diameter / 2.0) * (material[e].diameter / 2.0));
        extrudemultiply[e] = material[e].flow;
    }
    fanSpeed = 0;
    fanSpeedOverride = 0;	// auto
    enquecommand_P(PSTR("G28"));
    enquecommand_P(PSTR("G1 F12000 X100 Y50"));
}

#endif//ENABLE_ULTILCD2


//-----------------------------------------------------------------------------------------------------------------

