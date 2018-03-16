#ifndef ULTI_LCD2_MENU_PRINT_H
#define ULTI_LCD2_MENU_PRINT_H

#include "cardreader.h"
#include "SwissArmyCache.h"

// introduce a short delay before reading file details so director listings are more responsive...
#define FILE_READ_DELAY 35
extern int file_read_delay_counter;

extern float old_zlift;
extern float old_retraction;
extern bool last_print_aborted ;
void lcd_sd_filemenu_doAction();

void doCancelPrint();
void lcd_menu_maintenance_advanced_bed_heatup();

//-----------------------------------------------------------------------------------------------------------------
extern char last_print_name[LONG_FILENAME_LENGTH];
extern unsigned char nozzle_adjust_id;
void lcd_menu_print_tune_heatup_nozzle();
void lcd_menu_retraction_getDetails(uint8_t nr);
char* lcd_menu_retraction_getString(uint8_t nr);
void lcd_menu_retraction_doAction();
void lcd_menu_print_ready();
void lcd_menu_print_classic_warning();
void lcd_menu_print_error();
void lcd_menu_print_abort();
void prepareToPrintUltiGCode();
void lcd_menu_print_printing();
void lcd_menu_print_heatup();
void lcd_menu_print_tune_doAction();
#endif//ULTI_LCD2_MENU_PRINT_H
