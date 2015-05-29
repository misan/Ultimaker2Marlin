#ifndef ULTI_LCD2_MENU_PRINT_H
#define ULTI_LCD2_MENU_PRINT_H

#include "cardreader.h"
#include "SwissArmyCache.h"

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
#endif//ULTI_LCD2_MENU_PRINT_H
