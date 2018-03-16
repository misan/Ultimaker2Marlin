#ifndef ULTI_LCD2_MENU_MAINTENANCE_H
#define ULTI_LCD2_MENU_MAINTENANCE_H

void lcd_menu_maintenance_doAction();
void lcd_menu_draw_temp_adj_screen();
void moveXYLimits();
void randomMotion(int count);
void movementTest();
void lcd_menu_maintenance_adjust_max_X();
void lcd_menu_maintenance_adjust_max_Y();
void lcd_menu_maintenance_motion();
void lcd_messagescreen (const char * msg);
#endif//ULTI_LCD2_MENU_MAINTENANCE_H
