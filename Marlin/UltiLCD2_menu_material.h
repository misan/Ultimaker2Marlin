#ifndef ULTI_LCD2_MENU_MATERIAL_H
#define ULTI_LCD2_MENU_MATERIAL_H

struct materialSettings
{
    int16_t temperature;
    int16_t bed_temperature;
    uint8_t fan_speed; //0-100% of requested speed by GCode
    int16_t flow;      //Flow modification in %
    float diameter; //Filament diameter in mm
};

extern struct materialSettings material[EXTRUDERS];

#define FILAMENT_REVERSAL_LENGTH      750
#define FILAMENT_REVERSAL_SPEED       200
#define FILAMENT_LONG_MOVE_ACCELERATION 30

#define FILAMENT_FORWARD_LENGTH       650
#define FILAMENT_INSERT_SPEED         2     //Initial insert speed to grab the filament.
#define FILAMENT_INSERT_FAST_SPEED    200   //Speed during the forward length
#define FILAMENT_INSERT_EXTRUDE_SPEED 2     //Final speed when extruding

const byte MATERIAL_NAME_LENGTH = 12 ;		

extern char material_name_buf[MATERIAL_NAME_LENGTH+1];
extern char material_name[EXTRUDERS][MATERIAL_NAME_LENGTH+1];


#define EEPROM_MATERIAL_SETTINGS_OFFSET 0x800
#define EEPROM_MATERIAL_SETTINGS_MAX_COUNT 16
#define EEPROM_MATERIAL_SETTINGS_SIZE   (MATERIAL_NAME_LENGTH + 16)
#define EEPROM_MATERIAL_COUNT_OFFSET()            ((uint8_t*)(EEPROM_MATERIAL_SETTINGS_OFFSET + 0))
#define EEPROM_MATERIAL_NAME_OFFSET(n)            ((uint8_t*)(EEPROM_MATERIAL_SETTINGS_OFFSET + 1 + EEPROM_MATERIAL_SETTINGS_SIZE * uint16_t(n)))
#define EEPROM_MATERIAL_TEMPERATURE_OFFSET(n)     ((uint16_t*)(EEPROM_MATERIAL_SETTINGS_OFFSET + 1 + EEPROM_MATERIAL_SETTINGS_SIZE * uint16_t(n) + MATERIAL_NAME_LENGTH))
#define EEPROM_MATERIAL_BED_TEMPERATURE_OFFSET(n) ((uint16_t*)(EEPROM_MATERIAL_SETTINGS_OFFSET + 1 + EEPROM_MATERIAL_SETTINGS_SIZE * uint16_t(n) + MATERIAL_NAME_LENGTH+2))
#define EEPROM_MATERIAL_FAN_SPEED_OFFSET(n)       ((uint8_t*)(EEPROM_MATERIAL_SETTINGS_OFFSET + 1 + EEPROM_MATERIAL_SETTINGS_SIZE * uint16_t(n) + MATERIAL_NAME_LENGTH+4))
#define EEPROM_MATERIAL_FLOW_OFFSET(n)            ((uint16_t*)(EEPROM_MATERIAL_SETTINGS_OFFSET + 1 + EEPROM_MATERIAL_SETTINGS_SIZE * uint16_t(n) + MATERIAL_NAME_LENGTH+5))
#define EEPROM_MATERIAL_DIAMETER_OFFSET(n)        ((float*)(EEPROM_MATERIAL_SETTINGS_OFFSET + 1 + EEPROM_MATERIAL_SETTINGS_SIZE * uint16_t(n) + MATERIAL_NAME_LENGTH+7))

void lcd_menu_material();
bool lcd_material_verify_material_settings();
void lcd_material_reset_defaults();
void lcd_material_set_material(uint8_t nr, uint8_t e);
void lcd_material_store_material(uint8_t nr);
void lcd_material_read_current_material();
void lcd_material_store_current_material();

#endif//ULTI_LCD2_MENU_MATERIAL_H
