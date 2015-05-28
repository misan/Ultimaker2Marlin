/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Mega w/ ATmega2560 (Mega 2560), Platform=avr, Package=arduino
*/

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define __AVR_ATmega2560__
#define ARDUINO 157
#define ARDUINO_MAIN
#define __AVR__
#define __avr__
#define F_CPU 16000000L
#define __cplusplus
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__

#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
            
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}


#include "C:\dev\arduino-1.5.7\hardware\arduino\avr\cores\arduino\arduino.h"
#include "C:\dev\arduino-1.5.7\hardware\arduino\avr\variants\mega\pins_arduino.h" 
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Marlin.pde"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Configuration.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\ConfigurationStore.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\ConfigurationStore.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Configuration_adv.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\DOGMbitmaps.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\LiquidCrystalRus.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\LiquidCrystalRus.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Marlin.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\MarlinSerial.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\MarlinSerial.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Marlin_main.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Sd2Card.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Sd2Card.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Sd2PinMap.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdBaseFile.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdBaseFile.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdFatConfig.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdFatStructs.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdFatUtil.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdFatUtil.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdFile.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdFile.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdInfo.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdVolume.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\SdVolume.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Servo.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\Servo.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_gfx.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_gfx.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_hi_lib.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_hi_lib.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_low_lib.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_low_lib.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_menu_first_run.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_menu_first_run.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_menu_maintenance.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_menu_maintenance.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_menu_material.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_menu_material.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_menu_print.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\UltiLCD2_menu_print.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\cardreader.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\cardreader.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\dogm_font_data_marlin.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\dogm_lcd_implementation.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\fastio.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\language.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\lifetime_stats.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\lifetime_stats.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\motion_control.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\motion_control.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\pins.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\planner.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\planner.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\speed_lookuptable.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\stepper.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\stepper.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\temperature.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\temperature.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\thermistortables.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\ultralcd.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\ultralcd.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\ultralcd_implementation_hitachi_HD44780.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\ultralcd_implementation_ultiboard_v2.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\ultralcd_st7920_u8glib_rrd.h"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\watchdog.cpp"
#include "C:\dev\GitHub\Ultimaker2Marlin\Marlin\watchdog.h"
#endif
