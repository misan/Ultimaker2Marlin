// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"


#ifndef AT90USB
#define  HardwareSerial_h // trick to disable the standard HWserial
#endif

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
//Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#include "WString.h"
#include "MarlinSerial.h"
#include "CommandQ.h"
#include "PeriodTimer.h"

#define LOG_MOTION 0



#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
#define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START


#ifdef AT90USB
#define MYSERIAL Serial
#else
#define MYSERIAL MSerial
#endif

#include "SerialMacros.h"

void runTasks(bool with_command_processing=false);
void FlushSerialRequestResend();
void ClearToSend();

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};


void clamp_to_software_endstops(float target[3]);


#if defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
#define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
#define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
#define enable_x() ;
#define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
#define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
#define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
#define enable_y() ;
#define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
#ifdef Z_DUAL_STEPPER_DRIVERS
#define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
#define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); }
#else
#define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
#define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
#endif
#else
#define enable_z() ;
#define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
#define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
#define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e0()  /* nothing */
#define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
#define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
#define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e1()  /* nothing */
#define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
#define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
#define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e2()  /* nothing */
#define disable_e2() /* nothing */
#endif


#ifdef DELTA
void calculate_delta(float cartesian[3]);
#endif


void manage_Bed_Lights();
void manage_inactivity();
void controllerFan();

void kill();
#define STOP_REASON_MAXTEMP        1
#define STOP_REASON_MINTEMP        2
#define STOP_REASON_MAXTEMP_BED    3
#define STOP_REASON_SAFETY_TRIGGER 10
void Stop(uint8_t reasonNr);
extern uint8_t Stopped ;
inline bool IsStopped() { return Stopped; };
inline uint8_t StoppedReason() { return Stopped; };

//-----------------------------------------------------------------------------------------------------------------

// most of these don't live in Marlin.cpp -- messy!

extern float homing_feedrate[];
extern bool axis_relative_modes[];

extern int extrudemultiply[EXTRUDERS]; // Sets extrude multiply factor (in percent)
extern float current_position[NUM_AXIS] ;
extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];
extern uint8_t tmp_extruder;



//Inactivity shutdown variables
extern unsigned long previous_millis_cmd ;
extern unsigned long max_inactive_time ;
extern unsigned long stepper_inactive_time;

extern unsigned long starttime;
extern unsigned long stoptime;

class PeriodTimer;
extern PeriodTimer head_cooling_fan_timer;
extern PeriodTimer mobo_cooling_fan_timer;
extern PeriodTimer ui_timer;

extern PeriodTimer extruder_temp_timer ;
extern PeriodTimer temp_log_timer;
extern PeriodTimer inactivity_timer;

extern PeriodTimer bed_light_timer;
extern PeriodTimer stats_timer ;
extern PeriodTimer ambient_timer ;
extern PeriodTimer bed_temp_timer;

#ifdef BARICUDA
extern int ValvePressure;
extern int EtoPPressure;
#endif

#ifdef FAN_SOFT_PWM
extern unsigned char fanSpeedSoftPwm;
#endif




inline void swap (float &a, float &b)
{
    float c = a;
    a  = b;
    b = c;
}

#define XORSWAP(a, b)	((a)^=(b),(b)^=(a),(a)^=(b))



extern float  X_MAX_LENGTH;
extern float Y_MAX_LENGTH;


extern unsigned long starttime;
extern unsigned long stoptime;

//The printing state from the main command processor. Is not zero when the command processor is in a loop waiting for a result.
extern uint8_t printing_state;
#define PRINT_STATE_NORMAL      0
#define PRINT_STATE_DWELL       1
#define PRINT_STATE_WAIT_USER   2
#define PRINT_STATE_HEATING     3
#define PRINT_STATE_HEATING_BED 4
#define PRINT_STATE_HOMING      5

extern unsigned long last_user_interaction;


extern unsigned char NOWRAP_MENUS;
extern unsigned char LED_DIM_TIME;

// Handling multiple extruders pins
extern uint8_t active_extruder;

#if EXTRUDERS > 3
# error Unsupported number of extruders
#elif EXTRUDERS > 2
# define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif EXTRUDERS > 1
# define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
# define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif

#endif
