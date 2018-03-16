#include "marlin.h"
#include "gcode.h"


#include "ultralcd.h"
#include "UltiLCD2.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "lifetime_stats.h"
#include "language.h"
#include "pins_arduino.h"
#include "MenuUseful.h"

#include "DHT.h"


#if NUM_SERVOS > 0
#include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif
#include "UltiLCD2_hi_lib.h"
#include "UltiLCD2_menu_print.h"
#include "UltiLCD2_menu_material.h"
#include "voltage.h"
#include "gcode.h"
#include "stringHelpers.h"

#if EXTRUDERS > 1
float extruder_swap_retract_length=16;
#endif



#ifdef FWRETRACT
bool autoretract_enabled=false;
bool retracted=false;
float retract_length=4.5;
float retract_feedrate=25*60.0;
float retract_zlift=0.8;

float retract_recover_length=0;
float retract_recover_feedrate=25*60.0;
#endif

long gcode_N=0;
long gcode_LastN=0;
long Stopped_gcode_LastN = 0;

byte  fanSpeedOverride=0;
bool setTargetedHotend(int code);

uint8_t fanSpeed=0;
uint8_t fanSpeedPercent=100;

bool relative_mode = false;  //Determines Absolute or Relative Coordinates



float offset[3] = {0.0, 0.0, 0.0};
bool home_all_axis = true;
float feedrate = 1500.0, next_feedrate, saved_feedrate;
const int sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42
float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
int saved_feedmultiply;

float pause_target_position[4];
float pre_pause_position[4];
bool am_paused = false;


char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc


float code_value()
{
    return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
    return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
    strchr_pointer = strchr(cmdbuffer[bufindr], code);
    return (strchr_pointer != NULL);  //Return True if a character was found
}


bool setTargetedHotend(int code)
{
    tmp_extruder = active_extruder;
    if(code_seen('T'))
        {
            tmp_extruder = code_value();
            if(tmp_extruder >= EXTRUDERS)
                {
                    SERIAL_ECHO_START;
                    switch(code)
                        {
                            case 104:
                                SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
                                break;
                            case 105:
                                SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
                                break;
                            case 109:
                                SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
                                break;
                            case 218:
                                SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
                                break;
                        }
                    SERIAL_ECHOLN(tmp_extruder);
                    return true;
                }
        }
    return false;
}


#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
const int min_fan_speed=20;



#define DEFINE_PGM_READ_ANY(type, reader)       \
	static inline type pgm_read_any(const type *p)  \
	{ return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
	static const PROGMEM type array##_P[3] =        \
	{ X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
	static inline type array(int axis)          \
	{ return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
// XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
// XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
// XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

float base_home_pos[3];
static void axis_is_at_home(int axis)
{

    //X axis
#if X_HOME_DIR == -1
#ifdef BED_CENTER_AT_0_0
    base_home_pos[X_AXIS] =  X_MAX_LENGTH * -0.5
#else
    base_home_pos[X_AXIS] = min_pos[X_AXIS];
#endif //BED_CENTER_AT_0_0
#else
#ifdef BED_CENTER_AT_0_0
    base_home_pos[X_AXIS] = X_MAX_LENGTH * 0.5
#else
    base_home_pos[X_AXIS] = max_pos[X_AXIS];
#endif //BED_CENTER_AT_0_0
#endif //X_HOME_DIR == -1

                             //Y axis
#if Y_HOME_DIR == -1
#ifdef BED_CENTER_AT_0_0
                             base_home_pos[Y_AXIS] = Y_MAX_LENGTH * -0.5
#else
                             base_home_pos[Y_AXIS] = min_pos[Y_AXIS];
#endif //BED_CENTER_AT_0_0
#else
#ifdef BED_CENTER_AT_0_0
                             base_home_pos[Y_AXIS] = Y_MAX_LENGTH * 0.5
#else
                             base_home_pos[Y_AXIS] = max_pos[Y_AXIS];
#endif //BED_CENTER_AT_0_0
#endif //Y_HOME_DIR == -1

                                     // Z axis
#if Z_HOME_DIR == -1 //BED_CENTER_AT_0_0 not used
                                     base_home_pos[Z_AXIS] =min_pos[X_AXIS];
#else
                                     base_home_pos[Z_AXIS] = max_pos[Z_AXIS];
#endif //Z_HOME_DIR == -1



    current_position[axis] = base_home_pos[axis] + add_homeing[axis];
    min_pos[axis] =          base_min_pos(axis);// + add_homeing[axis];
    //   max_pos[axis] =          base_max_pos(axis);// + add_homeing[axis];
}

static void homeaxis(int axis)
{
#define HOMEAXIS_DO(LETTER) \
	((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))
    if (axis==X_AXIS ? HOMEAXIS_DO(X) :
            axis==Y_AXIS ? HOMEAXIS_DO(Y) :
            axis==Z_AXIS ? HOMEAXIS_DO(Z) :
            0)
        {

            // Engage Servo endstop if enabled
#ifdef SERVO_ENDSTOPS
            if (servo_endstops[axis] > -1) servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
#endif

            current_position[axis] = 0;
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            destination[axis] = 1.5 * max_pos[axis] * home_dir(axis);
            feedrate = homing_feedrate[axis];
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            st_synchronize();

            current_position[axis] = 0;
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            destination[axis] = -home_retract_mm(axis) * home_dir(axis);
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            st_synchronize();

            destination[axis] = 2*home_retract_mm(axis) * home_dir(axis);
            feedrate = homing_feedrate[axis]/3;
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
            st_synchronize();

            axis_is_at_home(axis);
            destination[axis] = current_position[axis];
            feedrate = 0.0;
            endstops_hit_on_purpose();

            // Retract Servo endstop if enabled
#ifdef SERVO_ENDSTOPS
            if (servo_endstops[axis] > -1) servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
#endif
        }
}


//-----------------------------------------------------------------------------------------------------------------
void moveToPausePosition()
{
    if (am_paused)
        {
            SERIAL_ECHO_START
            SERIAL_ECHOLNPGM("redundant pause command, ignoring");
            return;
        }
    am_paused = true;
    pause_target_position[X_AXIS]=current_position[X_AXIS];
    pause_target_position[Y_AXIS]=current_position[Y_AXIS];
    pause_target_position[Z_AXIS]=current_position[Z_AXIS];
    pause_target_position[E_AXIS]=current_position[E_AXIS];
    pre_pause_position[X_AXIS]=current_position[X_AXIS];
    pre_pause_position[Y_AXIS]=current_position[Y_AXIS];
    pre_pause_position[Z_AXIS]=current_position[Z_AXIS];
    pre_pause_position[E_AXIS]=current_position[E_AXIS];

    pause_target_position[E_AXIS] -= retract_length/volume_to_filament_length[active_extruder];
    plan_buffer_line(pause_target_position[X_AXIS], pause_target_position[Y_AXIS], pause_target_position[Z_AXIS], pause_target_position[E_AXIS], retract_feedrate/60, active_extruder);

    //lift Z
    if(code_seen('Z')) pause_target_position[Z_AXIS]+= code_value();

    plan_buffer_line(pause_target_position[X_AXIS], pause_target_position[Y_AXIS], pause_target_position[Z_AXIS], pause_target_position[E_AXIS], homing_feedrate[Z_AXIS]/60, active_extruder);

    //move xy
    if(code_seen('X')) pause_target_position[X_AXIS] = code_value();
    if(code_seen('Y')) pause_target_position[Y_AXIS] = code_value();
    plan_buffer_line(pause_target_position[X_AXIS], pause_target_position[Y_AXIS], pause_target_position[Z_AXIS], pause_target_position[E_AXIS], homing_feedrate[X_AXIS]/60, active_extruder);

    if(code_seen('L'))  pause_target_position[E_AXIS] -= code_value()/volume_to_filament_length[active_extruder];

    plan_buffer_line(pause_target_position[X_AXIS], pause_target_position[Y_AXIS], pause_target_position[Z_AXIS], pause_target_position[E_AXIS], retract_feedrate/60, active_extruder);
	SERIAL_ECHO_START
	SERIAL_ECHOLNPGM("paused");

    //finish moves
    st_synchronize();

	current_position[X_AXIS]=pause_target_position[X_AXIS];
	current_position[Y_AXIS]=pause_target_position[Y_AXIS];
	current_position[Z_AXIS]=pause_target_position[Z_AXIS];
	current_position[E_AXIS]=pause_target_position[E_AXIS];

}

//-----------------------------------------------------------------------------------------------------------------
void resumeFromPausePosition()
{
    if (!am_paused)
        {
            SERIAL_ECHO_START
            SERIAL_ECHOLNPGM("redundant unpause command, ignoring");
            return;
        }
    am_paused= false;
    //return to normal
    if(code_seen('L')) pause_target_position[E_AXIS] += code_value()/volume_to_filament_length[active_extruder];

    plan_buffer_line(pause_target_position[X_AXIS],		 pause_target_position[Y_AXIS],		 pause_target_position[Z_AXIS],  pause_target_position[E_AXIS], retract_feedrate/60.0,			active_extruder); //Move back the L feed.
    plan_buffer_line(	pre_pause_position[X_AXIS]   ,		pre_pause_position[Y_AXIS],		 pause_target_position[Z_AXIS],  pause_target_position[E_AXIS], homing_feedrate[X_AXIS]/60.0,	active_extruder); //move xy back
    plan_buffer_line(	pre_pause_position[X_AXIS]   ,		pre_pause_position[Y_AXIS],			pre_pause_position[Z_AXIS],	 pause_target_position[E_AXIS], homing_feedrate[Z_AXIS]/60.0,	active_extruder); //move z back
    plan_buffer_line(	pre_pause_position[X_AXIS]   ,		pre_pause_position[Y_AXIS],			pre_pause_position[Z_AXIS],		pre_pause_position[E_AXIS], retract_feedrate/60.0,			active_extruder); //final untretract
	SERIAL_ECHO_START
	SERIAL_ECHOLNPGM("unpaused");
	st_synchronize();
	current_position[X_AXIS]=pre_pause_position[X_AXIS];
	current_position[Y_AXIS]=pre_pause_position[Y_AXIS];
	current_position[Z_AXIS]=pre_pause_position[Z_AXIS];
	current_position[E_AXIS]=pre_pause_position[E_AXIS];

}



//-----------------------------------------------------------------------------------------------------------------
void process_commands()
{
    unsigned long codenum; //throw away variable
    char *starpos = NULL;

    printing_state = PRINT_STATE_NORMAL;
    if(code_seen('G'))
        {
            switch((int)code_value())
                {
                    case 0: // G0 -> G1
                    case 1: // G1
                        if(Stopped == false)
                            {
                                get_coordinates(); // For X Y Z E F
                                prepare_move();
                                //ClearToSend();
                                return;
                            }
                    //break;
                    case 2: // G2  - CW ARC
                        if(Stopped == false)
                            {
                                get_arc_coordinates();
                                prepare_arc_move(true);
                                return;
                            }
                    case 3: // G3  - CCW ARC
                        if(Stopped == false)
                            {
                                get_arc_coordinates();
                                prepare_arc_move(false);
                                return;
                            }
                    case 4: // G4 dwell
                        //  LCD_MESSAGEPGM(MSG_DWELL);
                        codenum = 0;
                        if(code_seen('P')) codenum = code_value(); // milliseconds to wait
                        if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

                        st_synchronize();
                        codenum += millis();  // keep track of when we started waiting
                        previous_millis_cmd = millis();
                        printing_state = PRINT_STATE_DWELL;
                        while(millis()  < codenum )
                            {
                                char buffer[20];
                                memset (buffer,0,sizeof(buffer));
                                char *c = buffer;
                                c+=6;
                                strcpy_P (buffer,PSTR("WAIT: "));
                                c = int_to_string((int) (codenum-millis()/1000),c,PSTR(" sec"));
                                *c++=0;
                                lcd_setstatus (buffer);
								runTasks(false,true);

                            }
                        clear_message();
                        break;
#ifdef FWRETRACT
                    case 10: // G10 retract
                        if(!retracted)									// shouldn't this flag be handled on a per extruder basis?
                            {
                                destination[X_AXIS]=current_position[X_AXIS];
                                destination[Y_AXIS]=current_position[Y_AXIS];
                                destination[Z_AXIS]=current_position[Z_AXIS] + retract_zlift;
#if EXTRUDERS > 1
                                if (code_seen('S') && code_value_long() == 1)
                                    destination[E_AXIS]=current_position[E_AXIS]-extruder_swap_retract_length/volume_to_filament_length[active_extruder];
                                else
                                    destination[E_AXIS]=current_position[E_AXIS]-retract_length/volume_to_filament_length[active_extruder];
#else
                                destination[E_AXIS]=current_position[E_AXIS]-retract_length/volume_to_filament_length[active_extruder];
#endif
                                float oldFeedrate = feedrate;
                                feedrate=retract_feedrate;
                                retract_recover_length = current_position[E_AXIS]-destination[E_AXIS];//Set the recover length to whatever distance we retracted so we recover properly.
                                retracted=true;
                                prepare_move();
                                feedrate = oldFeedrate;
                            }

                        break;
                    case 11: // G11 retract_recover
                        if(retracted)
                            {
                                destination[X_AXIS]=current_position[X_AXIS];
                                destination[Y_AXIS]=current_position[Y_AXIS];
                                destination[Z_AXIS]=current_position[Z_AXIS] - retract_zlift;
                                destination[E_AXIS]=current_position[E_AXIS]+retract_recover_length;
                                float oldFeedrate = feedrate;
                                feedrate=retract_recover_feedrate;
                                retracted=false;
                                prepare_move();
                                feedrate = oldFeedrate;
                            }
                        break;
#endif //FWRETRACT
                    case 28: //G28 Home all Axis one at a time
                        printing_state = PRINT_STATE_HOMING;
                        saved_feedrate = feedrate;
                        saved_feedmultiply = feedmultiply;
                        feedmultiply = 100;
                        previous_millis_cmd = millis();

                        enable_endstops(true);

                        for(int8_t i=0; i < NUM_AXIS; i++)
                            {
                                destination[i] = current_position[i];
                            }
                        feedrate = 0.0;

#ifdef DELTA
                        // A delta can only safely home all axis at the same time
                        // all axis have to home at the same time

                        // Move all carriages up together until the first endstop is hit.
                        current_position[X_AXIS] = 0;
                        current_position[Y_AXIS] = 0;
                        current_position[Z_AXIS] = 0;
                        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

                        destination[X_AXIS] = 3 * Z_MAX_LENGTH;
                        destination[Y_AXIS] = 3 * Z_MAX_LENGTH;
                        destination[Z_AXIS] = 3 * Z_MAX_LENGTH;
                        feedrate = 1.732 * homing_feedrate[X_AXIS];
                        plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
                        st_synchronize();
                        endstops_hit_on_purpose();

                        current_position[X_AXIS] = destination[X_AXIS];
                        current_position[Y_AXIS] = destination[Y_AXIS];
                        current_position[Z_AXIS] = destination[Z_AXIS];

                        // take care of back off and rehome now we are all at the top
                        HOMEAXIS(X);
                        HOMEAXIS(Y);
                        HOMEAXIS(Z);

                        calculate_delta(current_position);
                        plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);

#else // NOT DELTA

                        home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

#if Z_HOME_DIR > 0                      // If homing away from BED do Z first
#if defined(QUICK_HOME)
                        if(home_all_axis)
                            {
                                current_position[X_AXIS] = 0; current_position[Y_AXIS] = 0; current_position[Z_AXIS] = 0;

                                plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

                                destination[X_AXIS] = 1.5 * X_MAX_LENGTH * X_HOME_DIR;
                                destination[Y_AXIS] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
                                destination[Z_AXIS] = 1.5 * Z_MAX_LENGTH * Z_HOME_DIR;
                                feedrate = homing_feedrate[X_AXIS];
                                plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
                                st_synchronize();
                                endstops_hit_on_purpose();

                                axis_is_at_home(X_AXIS);
                                axis_is_at_home(Y_AXIS);
                                axis_is_at_home(Z_AXIS);
                                plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
                                destination[X_AXIS] = current_position[X_AXIS];
                                destination[Y_AXIS] = current_position[Y_AXIS];
                                destination[Z_AXIS] = current_position[Z_AXIS];
                                plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
                                feedrate = 0.0;
                                st_synchronize();
                                endstops_hit_on_purpose();

                                current_position[X_AXIS] = destination[X_AXIS];
                                current_position[Y_AXIS] = destination[Y_AXIS];
                                current_position[Z_AXIS] = destination[Z_AXIS];
                            }
#endif
                        if((home_all_axis) || (code_seen(axis_codes[Z_AXIS])))
                            {
                                HOMEAXIS(Z);
                            }
#endif

#if defined(QUICK_HOME)
                        if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
                            {
                                current_position[X_AXIS] = 0; current_position[Y_AXIS] = 0;

                                plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
                                destination[X_AXIS] = 1.5 * X_MAX_LENGTH * X_HOME_DIR; destination[Y_AXIS] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
                                feedrate = homing_feedrate[X_AXIS];
                                if(homing_feedrate[Y_AXIS]<feedrate)
                                    feedrate =homing_feedrate[Y_AXIS];
                                plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
                                st_synchronize();

                                axis_is_at_home(X_AXIS);
                                axis_is_at_home(Y_AXIS);
                                plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
                                destination[X_AXIS] = current_position[X_AXIS];
                                destination[Y_AXIS] = current_position[Y_AXIS];
                                plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
                                feedrate = 0.0;
                                st_synchronize();
                                endstops_hit_on_purpose();

                                current_position[X_AXIS] = destination[X_AXIS];
                                current_position[Y_AXIS] = destination[Y_AXIS];
                                current_position[Z_AXIS] = destination[Z_AXIS];
                            }
#endif

                        if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
                            {
                                HOMEAXIS(X);
                            }

                        if((home_all_axis) || (code_seen(axis_codes[Y_AXIS])))
                            {
                                HOMEAXIS(Y);
                            }

#if Z_HOME_DIR < 0                      // If homing towards BED do Z last
                        if((home_all_axis) || (code_seen(axis_codes[Z_AXIS])))
                            {
                                HOMEAXIS(Z);
                            }
#endif

                        if(code_seen(axis_codes[X_AXIS]))
                            {
                                if(code_value_long() != 0)
                                    {
                                        current_position[X_AXIS]=code_value()+add_homeing[0];
                                    }
                            }

                        if(code_seen(axis_codes[Y_AXIS]))
                            {
                                if(code_value_long() != 0)
                                    {
                                        current_position[Y_AXIS]=code_value()+add_homeing[1];
                                    }
                            }

                        if(code_seen(axis_codes[Z_AXIS]))
                            {
                                if(code_value_long() != 0)
                                    {
                                        current_position[Z_AXIS]=code_value()+add_homeing[2];
                                    }
                            }
                        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
#endif // DELTA

#ifdef ENDSTOPS_ONLY_FOR_HOMING
                        enable_endstops(false);
#endif

                        feedrate = saved_feedrate;
                        feedmultiply = saved_feedmultiply;
                        previous_millis_cmd = millis();
                        endstops_hit_on_purpose();
                        break;
                    case 90: // G90
                        relative_mode = false;
                        break;
                    case 91: // G91
                        relative_mode = true;
                        break;
                    case 92: // G92
                        if(!code_seen(axis_codes[E_AXIS]))
                            st_synchronize();
                        for(int8_t i=0; i < NUM_AXIS; i++)
                            {
                                if(code_seen(axis_codes[i]))
                                    {
                                        if(i == E_AXIS)
                                            {
                                                current_position[i] = code_value();
                                                plan_set_e_position(current_position[E_AXIS]);
                                            }
                                        else
                                            {
                                                current_position[i] = code_value();
                                                plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
                                            }
                                    }
                            }
                        break;
                }
        }

    else
        if(code_seen('M'))
            {
                switch( (int)code_value() )
                    {
                        default:
                            SERIAL_ECHO_START;
                            SERIAL_PROTOCOLPGM("UNSUPPORTED COMMAND: M" );
                            SERIAL_PROTOCOLLN( (int)code_value() );
                            break;
#ifdef ULTIPANEL
                        case 0: // M0 - Unconditional stop - Wait for user button press on LCD
                        case 1: // M1 - Conditional stop - Wait for user button press on LCD
                            {
                                printing_state = PRINT_STATE_WAIT_USER;
                                LCD_MESSAGEPGM(MSG_USERWAIT);
                                codenum = 0;
                                if(code_seen('P')) codenum = code_value(); // milliseconds to wait
                                if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

                                st_synchronize();
                                previous_millis_cmd = millis();
                                if (codenum > 0)
                                    {
                                        codenum += millis();  // keep track of when we started waiting
                                        while(millis()  < codenum && !lcd_clicked())
                                            {
                                                manage_heater();
                                                manage_inactivity();
                                                lcd_update();
                                                lifetime_stats_tick();
                                            }
                                    }
                                else
                                    {
                                        while(!lcd_clicked())
                                            {
                                                manage_heater();
                                                manage_inactivity();
                                                lcd_update();
                                                lifetime_stats_tick();
                                            }
                                    }
                                LCD_MESSAGEPGM(MSG_RESUMING);
                            }
                            break;
#endif
#ifdef ENABLE_ULTILCD2
                        case 0: // M0 - Unconditional stop - Wait for user button press on LCD
                        case 1: // M1 - Conditional stop - Wait for user button press on LCD

                            {
                                printing_state = PRINT_STATE_WAIT_USER;

                                codenum = 0;
                                if(code_seen('P')) codenum = code_value(); // milliseconds to wait
                                if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

                                st_synchronize();
                                previous_millis_cmd = millis();
                                if (codenum > 0)
                                    {
                                        codenum += millis();  // keep track of when we started waiting
                                        while(millis()  < codenum && !lcd_lib_button_pressed())
                                            {
                                                char buffer[20];
                                                memset (buffer,0,sizeof(buffer));

                                                char *c = buffer;
                                                c+=6;
                                                strcpy_P (buffer,PSTR("WAIT: "));
                                                c = int_to_string((int) (codenum-millis()/1000),c,PSTR(" sec"));
                                                *c++=0;
                                                lcd_setstatus (buffer);
												last_user_interaction = millis();
                                                lcd_lib_update_screen();
													runTasks(false,true);
                                            }
                                    }
                                else
                                    {
                                        while(!lcd_lib_button_pressed())
                                            {
                                                LCD_MESSAGEPGM (PSTR("CLICK TO CONTINUE!"));
													runTasks(false,true);
                                                last_user_interaction = millis();
                                            }
                                        clear_message();
                                    }
                            }
                            break;
#endif
                        case 17:
                            LCD_MESSAGEPGM(MSG_NO_MOVE);
                            enable_x();
                            enable_y();
                            enable_z();
                            enable_e0();
                            enable_e1();
                            enable_e2();
                            // 		if (MOTHERBOARD_FAN>-1)
                            // 			WRITE (MOTHERBOARD_FAN,1);

                            break;

#ifdef SDSUPPORT
                        case 20: // M20 - list SD card
#ifdef USE_WATCHDOG
							watchdog_stop();
#endif 
                            SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
                            card.ls();
                            SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
#ifdef USE_WATCHDOG
							watchdog_start();

#endif // USE_WATCHDOG
							break;
                        case 21: // M21 - init SD card

                            card.initsd();

                            break;
                        case 22: //M22 - release SD card
                            card.release();

                            break;
                        case 23: //M23 - Select file
                            starpos = (strchr(strchr_pointer + 4,'*'));
                            if(starpos!=NULL)
                                *(starpos-1)='\0';
                            card.openFile(strchr_pointer + 4,true);
                            break;
                        case 24: //M24 - Start SD print
                            card.startFileprint();
                            starttime=millis();
                            break;
                        case 25: //M25 - Pause SD print
                            //card.pauseSDPrint();
                            card.closefile();
                            break;
                        case 26: //M26 - Set SD index
                            if(card.isOk() && code_seen('S'))
                                {
                                    card.setIndex(code_value_long());
                                }
                            break;
                        case 27: //M27 - Get SD status
                            card.getStatus();
                            break;
                        case 28: //M28 - Start SD write
                            starpos = (strchr(strchr_pointer + 4,'*'));
                            if(starpos != NULL)
                                {
                                    char* npos = strchr(cmdbuffer[bufindr], 'N');
                                    strchr_pointer = strchr(npos,' ') + 1;
                                    *(starpos-1) = '\0';
                                }
                            card.openFile(strchr_pointer+4,false);
                            break;
                        case 29: //M29 - Stop SD write
                            //processed in write to file routine above
                            //card,saving = false;
                            break;
                        case 30: //M30 <filename> Delete File
                            if (card.isOk())
                                {
                                    card.closefile();
                                    starpos = (strchr(strchr_pointer + 4,'*'));
                                    if(starpos != NULL)
                                        {
                                            char* npos = strchr(cmdbuffer[bufindr], 'N');
                                            strchr_pointer = strchr(npos,' ') + 1;
                                            *(starpos-1) = '\0';
                                        }
                                    card.removeFile(strchr_pointer + 4);
                                }
                            break;
                        case 923: //M923 - Select file and start printing
                            starpos = (strchr(strchr_pointer + 4,'*'));
                            if(starpos!=NULL)
                                *(starpos-1)='\0';
                            card.openFile(strchr_pointer + 4,true);
                            card.startFileprint();
                            starttime=millis();
                            break;
                        case 928: //M928 - Start SD write
                            starpos = (strchr(strchr_pointer + 5,'*'));
                            if(starpos != NULL)
                                {
                                    char* npos = strchr(cmdbuffer[bufindr], 'N');
                                    strchr_pointer = strchr(npos,' ') + 1;
                                    *(starpos-1) = '\0';
                                }
                            card.openLogFile(strchr_pointer+5);
                            break;

#endif //SDSUPPORT

                        case 31: //M31 take time since the start of the SD print or an M109 command
                            {
                                stoptime=millis();
                                char time[30];
                                unsigned long t=(stoptime-starttime)/1000;
                                int sec,min;
                                min=t/60;
                                sec=t%60;
                                sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
                                SERIAL_ECHO_START;
                                SERIAL_ECHOLN(time);
                                lcd_setstatus(time);
                                lcd_lib_update_screen();
                                autotempShutdown();
                            }
                            break;
                        case 42: //M42 -Change pin status via gcode
                            if (code_seen('S'))
                                {
                                    int pin_status = code_value();
                                    int pin_number = LED_PIN;
                                    if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
                                        pin_number = code_value();
#if 1
                                    for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
                                        {
                                            if (pgm_read_byte (&sensitive_pins[i])  == pin_number)
                                                {
                                                    char msg[30];
                                                    sprintf_P(msg, PSTR("Illegal pin set %i"), pin_number);
                                                    SERIAL_ECHO_START;
                                                    SERIAL_ECHOLN(msg);
                                                    lcd_setstatus(msg);
                                                    lcd_lib_update_screen();

                                                    ERROR_BEEP();
                                                    pin_number = -1;
                                                    return;
                                                    break;
                                                }
                                        }
#endif
#if defined(FAN_PIN) && FAN_PIN > -1
                                    if (pin_number == FAN_PIN)
                                        fanSpeed = pin_status;
#endif
                                    if (pin_number > -1)
                                        {
                                            pinMode(pin_number, OUTPUT);
                                            digitalWrite(pin_number, pin_status);
                                            analogWrite(pin_number, pin_status);
                                            char msg[30];
                                            sprintf_P(msg, PSTR("Pin %i set to %i"), pin_number, pin_status);
                                            SERIAL_ECHO_START;
                                            SERIAL_ECHOLN(msg);
                                            lcd_setstatus(msg);
                                            lcd_lib_update_screen();
                                        }
                                    else
                                        {
                                            SERIAL_ECHO_START;
                                            SERIAL_ECHOLNPGM("Need (P)in # and (S)tatus to set pin!");
                                        }
                                }
                            break;
                        case 104: // M104
                            if(setTargetedHotend(104))
                                {
                                    break;
                                }
                            if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
                            setWatch();
                            break;
                        case 140: // M140 set bed temp
                            if (code_seen('S')) setTargetBed(code_value());
                            break;
                        case 105 : // M105
                            if(setTargetedHotend(105))
                                {
                                    break;
                                }
#if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
                            SERIAL_PROTOCOLPGM("ok T:");
                            SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
                            SERIAL_PROTOCOLPGM(" /");
                            SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
                            SERIAL_PROTOCOLPGM(" B:");
                            SERIAL_PROTOCOL_F(degBed(),1);
                            SERIAL_PROTOCOLPGM(" /");
                            SERIAL_PROTOCOL_F(degTargetBed(),1);
#endif //TEMP_BED_PIN
#else
                            SERIAL_ERROR_START;
                            SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
#endif

                            SERIAL_PROTOCOLPGM(" @:");
                            SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));

                            SERIAL_PROTOCOLPGM(" B@:");
                            SERIAL_PROTOCOL(getHeaterPower(-1));

                            SERIAL_PROTOCOLPGM(" F:");
                            SERIAL_PROTOCOL((int) getFanSpeed());
                            SERIAL_PROTOCOLPGM(" / ");
                            SERIAL_PROTOCOL((int) fanSpeedOverride);

                            SERIAL_PROTOCOLLN("");

                            return;
                            break;
                        case 109:
                            {
                                // M109 - Wait for extruder heater to reach target.
                                if(setTargetedHotend(109))
                                    {
                                        break;
                                    }
                                printing_state = PRINT_STATE_HEATING;
                                LCD_MESSAGEPGM(MSG_HEATING);
                                LED_HEAT();
#ifdef AUTOTEMP
                                autotemp_enabled=false;
#endif
                                if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
#ifdef AUTOTEMP
                                if (code_seen('S')) autotemp_min=code_value();
                                if (code_seen('B')) autotemp_max=code_value();
                                if (code_seen('F'))
                                    {
                                        autotemp_factor=code_value();
                                        autotemp_enabled=true;
                                    }
#endif

                                setWatch();
                                codenum = millis();

                                /* See if we are heating up or cooling down */
                                bool target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

#ifdef TEMP_RESIDENCY_TIME
                                long residencyStart;
                                residencyStart = -1;
                                /* continue to loop until we have reached the target temp
                                _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
                                while((residencyStart == -1) ||
                                        (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) )
                                    {
#else
                                while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) )
                                    {
#endif //TEMP_RESIDENCY_TIME
                                        if( (millis() - codenum) > 1000UL )
                                            {
                                                //Print Temp Reading and remaining time every 1 second while heating up/cooling down
                                                SERIAL_PROTOCOLPGM("T:");
                                                SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
                                                SERIAL_PROTOCOLPGM(" E:");
                                                SERIAL_PROTOCOL((int)tmp_extruder);
#ifdef TEMP_RESIDENCY_TIME
                                                SERIAL_PROTOCOLPGM(" W:");
                                                if(residencyStart > -1)
                                                    {
                                                        codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
                                                        SERIAL_PROTOCOLLN( codenum );
                                                    }
                                                else
                                                    {
                                                        SERIAL_PROTOCOLLN( "?" );
                                                    }
#else
                                                SERIAL_PROTOCOLLN("");
#endif
                                                codenum = millis();
                                            }
                                       	runTasks(false,true);
#ifdef TEMP_RESIDENCY_TIME
                                        /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
                                        or when current temp falls outside the hysteresis after target temp was reached */
                                        if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
                                                (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
                                                (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS && (!target_direction || !CooldownNoWait)) )
                                            {
                                                residencyStart = millis();
                                            }
#endif //TEMP_RESIDENCY_TIME
                                    }
                                LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
                                starttime=millis();
                                true_e_position=0;
                                previous_millis_cmd = millis();
                            }
                            break;
                        case 190: // M190 - Wait for bed heater to reach target.
#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
                            printing_state = PRINT_STATE_HEATING_BED;
                            LCD_MESSAGEPGM(MSG_BED_HEATING);
                            LED_HEAT();
                            if (code_seen('S')) setTargetBed(code_value());
                            codenum = millis();
                            while(current_temperature_bed < target_temperature_bed - TEMP_WINDOW)
                                {
                                    if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
                                        {
                                            float tt=degHotend(active_extruder);
                                            SERIAL_PROTOCOLPGM("T:");
                                            SERIAL_PROTOCOL(tt);
                                            SERIAL_PROTOCOLPGM(" E:");
                                            SERIAL_PROTOCOL((int)active_extruder);
                                            SERIAL_PROTOCOLPGM(" B:");
                                            SERIAL_PROTOCOL_F(degBed(),1);
                                            SERIAL_PROTOCOLLN("");
                                            codenum = millis();
                                        }
                                 	runTasks(false,true);
                                }
                            LCD_MESSAGEPGM(MSG_BED_DONE);
                            previous_millis_cmd = millis();
#endif
                            break;

#if defined(FAN_PIN) && FAN_PIN > -1
                        case 106: //M106 Fan On

                            if (code_seen('S'))
                                {
                                    fanSpeed=constrain(code_value() * fanSpeedPercent / 100,0,255);
                                }
                            else
                                {
                                    fanSpeed = 255 * int(fanSpeedPercent) / 100;
                                }
                            if (fanSpeed < min_fan_speed) fanSpeed =0;
                            break;
                        case 107: //M107 Fan Off
                            fanSpeed = 0;
                            break;
#endif //FAN_PIN
#ifdef BARICUDA
                            // PWM for HEATER_1_PIN
#if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
                        case 126: //M126 valve open
                            if (code_seen('S'))
                                {
                                    ValvePressure=constrain(code_value(),0,255);
                                }
                            else
                                {
                                    ValvePressure=255;
                                }
                            break;
                        case 127: //M127 valve closed
                            ValvePressure = 0;
                            break;
#endif //HEATER_1_PIN

                            // PWM for HEATER_2_PIN
#if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
                        case 128: //M128 valve open
                            if (code_seen('S'))
                                {
                                    EtoPPressure=constrain(code_value(),0,255);
                                }
                            else
                                {
                                    EtoPPressure=255;
                                }
                            break;
                        case 129: //M129 valve closed
                            EtoPPressure = 0;
                            break;
#endif //HEATER_2_PIN
#endif

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
                        case 80: // M80 - ATX Power On
                            SET_OUTPUT(PS_ON_PIN); //GND
                            WRITE(PS_ON_PIN, PS_ON_AWAKE);
                            break;
#endif

                        case 81: // M81 - ATX Power Off

#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
                            st_synchronize();
                            suicide();
#elif defined(PS_ON_PIN) && PS_ON_PIN > -1
                            SET_OUTPUT(PS_ON_PIN);
                            WRITE(PS_ON_PIN, PS_ON_ASLEEP);
#endif
                            break;

                        case 82:
                            axis_relative_modes[3] = false;
                            break;
                        case 83:
                            axis_relative_modes[3] = true;
                            break;
                        case 18: //compatibility
                        case 84: // M84
                            if(code_seen('S'))
                                {
                                    stepper_inactive_time = code_value() * 1000;
                                }
                            else
                                {
                                    bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
                                    if(all_axis)
                                        {
                                            st_synchronize();
                                            disable_e0();
                                            disable_e1();
                                            disable_e2();
                                            finishAndDisableSteppers();
                                        }
                                    else
                                        {
                                            st_synchronize();
                                            if(code_seen('X')) disable_x();
                                            if(code_seen('Y')) disable_y();
                                            if(code_seen('Z')) disable_z();
#if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
                                            if(code_seen('E'))
                                                {
                                                    disable_e0();
                                                    disable_e1();
                                                    disable_e2();
                                                }
#endif
                                        }
                                }
                            break;
                        case 85: // M85
                            if (code_seen('S')) max_inactive_time = code_value() * 1000;
                            break;
                        case 92: // M92
                            for(int8_t i=0; i < NUM_AXIS; i++)
                                {
                                    if(code_seen(axis_codes[i]))
                                        {
                                            if(i == 3)   // E
                                                {
                                                    float value = code_value();
                                                    if(value < 20.0)
                                                        {
                                                            float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
                                                            max_e_jerk *= factor;
                                                            max_feedrate[i] *= factor;
                                                            axis_steps_per_sqr_second[i] *= factor;
                                                        }
                                                    axis_steps_per_unit[i] = value;
                                                }
                                            else
                                                {
                                                    axis_steps_per_unit[i] = code_value();
                                                }
                                        }
                                }
                            break;
                        case 115: // M115
                            SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
                            break;
                        case 70:  // M70 display message (replicator)
                        case 117: // M117 display message
                            starpos = (strchr(strchr_pointer + 5,'*'));
                            if(starpos!=NULL)
                                *(starpos-1)='\0';
                            lcd_setstatus(strchr_pointer + 5);
                            lcd_lib_update_screen();
                            break;
                        case 114: // M114
                            SERIAL_PROTOCOLPGM(" X:");
                            SERIAL_PROTOCOL(current_position[X_AXIS]);
                            SERIAL_PROTOCOLPGM(" Y:");
                            SERIAL_PROTOCOL(current_position[Y_AXIS]);
                            SERIAL_PROTOCOLPGM(" Z:");
                            SERIAL_PROTOCOL(current_position[Z_AXIS]);
                            SERIAL_PROTOCOLPGM(" E:");
                            SERIAL_PROTOCOL(current_position[E_AXIS]);

                            SERIAL_PROTOCOLPGM(MSG_COUNT_X);
                            SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
                            SERIAL_PROTOCOLPGM(" Y:");
                            SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
                            SERIAL_PROTOCOLPGM(" Z:");
                            SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

                            SERIAL_PROTOCOLLN("");
                            break;
                        case 120: // M120
                            enable_endstops(false) ;
                            break;
                        case 121: // M121
                            enable_endstops(true) ;
                            break;
                        case 119: // M119
                            SERIAL_PROTOCOLLN(MSG_M119_REPORT);
#if defined(X_MIN_PIN) && X_MIN_PIN > -1
                            SERIAL_PROTOCOLPGM(MSG_X_MIN);
                            SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if defined(X_MAX_PIN) && X_MAX_PIN > -1
                            SERIAL_PROTOCOLPGM(MSG_X_MAX);
                            SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
                            SERIAL_PROTOCOLPGM(MSG_Y_MIN);
                            SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
                            SERIAL_PROTOCOLPGM(MSG_Y_MAX);
                            SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
                            SERIAL_PROTOCOLPGM(MSG_Z_MIN);
                            SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
#if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
                            SERIAL_PROTOCOLPGM(MSG_Z_MAX);
                            SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_ENDSTOPS_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
#endif
                            break;
                        //TODO: update for all axis, use for loop
                        case 201: // M201
                            for(int8_t i=0; i < NUM_AXIS; i++)
                                {
                                    if(code_seen(axis_codes[i]))
                                        {
                                            max_acceleration_units_per_sq_second[i] = code_value();
                                        }
                                }
                            // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
                            reset_acceleration_rates();
                            break;
#if 0 // Not used for Sprinter/grbl gen6
                        case 202: // M202
                            for(int8_t i=0; i < NUM_AXIS; i++)
                                {
                                    if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
                                }
                            break;
#endif
                        case 203: // M203 max feedrate mm/sec
                            for(int8_t i=0; i < NUM_AXIS; i++)
                                {
                                    if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
                                }
                            break;
                        case 204: // M204 acceleration: S - normal moves;  T - filament only moves
                            {
                                if(code_seen('S')) acceleration = code_value() ;
                                if(code_seen('T')) retract_acceleration = code_value() ;
                            }
                            break;
                        case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
                            {
                                if(code_seen('S')) minimumfeedrate = code_value();
                                if(code_seen('T')) mintravelfeedrate = code_value();
                                if(code_seen('B')) minsegmenttime = code_value() ;
                                if(code_seen('X')) max_xy_jerk = code_value() ;
                                if(code_seen('Z')) max_z_jerk = code_value() ;
                                if(code_seen('E')) max_e_jerk = code_value() ;
                            }
                            break;
                        case 206: // M206 additional homing offset
                            for(int8_t i=0; i < 3; i++)
                                {
                                    if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
                                }
                            break;
#ifdef FWRETRACT
                        case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
                            {
                                if(code_seen('S'))
                                    {
                                        retract_length = code_value() ;
                                    }
                                if(code_seen('F'))
                                    {
                                        retract_feedrate = code_value() ;
                                    }
                                if(code_seen('Z'))
                                    {
                                        retract_zlift = code_value() ;
                                    }
                            } break;
                        case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
                            {
                                if(code_seen('S'))
                                    {
                                        retract_recover_length = code_value() ;
                                    }
                                if(code_seen('F'))
                                    {
                                        retract_recover_feedrate = code_value() ;
                                    }
                            } break;
                        case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
                            {
                                if(code_seen('S'))
                                    {
                                        int t= code_value() ;
                                        switch(t)
                                            {
                                                case 0: autoretract_enabled=false; retracted=false; break;
                                                case 1: autoretract_enabled=true; retracted=false; break;
                                                default:
                                                    SERIAL_ECHO_START;
                                                    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
                                                    SERIAL_ECHO(cmdbuffer[bufindr]);
                                                    SERIAL_ECHOLNPGM("\"");
                                            }
                                    }

                            } break;
#endif // FWRETRACT
#if EXTRUDERS > 1
                        case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
                            {
                                if(setTargetedHotend(218))
                                    {
                                        break;
                                    }
                                if(code_seen('X'))
                                    {
                                        extruder_offset[X_AXIS][tmp_extruder] = code_value();
                                    }
                                if(code_seen('Y'))
                                    {
                                        extruder_offset[Y_AXIS][tmp_extruder] = code_value();
                                    }
                                SERIAL_ECHO_START;
                                SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
                                for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
                                    {
                                        SERIAL_ECHO(" ");
                                        SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
                                        SERIAL_ECHO(",");
                                        SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
                                    }
                                SERIAL_ECHOLN("");
                            } break;
#endif
                        case 220: // M220 S<factor in percent>- set speed factor override percentage
                            {
                                if(code_seen('S'))
                                    {
                                        feedmultiply = code_value() ;
                                    }
                            }
                            break;
                        case 221: // M221 S<factor in percent>- set extrude factor override percentage
                            {
                                if(code_seen('S'))
                                    {
                                        extrudemultiply[active_extruder] = code_value() ;
                                    }
                            }
                            break;

#if NUM_SERVOS > 0
                        case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
                            {
                                int servo_index = -1;
                                int servo_position = 0;
                                if (code_seen('P'))
                                    servo_index = code_value();
                                if (code_seen('S'))
                                    {
                                        servo_position = code_value();
                                        if ((servo_index >= 0) && (servo_index < NUM_SERVOS))
                                            {
                                                servos[servo_index].write(servo_position);
                                            }
                                        else
                                            {
                                                SERIAL_ECHO_START;
                                                SERIAL_ECHO("Servo ");
                                                SERIAL_ECHO(servo_index);
                                                SERIAL_ECHOLN(" out of range");
                                            }
                                    }
                                else
                                    if (servo_index >= 0)
                                        {
                                            SERIAL_PROTOCOL(MSG_OK);
                                            SERIAL_PROTOCOL(" Servo ");
                                            SERIAL_PROTOCOL(servo_index);
                                            SERIAL_PROTOCOL(": ");
                                            SERIAL_PROTOCOL(servos[servo_index].read());
                                            SERIAL_PROTOCOLLN("");
                                        }
                            }
                            break;
#endif // NUM_SERVOS > 0

                        case 300: // M300
                            {
                                int beepS = code_seen('S') ? code_value() : 880;
                                int beepP = code_seen('P') ? code_value() : 100;
                                if (beepS > 0)
                                    {
#if LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) )
#if BEEPER > 0
                                        tone(BEEPER, beepS);
                                        delay(beepP);
                                        noTone(BEEPER);
#elif defined(ULTRALCD)
                                        lcd_buzz(beepS, beepP);
#endif

#else
                                        lcd_lib_beep_ext(beepS,beepP);
#endif // M300
                                    }
                                else  // even if we don't beep, we need to delay because it might be used for some timing purpose (it would be bad practice, but just being defensive)
                                    {
                                        delay(beepP);
                                    }
                            }
                            break;
#if MOODLIGHT
                        case 420: 	// set RGB "mood light"
                            {
                                int r=0; int g=0; int b=0;
                                if (code_seen('R')) r=code_value();
                                if (code_seen('E')) g=code_value();
                                // yes, this is an E, not a G
                                if (code_seen('B')) b=code_value();
                                lcd_lib_led_color(r,g,b, true);
                            }
                            break;
                        case 421:
                            {
                                int s = 0;
                                if (code_seen('S')) s=code_value();
                                analogWrite(LED_PIN, s);
                                led_brightness_level = (s * 100) >> 8;
                            }
                            break;
#endif

#ifdef PIDTEMP
                        case 301: // M301
                            {
                                if(code_seen('P')) Kp = code_value();
                                if(code_seen('I')) Ki = scalePID_i(code_value());
                                if(code_seen('D')) Kd = scalePID_d(code_value());
                                if(code_seen('R'))
                                    {
                                        Kp = DEFAULT_Kp;
                                        Ki = scalePID_i(DEFAULT_Ki);
                                        Kd = scalePID_d(DEFAULT_Kd);
                                    }


#ifdef PID_ADD_EXTRUSION_RATE
                                if(code_seen('C')) Kc = code_value();
#endif

                                updatePID();
                                SERIAL_PROTOCOL(MSG_OK);
                                SERIAL_PROTOCOL(" p:");
                                SERIAL_PROTOCOL(Kp);
                                SERIAL_PROTOCOL(" i:");
                                SERIAL_PROTOCOL(unscalePID_i(Ki));
                                SERIAL_PROTOCOL(" d:");
                                SERIAL_PROTOCOL(unscalePID_d(Kd));
#ifdef PID_ADD_EXTRUSION_RATE
                                SERIAL_PROTOCOL(" c:");
                                //Kc does not have scaling applied above, or in resetting defaults
                                SERIAL_PROTOCOL(Kc);
#endif
                                SERIAL_PROTOCOLLN("");
                            }
                            break;
#endif //PIDTEMP
#ifdef PIDTEMPBED
                        case 304: // M304
                            {
                                if(code_seen('P')) bedKp = code_value();
                                if(code_seen('I')) bedKi = scalePID_i(code_value());
                                if(code_seen('D')) bedKd = scalePID_d(code_value());

                                updatePID();
                                SERIAL_PROTOCOL(MSG_OK);
                                SERIAL_PROTOCOL(" p:");
                                SERIAL_PROTOCOL(bedKp);
                                SERIAL_PROTOCOL(" i:");
                                SERIAL_PROTOCOL(unscalePID_i(bedKi));
                                SERIAL_PROTOCOL(" d:");
                                SERIAL_PROTOCOL(unscalePID_d(bedKd));
                                SERIAL_PROTOCOLLN("");
                            }
                            break;
#endif //PIDTEMP
#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
                        case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
                            {
                                const uint8_t NUM_PULSES=16;
                                const float PULSE_LENGTH=0.01524;
                                for(int i=0; i < NUM_PULSES; i++)
                                    {
                                        WRITE(PHOTOGRAPH_PIN, HIGH);
                                        _delay_ms(PULSE_LENGTH);
                                        WRITE(PHOTOGRAPH_PIN, LOW);
                                        _delay_ms(PULSE_LENGTH);
                                    }
                                delay(7.33);
                                for(int i=0; i < NUM_PULSES; i++)
                                    {
                                        WRITE(PHOTOGRAPH_PIN, HIGH);
                                        _delay_ms(PULSE_LENGTH);
                                        WRITE(PHOTOGRAPH_PIN, LOW);
                                        _delay_ms(PULSE_LENGTH);
                                    }
                            }
                            break;
#endif


#ifdef PREVENT_DANGEROUS_EXTRUDE
                        case 302: // allow cold extrudes, or set the minimum extrude temperature
                            {
                                float temp = .0;
                                if (code_seen('S')) temp=code_value();
                                set_extrude_min_temp(temp);
                            }
                            break;
#endif
                        case 303: // M303 PID autotune
                            {
                                float temp = 150.0;
                                int e=0;
                                int c=5;
                                if (code_seen('E')) e=code_value();
                                if (e<0)
                                    temp=70;
                                if (code_seen('S')) temp=code_value();
                                if (code_seen('C')) c=code_value();
                                PID_autotune(temp, e, c);
                            }
                            break;
                        case 400: // M400 finish all moves
                            {
                                st_synchronize();
                            }
                            break;
                        case 500: // M500 Store settings in EEPROM
                            {
                                Config_StoreSettings();
                                LCD_MESSAGEPGM("SETTINGS SAVED");
                                lcd_lib_beep();
                            }
                            break;
                        case 501: // M501 Read settings from EEPROM
                            {
                                Config_RetrieveSettings();
                            }
                            break;
                        case 502: // M502 Revert to default settings
                            {
                                Config_ResetDefault();
                                LCD_MESSAGEPGM("DEFAULT SETTINGS");
                                lcd_lib_beep();
                            }
                            break;
                        case 503: // M503 print settings currently in memory
                            {
                                Config_PrintSettings();
                            }
                            break;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
                        case 540:
                            {
                                if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
                            }
                            break;
#endif
#ifdef FILAMENTCHANGEENABLE
                        case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
                            {
                                float target[4];
                                float lastpos[4];
                                target[X_AXIS]=current_position[X_AXIS];
                                target[Y_AXIS]=current_position[Y_AXIS];
                                target[Z_AXIS]=current_position[Z_AXIS];
                                target[E_AXIS]=current_position[E_AXIS];
                                lastpos[X_AXIS]=current_position[X_AXIS];
                                lastpos[Y_AXIS]=current_position[Y_AXIS];
                                lastpos[Z_AXIS]=current_position[Z_AXIS];
                                lastpos[E_AXIS]=current_position[E_AXIS];
                                //retract by E
                                if(code_seen('E'))
                                    {
                                        target[E_AXIS]+= code_value();
                                    }
                                else
                                    {
#ifdef FILAMENTCHANGE_FIRSTRETRACT
                                        target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
#endif
                                    }
                                plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

                                //lift Z
                                if(code_seen('Z'))
                                    {
                                        target[Z_AXIS]+= code_value();
                                    }
                                else
                                    {
#ifdef FILAMENTCHANGE_ZADD
                                        target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;
#endif
                                    }
                                plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

                                //move xy
                                if(code_seen('X'))
                                    {
                                        target[X_AXIS]+= code_value();
                                    }
                                else
                                    {
#ifdef FILAMENTCHANGE_XPOS
                                        target[X_AXIS]= FILAMENTCHANGE_XPOS ;
#endif
                                    }
                                if(code_seen('Y'))
                                    {
                                        target[Y_AXIS]= code_value();
                                    }
                                else
                                    {
#ifdef FILAMENTCHANGE_YPOS
                                        target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
#endif
                                    }

                                plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

                                if(code_seen('L'))
                                    {
                                        target[E_AXIS]+= code_value();
                                    }
                                else
                                    {
#ifdef FILAMENTCHANGE_FINALRETRACT
                                        target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
#endif
                                    }

                                plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder);

                                //finish moves
                                st_synchronize();
                                //disable extruder steppers so filament can be removed
                                disable_e0();
                                disable_e1();
                                disable_e2();
                                delay(100);
                                LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
                                uint8_t cnt=0;
                                while(!lcd_clicked())
                                    {
                                        cnt++;
                                        manage_heater();
                                        manage_inactivity();
                                        lcd_update();
                                        lifetime_stats_tick();
                                        if(cnt==0)
                                            {
#if BEEPER > 0
                                                SET_OUTPUT(BEEPER);

                                                WRITE(BEEPER,HIGH);
                                                delay(3);
                                                WRITE(BEEPER,LOW);
                                                delay(3);
#else
                                                lcd_buzz(1000/6,100);
#endif
                                            }
                                    }

                                //return to normal
                                if(code_seen('L'))
                                    {
                                        target[E_AXIS]+= -code_value();
                                    }
                                else
                                    {
#ifdef FILAMENTCHANGE_FINALRETRACT
                                        target[E_AXIS]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
#endif
                                    }
                                current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
                                plan_set_e_position(current_position[E_AXIS]);
                                plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //should do nothing
                                plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move xy back
                                plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder); //move z back
                                plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder); //final untretract
                            }
                            break;
#endif //FILAMENTCHANGEENABLE
#ifdef ENABLE_ULTILCD2
                        case 601: //Pause in UltiLCD2, X[pos] Y[pos] Z[relative lift] L[later retract distance]
                            moveToPausePosition();
                            //disable extruder steppers so filament can be removed
                            disable_e0();
                            disable_e1();
                            disable_e2();
                            while(card.pause)
                                {
                                    if (!is_message_shown) LCD_MESSAGEPGM ("PAUSED");
										runTasks(false,true);
                                }
                            clear_message();
                            resumeFromPausePosition();

                            break;
                        case 602:
                            // same as 601, but does not block.  gcode can be executed while paused
                            // use 603 to reume
                            moveToPausePosition();
                            disable_e0();
                            disable_e1();
                            disable_e2();
                            break;
                        case 603:
                            resumeFromPausePosition();
                            break;



#endif//ENABLE_ULTILCD2
                        case 906: // M906 per http://reprap.org/wiki/G-code
                        case 907: // M907 Set digital trimpot motor current using axis codes.
                            {
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
                                for(int i=0; i<NUM_AXIS; i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
                                if(code_seen('B')) digipot_current(4,code_value());
                                if(code_seen('S')) for(int i=0; i<=4; i++) digipot_current(i,code_value());
#endif
#if defined(MOTOR_CURRENT_PWM_XY_PIN) && MOTOR_CURRENT_PWM_XY_PIN > -1
                                if(code_seen('X')) digipot_current(0, code_value());
#endif
#if defined(MOTOR_CURRENT_PWM_Z_PIN) && MOTOR_CURRENT_PWM_Z_PIN > -1
                                if(code_seen('Z')) digipot_current(1, code_value());
#endif
#if defined(MOTOR_CURRENT_PWM_E_PIN) && MOTOR_CURRENT_PWM_E_PIN > -1
                                if(code_seen('E')) digipot_current(2, code_value());
#endif
                            }
                            break;
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
                        case 908: // M908 Control digital trimpot directly.
                            {
                                uint8_t channel,current;
                                if(code_seen('P')) channel=code_value();
                                if(code_seen('S')) current=code_value();
                                digitalPotWrite(channel, current);
                            }
                            break;
#endif
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
                        case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
                            {
                                if(code_seen('S')) for(int i=0; i<=4; i++) microstep_mode(i,code_value());
                                for(int i=0; i<NUM_AXIS; i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
                                if(code_seen('B')) microstep_mode(4,code_value());
                                microstep_readings();
                            }
                            break;
#endif
#if defined(X_MS1_PIN) && X_MS1_PIN > -1
                        case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
                            {
                                if(code_seen('S')) switch((int)code_value())
                                        {
                                            case 1:
                                                for(int i=0; i<NUM_AXIS; i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
                                                if(code_seen('B')) microstep_ms(4,code_value(),-1);
                                                break;
                                            case 2:
                                                for(int i=0; i<NUM_AXIS; i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
                                                if(code_seen('B')) microstep_ms(4,-1,code_value());
                                                break;
                                        }
                                microstep_readings();
                            }
                            break;
#endif
                        case 999: // M999: Restart after being stopped
                            Stopped = false;
                            lcd_reset_alert_level();
                            gcode_LastN = Stopped_gcode_LastN;
                            FlushSerialRequestResend();
                            break;
#ifdef ENABLE_ULTILCD2
                        case 10000://M10000 - Clear the whole LCD
                            lcd_lib_clear();
                            break;
                        case 10001://M10001 - Draw text on LCD, M10002 X0 Y0 SText
                            {
                                uint8_t x = 0, y = 0;
                                if (code_seen('X')) x = code_value_long();
                                if (code_seen('Y')) y = code_value_long();
                                if (code_seen('S')) lcd_lib_draw_string(x, y, &cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1]);
                            }
                            break;
                        case 10002://M10002 - Draw inverted text on LCD, M10002 X0 Y0 SText
                            {
                                uint8_t x = 0, y = 0;
                                if (code_seen('X')) x = code_value_long();
                                if (code_seen('Y')) y = code_value_long();
                                if (code_seen('S')) lcd_lib_clear_string(x, y, &cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1]);
                            }
                            break;
                        case 10003://M10003 - Draw square on LCD, M10003 X1 Y1 W10 H10
                            {
                                uint8_t x = 0, y = 0, w = 1, h = 1;
                                if (code_seen('X')) x = code_value_long();
                                if (code_seen('Y')) y = code_value_long();
                                if (code_seen('W')) w = code_value_long();
                                if (code_seen('H')) h = code_value_long();
                                lcd_lib_set(x, y, x + w, y + h);
                            }
                            break;
                        case 10004://M10004 - Draw shaded square on LCD, M10004 X1 Y1 W10 H10
                            {
                                uint8_t x = 0, y = 0, w = 1, h = 1;
                                if (code_seen('X')) x = code_value_long();
                                if (code_seen('Y')) y = code_value_long();
                                if (code_seen('W')) w = code_value_long();
                                if (code_seen('H')) h = code_value_long();
                                lcd_lib_draw_shade(x, y, x + w, y + h);
                            }
                            break;
                        case 10005://M10005 - Draw shaded square on LCD, M10004 X1 Y1 W10 H10
                            {
                                uint8_t x = 0, y = 0, w = 1, h = 1;
                                if (code_seen('X')) x = code_value_long();
                                if (code_seen('Y')) y = code_value_long();
                                if (code_seen('W')) w = code_value_long();
                                if (code_seen('H')) h = code_value_long();
                                lcd_lib_draw_shade(x, y, x + w, y + h);
                            }
                            break;
                        case 10010://M10010 - Request LCD screen button info (R:[rotation difference compared to previous request] B:[button down])
                            {
                                SERIAL_PROTOCOLPGM("ok R:");
                                SERIAL_PROTOCOL_F(lcd_lib_encoder_pos, 10);
                                lcd_lib_encoder_pos = 0;
                                if (lcd_lib_button_down)
                                    SERIAL_PROTOCOLLNPGM(" B:1");
                                else
                                    SERIAL_PROTOCOLLNPGM(" B:0");
                                return;
                            }
                            break;
#endif//ENABLE_ULTILCD2
                    }
            }

        else
            if(code_seen('T'))
                {
                    tmp_extruder = code_value();
                    if(tmp_extruder >= EXTRUDERS)
                        {
                            SERIAL_ECHO_START;
                            SERIAL_ECHO("T");
                            SERIAL_ECHO(tmp_extruder);
                            SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
                        }
                    else
                        {
                            boolean make_move = false;
                            if(code_seen('F'))
                                {
                                    make_move = true;
                                    next_feedrate = code_value();
                                    if(next_feedrate > 0.0)
                                        {
                                            feedrate = next_feedrate;
                                        }
                                }
#if EXTRUDERS > 1
                            if(tmp_extruder != active_extruder)
                                {
                                    // Save current position to return to after applying extruder offset
                                    memcpy(destination, current_position, sizeof(destination));
                                    // Offset extruder (only by XY)
                                    int i;
                                    for(i = 0; i < 2; i++)
                                        {
                                            current_position[i] = current_position[i] -
                                                                  extruder_offset[i][active_extruder] +
                                                                  extruder_offset[i][tmp_extruder];
                                        }
                                    // Set the new active extruder and position
                                    active_extruder = tmp_extruder;
                                    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
                                    // Move to the old position if 'F' was in the parameters
                                    if(make_move && Stopped == false)
                                        {
                                            prepare_move();
                                        }
                                }
#endif
                            SERIAL_ECHO_START;
                            SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
                            SERIAL_PROTOCOLLN((int)active_extruder);
                        }
                }

            else
                {
                    SERIAL_ECHO_START;
                    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
                    SERIAL_ECHO(cmdbuffer[bufindr]);
                    SERIAL_ECHOLNPGM("\"");
                }
    printing_state = PRINT_STATE_NORMAL;

    ClearToSend();
}
//-----------------------------------------------------------------------------------------------------------------
void get_coordinates()
{
    bool seen[4]= {false,false,false,false};
    for(int8_t i=0; i < NUM_AXIS; i++)
        {
            if(code_seen(axis_codes[i]))
                {
                    destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
                    seen[i]=true;
                }
            else
                {
                    destination[i] = current_position[i]; //Are these else lines really needed?
                }
        }
    if(code_seen('F'))
        {
            next_feedrate = code_value();
            if(next_feedrate > 0.0) feedrate = next_feedrate;
        }

    if(code_seen('Q'))
        {
            next_quality = code_value();
            if(next_quality > 0.0) quality = next_quality;
        }


#ifdef FWRETRACT
    if(autoretract_enabled)
        {
            if( !(seen[X_AXIS] || seen[Y_AXIS] || seen[Z_AXIS]) && seen[E_AXIS])
                {
                    float echange=destination[E_AXIS]-current_position[E_AXIS];
                    if(echange<-MIN_RETRACT) //retract
                        {
                            if(!retracted)
                                {
                                    destination[Z_AXIS]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
                                    //if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
                                    float correctede=-echange-retract_length;
                                    //to generate the additional steps, not the destination is changed, but inversely the current position
                                    current_position[E_AXIS]+=-correctede;
                                    feedrate=retract_feedrate;
                                    retracted=true;
                                }
                        }
                    else
                        if(echange>MIN_RETRACT) //retract_recover
                            {
                                if(retracted)
                                    {
                                        //current_position[Z_AXIS]+=-retract_zlift;
                                        //if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
                                        float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
                                        current_position[E_AXIS]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
                                        feedrate=retract_recover_feedrate;
                                        retracted=false;
                                    }
                            }
                }
        }
#endif //FWRETRACT
}
//-----------------------------------------------------------------------------------------------------------------
void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
    bool relative_mode_backup = relative_mode;
    relative_mode = true;
#endif
    get_coordinates();
#ifdef SF_ARC_FIX
    relative_mode=relative_mode_backup;
#endif

    if(code_seen('I'))
        {
            offset[0] = code_value();
        }
    else
        {
            offset[0] = 0.0;
        }
    if(code_seen('J'))
        {
            offset[1] = code_value();
        }
    else
        {
            offset[1] = 0.0;
        }
}


void prepare_move()
{
    clamp_to_software_endstops(destination);

    previous_millis_cmd = millis();
#ifdef DELTA
    float difference[NUM_AXIS];
    for (int8_t i=0; i < NUM_AXIS; i++)
        {
            difference[i] = destination[i] - current_position[i];
        }
    float cartesian_mm = sqrt(sq(difference[X_AXIS]) +
                              sq(difference[Y_AXIS]) +
                              sq(difference[Z_AXIS]));
    if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[E_AXIS]); }
    if (cartesian_mm < 0.000001) { return; }
    float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
    int steps = max(1, int(DELTA_SEGMENTS_PER_SECOND * seconds));
    // SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
    // SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
    // SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);
    for (int s = 1; s <= steps; s++)
        {
            float fraction = float(s) / float(steps);
            for(int8_t i=0; i < NUM_AXIS; i++)
                {
                    destination[i] = current_position[i] + difference[i] * fraction;
                }
            calculate_delta(destination);
            plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
                             destination[E_AXIS], feedrate*feedmultiply/60/100.0,
                             active_extruder);
        }
#else
    // Do not use feedmultiply for E or Z only moves
    if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS]))
        {
            if( (current_position[E_AXIS] != destination [E_AXIS]) ||(current_position[Z_AXIS] != destination [Z_AXIS]))											// handle cases with 0  movement.  for example, a retract command when retraction length is zero will hang!
                plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60.0, active_extruder,quality);
        }
    else
        {
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60.0/100.0, active_extruder,quality);
        }
#endif
    for(int8_t i=0; i < NUM_AXIS; i++)
        {
            current_position[i] = destination[i];
        }
}

void prepare_arc_move(char isclockwise)
{
    float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

    // Trace the arc
    mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    for(int8_t i=0; i < NUM_AXIS; i++)
        {
            current_position[i] = destination[i];
        }
    previous_millis_cmd = millis();
}


