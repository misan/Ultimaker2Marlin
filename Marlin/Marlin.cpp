/* -*- c++ -*- */

/*
Reprap firmware based on Sprinter and grbl.
Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
This firmware is a mashup between Sprinter and grbl.
(https://github.com/kliment/Sprinter)
(https://github.com/simen/grbl/tree)

It has preliminary support for Matthew Roberts advance algorithm
http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
*/

#include "Marlin.h"

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
#include "PeriodTimer.h"
#include "UltiLCD2_low_lib.h"


#define VERSION_STRING  "LarsUM2 3.0.4"
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif

float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2

int extrudemultiply[EXTRUDERS]=ARRAY_BY_EXTRUDERS(100, 100, 100); //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]= {0,0,0};
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { BASELINE_XLIMIT, BASELINE_YLIMIT, Z_MAX_POS };


float Y_MAX_LENGTH = BASELINE_YLIMIT - Y_MIN_POS;
float X_MAX_LENGTH = BASELINE_XLIMIT - X_MIN_POS;



static unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
static unsigned long lastMotorCheck = 0;


#ifdef DHT_ENVIRONMENTAL_SENSOR

#endif


// Extruder offset, only in XY plane
#if EXTRUDERS > 1
float extruder_offset[2][EXTRUDERS] =
	{
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
	EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
#endif
	};
#endif
uint8_t active_extruder = 0;


#ifdef SERVO_ENDSTOPS
int servo_endstops[] = SERVO_ENDSTOPS;
int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif



uint8_t printing_state;


#ifdef DELTA
static float delta[3] = {0.0, 0.0, 0.0};
#endif


#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif




unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = 0;
unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;
unsigned long starttime=0;
unsigned long stoptime=0;
uint8_t tmp_extruder;
unsigned long last_user_interaction=0;
uint8_t Stopped = false;

#if NUM_SERVOS > 0
Servo servos[NUM_SERVOS];
#endif


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup_killpin()
	{
#if defined(KILL_PIN) && KILL_PIN > -1
	SET_INPUT(KILL_PIN);
	WRITE(KILL_PIN,HIGH);
#endif
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup_photpin()
	{
#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
	SET_OUTPUT(PHOTOGRAPH_PIN);
	WRITE(PHOTOGRAPH_PIN, LOW);
#endif
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup_powerhold()
	{
#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
	SET_OUTPUT(SUICIDE_PIN);
	WRITE(SUICIDE_PIN, HIGH);
#endif
#if defined(PS_ON_PIN) && PS_ON_PIN > -1
	SET_OUTPUT(PS_ON_PIN);
	WRITE(PS_ON_PIN, PS_ON_AWAKE);
#endif
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void suicide()
	{
#if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
	SET_OUTPUT(SUICIDE_PIN);
	WRITE(SUICIDE_PIN, LOW);
#endif
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void servo_init()
	{
#if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
	servos[0].attach(SERVO0_PIN);
#endif
#if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
	servos[1].attach(SERVO1_PIN);
#endif
#if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
	servos[2].attach(SERVO2_PIN);
#endif
#if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
	servos[3].attach(SERVO3_PIN);
#endif
#if (NUM_SERVOS >= 5)
#error "TODO: enter initalisation code for more servos"
#endif

	// Set position of Servo Endstops that are defined
#ifdef SERVO_ENDSTOPS
	for(int8_t i = 0; i < 3; i++)
		{
		if(servo_endstops[i] > -1)
			{
			servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
			}
		}
#endif
	}
//-----------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------
void spewHello()
	{
	SERIAL_ECHOLNPGM(STRING_BIG_LINE);
	SERIAL_PROTOCOLLNPGM("start");
	SERIAL_ECHO_START;

	// Check startup - does nothing if bootloader sets MCUSR to 0
	byte mcu = MCUSR;
	if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
	if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
	if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
	if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
	if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
	MCUSR=0;

	SERIAL_ECHOPGM(MSG_MARLIN);
	SERIAL_ECHOLNPGM(VERSION_STRING);
#ifdef STRING_VERSION_CONFIG_H
#ifdef STRING_CONFIG_H_AUTHOR
	SERIAL_ECHO_START;
	SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
	SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
	SERIAL_ECHOPGM(MSG_AUTHOR);
	SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
	SERIAL_ECHOPGM("Compiled: ");
	SERIAL_ECHOLNPGM(__DATE__);
#endif
#endif
	SERIAL_ECHO_START;
	SERIAL_ECHOPGM(MSG_FREE_MEMORY);
	SERIAL_ECHOLN(freeMemory());
	SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
	SERIAL_ECHO((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
	SERIAL_ECHOPAIR(" (",(unsigned long) BLOCK_BUFFER_SIZE);
	SERIAL_ECHOPAIR(" x ",(unsigned long)sizeof(block_t) );
	SERIAL_ECHOLNPGM(" bytes) ");

	SERIAL_ECHOPAIR("  LCD_CACHE=",(unsigned long)sizeof(lcd_cache_new) );
	SERIAL_ECHOLNPGM(" bytes");

	SERIAL_ECHOPAIR("  Screen buffer= ",(unsigned long)LCD_GFX_WIDTH * (LCD_GFX_HEIGHT+1) / 8);
	SERIAL_ECHOLNPGM(" bytes");

	SERIAL_ECHOPAIR("  Command buffer= ",(unsigned long)BUFSIZE);
	SERIAL_ECHOPAIR("  commands for  ",(unsigned long)BUFSIZE*MAX_CMD_SIZE);
	SERIAL_ECHOLNPGM(" bytes");

	SERIAL_ECHOPAIR("  Serial RX comm buffer= ",(unsigned long)sizeof (rx_buffer) );
	SERIAL_ECHOLNPGM(" bytes");

	SERIAL_ECHOPAIR("  Material name buffers= ",(unsigned long)(MATERIAL_NAME_LENGTH* (1+EXTRUDERS)));
	SERIAL_ECHOLNPGM(" bytes");

	SERIAL_ECHOPAIR("  Millisecond timer= ",(unsigned long)millis());
	SERIAL_ECHOLNPGM(" ms");

	read_INTERNAL_AVR_VCC();

	SERIAL_ECHOPGM("VCC: ");
	SERIAL_ECHO(readAVR_VCC());
	SERIAL_ECHOLNPGM("VDC");

	SERIAL_ECHOPGM("PSU Voltage: ");
	SERIAL_ECHO(readVoltage());
	SERIAL_ECHOLNPGM("VDC");
	updateAuxTempSensors();

	SERIAL_ECHOLNPGM(STRING_BIG_LINE);
	}

// extern unsigned long bins_a[16];
// extern unsigned long bins_d[16];


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup()
	{
	setup_killpin();
	setup_powerhold();
	MYSERIAL.begin(BAUDRATE);
	spewHello();
	lcd_cache_new.init();
	LED_DIM_TIME = 30;
	initQueue();
	material_name_buf[0]=0;
	material_name[0][0]=0;
	old_zlift=0.0;
	old_retraction =0.0;


	run_history = false;
	lcd_init();
	if (!lcd_material_verify_material_settings())
		{
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM("Invalid material settings found, resetting to defaults");
		lcd_material_reset_defaults();
		for(uint8_t e=0; e<EXTRUDERS; e++)
			lcd_material_set_material(0, e);
		}
	lcd_material_read_current_material();
	SERIAL_ECHOLNPGM(STRING_BIG_LINE);
	// loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
	Config_RetrieveSettings();
	lifetime_stats_init();
	tp_init();    // Initialize temperature loop
	plan_init();  // Initialize planner;
	watchdog_init();
	st_init();    // Initialize stepper, this enables interrupts!
	setup_photpin();
	servo_init();
	pinMode(MOBO_TEMP_SENSOR_PIN,INPUT);
	// 	pinMode(CURRENT_SENSOR_PIN,INPUT);
	pinMode(EXT_VOLTAGE_REF_PIN,INPUT);
	pinMode(HEAD_TEMP_SENSOR_PIN,INPUT);
	pinMode(STEPPER_TEMP_SENSOR_PIN,INPUT);

	pinMode(HEAD_AUX_TEMP_SENSOR_PIN,INPUT);

#ifdef DHT_ENVIRONMENTAL_SENSOR
	dht.begin();
	DHT11_disable = false;
	updateAmbientSensor();
	if (last_mobo_temp_reading==0) 
		{ 
		SERIAL_ECHOLNPGM("DHT11 did not respond, disabling.");
		DHT11_disable = true;
#ifdef MOBO_TEMP_SENSOR_PIN
		if (MOBO_TEMP_SENSOR_PIN>-1) 
			{
			last_mobo_temp_reading = last_mobo_temp;
			}
		SERIAL_ECHOPAIR("Motherboard temp= ", (unsigned long) last_mobo_temp_reading);
		SERIAL_ECHOLNPGM(" 'C");

#endif // MOBO_TEMP_SENSOR_PIN
		} 
	else 
		{

		SERIAL_ECHOPAIR("temp= ", (unsigned long) last_mobo_temp_reading);
		SERIAL_ECHOLNPGM(" 'C");
		SERIAL_ECHOPAIR("RH= ", (unsigned long) last_humid);
		SERIAL_ECHOLNPGM("%");

		}

#endif

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
	SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
	digitalWrite(CONTROLLERFAN_PIN,1);
#endif



#if defined(HEAD_FAN_PIN) && HEAD_FAN_PIN > -1
	pinMode (HEAD_FAN_PIN,OUTPUT);
	digitalWrite(HEAD_FAN_PIN,1);
#endif
#if defined(STEPPER_FAN_PIN) && STEPPER_FAN_PIN > -1
	SET_OUTPUT(STEPPER_FAN_PIN);
	digitalWrite(STEPPER_FAN_PIN,1);
#endif



#if EXTENDED_BEEP
	// play a happy wake tune
	lcd_lib_beep_ext(880,50);
	lcd_lib_beep_ext(792,50);
	lcd_lib_beep_ext(880,25);
	lcd_lib_beep_ext(1320,100);
#endif
	SERIAL_ECHOLNPGM(STRING_BIG_LINE);
	lastMotorCheck=previous_millis_cmd=last_user_interaction = millis();
	starttime =0;
	last_print_name[0]=0;
	estimatedTime=0;
	//     memset (bins_a,0,sizeof (bins_a));
	//     memset (bins_d,0,sizeof (bins_d));

	stoptime = 0;
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


#define BED_PID_CHECK_INTERVAL 100
#define EXTRUDER_PID_CHECK_INTERVAL 100
#define UI_UPDATE_INTERVAL (1000/15)

PeriodTimer ui_timer (lcd_update,UI_UPDATE_INTERVAL);

PeriodTimer extruder_temp_timer (manage_heater,EXTRUDER_PID_CHECK_INTERVAL);
PeriodTimer temp_log_timer (updateTempHistory,1111);
PeriodTimer inactivity_timer (manage_inactivity,1000);
PeriodTimer bed_light_timer (manage_Bed_Lights,100);
PeriodTimer stats_timer (lifetime_stats_tick,60000);
PeriodTimer ambient_timer (updateAmbientSensor,15200);
PeriodTimer aux_temp_timer (updateAuxTempSensors,5000);
PeriodTimer buttons (lcd_lib_buttons_update_interrupt,10);

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#if !TEMP_IRQ
void readTemps();
#define TEMP_READING_INTERVAL 24

PeriodTimer temp_readings (readTemps,TEMP_READING_INTERVAL);
#endif 


#if defined(HEAD_FAN_PIN) && HEAD_FAN_PIN > -1
PeriodTimer head_cooling_fan_timer (checkExtruderAutoFans,FAN_CHECK_INTERVAL/5);
#endif

#ifndef PIDTEMPBED
PeriodTimer bed_temp_timer (manage_bed_temp_control,BED_CHECK_INTERVAL);
#else
PeriodTimer bed_temp_timer (manage_bed_temp_control,BED_PID_CHECK_INTERVAL);
#endif

#ifdef REPORT_TEMPS
PeriodTimer temp_logging_timer (report_temps,2000);
#endif
#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
PeriodTimer mobo_cooling_fan_timer (controllerFan,FAN_CHECK_INTERVAL);
#if defined(FAN_PIN)
#if CONTROLLERFAN_PIN == FAN_PIN
#error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
#endif
#endif


#if LOG_MOTION

extern unsigned short current_speed;
extern unsigned long block_start;

extern short  curve_index ;
extern unsigned  short cur_lin_acc ;
extern unsigned  short cur_lin_acc_raw ;
extern volatile  unsigned long steps_completed; // The number of step events executed in the current block
void log_stepper2()
	{
	if (current_speed==0) return;
	unsigned short cs = current_speed;
	current_speed =0 ;
	SERIAL_ECHO ((micros() - block_start)>>7);
	SERIAL_ECHOPGM("\t");
	SERIAL_ECHO ((unsigned short) cs);
	SERIAL_ECHOPGM("\t");
	SERIAL_ECHO ((unsigned short) curve_index);
	SERIAL_ECHOPGM("\t");
	SERIAL_ECHO ((unsigned short) cur_lin_acc);
	SERIAL_ECHOPGM("\t");
	SERIAL_ECHO ((short) cur_lin_acc_raw);

	SERIAL_ECHOPGM("\t");
	SERIAL_ECHO ((unsigned long) steps_completed);


	SERIAL_ECHOLNPGM(" ");


	}

PeriodTimer motion_logger (log_stepper2,1);

//-----------------------------------------------------------------------------------------------------------------
void log_stepper()
	{
	log_stepper2();
	//	motion_logger.tick();
	}

#endif

//-----------------------------------------------------------------------------------------------------------------
void runTasks(bool with_command_processing,bool with_ui)
	{

	//	updateI2C();
	if (with_command_processing)
		{
		manageBuffer();
#if LOG_MOTION
		log_stepper();
#endif
		checkHitEndstops();
		}

	buttons.tick();
	if (high_speed) 
		{ 
		//	readTemps();		// do the temp management, but not as an IRQ while we are moving at high speed.
		return;
		}

	if (with_ui) 
		ui_timer.tick();
	// 
	// 	if ((i2Ctimeout))
	// 		{
	// 		SERIAL_ECHO_START;
	// 		SERIAL_ECHOLNPGM("I2C TIMEOUT");
	// 		}

#if !TEMP_IRQ
	temp_readings.tick();
#endif 



#if LOG_MOTION
	log_stepper();
#endif
	inactivity_timer.tick();
#if LOG_MOTION
	log_stepper();
#endif
	bed_temp_timer.tick();
#if LOG_MOTION
	log_stepper();
#endif
	extruder_temp_timer.tick();
#if defined(HEAD_FAN_PIN) && HEAD_FAN_PIN > -1
	head_cooling_fan_timer.tick();
#endif
#if LOG_MOTION
	log_stepper();
#endif

	stats_timer.tick();
#if LOG_MOTION
	log_stepper();
#endif
	bed_light_timer.tick();
#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
	mobo_cooling_fan_timer.tick();
#endif

#ifdef REPORT_TEMPS
	temp_logging_timer.tick();
#endif
	temp_log_timer.tick();
	ambient_timer.tick();
	aux_temp_timer.tick();

#if LOG_MOTION
	log_stepper();
#endif
	}

//-----------------------------------------------------------------------------------------------------------------
void loop()
	{
#if LOG_MOTION
	log_stepper();
#endif
	if(commands_queued() < (BUFSIZE-1))
		get_command();
#ifdef SDSUPPORT
	card.checkautostart(false);
#endif
	runTasks(true,true);


	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void FlushSerialRequestResend()
	{
	//char cmdbuffer[bufindr][100]="Resend:";
	MYSERIAL.flush();
	SERIAL_PROTOCOLPGM(MSG_RESEND);
	SERIAL_PROTOCOLLN(gcode_LastN + 1);
	ClearToSend();
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void ClearToSend()
	{
	previous_millis_cmd = millis();
#ifdef SDSUPPORT
	if(is_command_from_sd(bufindr))
		return;
#endif //SDSUPPORT
	SERIAL_PROTOCOLLNPGM(MSG_OK);
	}



//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#ifdef DELTA
void calculate_delta(float cartesian[3])
	{
	delta[X_AXIS] = sqrt(sq(DELTA_DIAGONAL_ROD)
		- sq(DELTA_TOWER1_X-cartesian[X_AXIS])
		- sq(DELTA_TOWER1_Y-cartesian[Y_AXIS])
		) + cartesian[Z_AXIS];
	delta[Y_AXIS] = sqrt(sq(DELTA_DIAGONAL_ROD)
		- sq(DELTA_TOWER2_X-cartesian[X_AXIS])
		- sq(DELTA_TOWER2_Y-cartesian[Y_AXIS])
		) + cartesian[Z_AXIS];
	delta[Z_AXIS] = sqrt(sq(DELTA_DIAGONAL_ROD)
		- sq(DELTA_TOWER3_X-cartesian[X_AXIS])
		- sq(DELTA_TOWER3_Y-cartesian[Y_AXIS])
		) + cartesian[Z_AXIS];
	/*
	SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
	SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
	SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

	SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
	SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
	SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
	*/
	}
#endif

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// manage the motherboard / stepper cooling fans
void controllerFan()
	{
	static byte previous_mobo_temp = 0;
	static byte previous_step_temp = 0;

#if defined(MOBO_TEMP_SENSOR_PIN) && MOBO_TEMP_SENSOR_PIN>-1 
	if (last_mobo_temp_reading > CASE_FAN_ON_THRESHOLD && previous_mobo_temp >CASE_FAN_ON_THRESHOLD )
		{
		// allows digital or PWM fan output to be used (see M42 handling)
		digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
		analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
//		lastMotor = millis(); 
		SERIAL_ECHOPAIR ("MOBO TEMP=",last_mobo_temp_reading);
		SERIAL_ECHOLNPGM("'C - FAN ON, TEMPERATURE");
		previous_mobo_temp = last_mobo_temp_reading;
		return;
		}
	previous_mobo_temp = last_mobo_temp_reading;

#endif 


#if defined(STEPPER_TEMP_SENSOR_PIN) && STEPPER_TEMP_SENSOR_PIN>-1 
	if (last_step_temp > CASE_FAN_ON_THRESHOLD && previous_step_temp> CASE_FAN_ON_THRESHOLD)
		{
		// allows digital or PWM fan output to be used (see M42 handling)
		digitalWrite(STEPPER_FAN_PIN, CONTROLLERFAN_SPEED);
		analogWrite(STEPPER_FAN_PIN, CONTROLLERFAN_SPEED);
//		lastMotor = millis(); 
		SERIAL_ECHOPAIR ("STEPPER TEMP=",last_step_temp);
		SERIAL_ECHOLNPGM("'C - STEPPER FAN ON, TEMPERATURE");
		previous_step_temp = last_step_temp;
		return;
		}
	previous_step_temp = last_step_temp;
#endif 

	if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN)
#if EXTRUDERS > 2
		|| !READ(E2_ENABLE_PIN)
#endif
#if EXTRUDER > 1
		|| !READ(E1_ENABLE_PIN)
#endif
		|| !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
		{
		lastMotor = millis(); //... set time to NOW so the fan will turn on
		}
	if ((millis() - lastMotor) /1000UL >= (CONTROLLERFAN_SECS) || lastMotor == 0UL) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
		{
		digitalWrite(CONTROLLERFAN_PIN, 0);
		analogWrite(CONTROLLERFAN_PIN, 0);
		digitalWrite(STEPPER_FAN_PIN, 0);
		analogWrite(STEPPER_FAN_PIN, 0);
		}
	else 
		{
/*
		if ((millis() - lastMotor) /1000UL >= (CONTROLLERFAN_SECS/2) ) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
			{
			analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED/2);
			analogWrite(STEPPER_FAN_PIN, CONTROLLERFAN_SPEED/2);
			}
		else
*/
			{
			// allows digital or PWM fan output to be used (see M42 handling)
			digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
			analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
			digitalWrite(STEPPER_FAN_PIN, CONTROLLERFAN_SPEED);
			analogWrite(STEPPER_FAN_PIN, CONTROLLERFAN_SPEED);
			SERIAL_ECHOPAIR ("x=",!READ(X_ENABLE_PIN));
			SERIAL_ECHOPAIR (" \ty=",!READ(Y_ENABLE_PIN));
			SERIAL_ECHOPAIR (" \tz=",!READ(Z_ENABLE_PIN));
			SERIAL_ECHOPAIR (" \te0=",!READ(E0_ENABLE_PIN));
#if EXTRUDER > 1
			SERIAL_ECHOPAIR (" \te1=",!READ(E1_ENABLE_PIN));
#endif
			SERIAL_ECHOLNPGM("\t MOBO FAN ON, LAST MOTOR MOVE");
			}
		} 

	return;


	// debug spattering 


#define ACS712RATIO (25.0/512.0)
	// 	analogRead(CURRENT_SENSOR_PIN);
	// 	float power_usage = ACS712RATIO*(analogRead(CURRENT_SENSOR_PIN)-512);

	if (digitalRead((HEAD_FAN_PIN)))
		{ SERIAL_ECHOPGM ("FAN ON ");} 
	else 
		{ SERIAL_ECHOPGM ("FAN OFF"); } 

	// 	SERIAL_ECHOPAIR(" \tCurrent=",power_usage);
	// 	SERIAL_ECHOPGM(" Amps   ");
	analogRead(EXT_VOLTAGE_REF_PIN);
	float v_ref = analogRead(EXT_VOLTAGE_REF_PIN);
	SERIAL_ECHOPAIR("\tVRef=",v_ref);
	SERIAL_ECHOPGM(" \t");


	/*
	DHT11_disable = false;
	updateAmbientSensor();
	SERIAL_ECHOPAIR("temp= ", (unsigned long) last_temp);
	SERIAL_ECHOPGM(" 'C   ");
	SERIAL_ECHOPAIR("RH=", (unsigned long) last_humid);
	SERIAL_ECHOLNPGM("%");
	if (last_temp==0) DHT11_disable = true;
	*/

	SERIAL_ECHOLNPGM(" ");

	}
#endif


//-----------------------------------------------------------------------------------------------------------------
// handles the LED strip lighting ramping up and fading down
void manage_Bed_Lights()
	{
	static byte current_led_brightness  = 0;
	bed_light_timer.setPeriod(100);

	// fade out if idle time has expired.
	if (LED_DIM_TIME>0 && (millis() - last_user_interaction > LED_DIM_TIME*MILLISECONDS_PER_MINUTE ))
		{
		if (current_led_brightness > DIM_LEVEL) current_led_brightness--;
		else 
			// we're at dimmed level, set the timer to 1Hz
			bed_light_timer.setPeriod(1000);
		}

	// else ramp up again
	else
		{
		if (current_led_brightness <  led_brightness_level-3) current_led_brightness+=3;
		else 
			{		// we're at nominal brightness, set the timer to 1Hz 
			current_led_brightness = led_brightness_level;
			bed_light_timer.setPeriod(1000);
			} 
		}

	analogWrite(LED_PIN, ((unsigned int) current_led_brightness*255) / 100);
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void manage_inactivity()
	{

	if(max_inactive_time)
		if( (millis() - previous_millis_cmd) >  max_inactive_time )
			kill();
	if(stepper_inactive_time)
		{
		if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
			{
			if(blocks_queued() == false)
				{
				if(DISABLE_X) disable_x();
				if(DISABLE_Y) disable_y();
				if(DISABLE_Z) disable_z();
				if(DISABLE_E)
					{
					disable_e0();
					disable_e1();
					disable_e2();
					}
				// 		if (MOTHERBOARD_FAN>-1)
				// 			WRITE (MOTHERBOARD_FAN,0);

				}
			}
		}
#if defined(KILL_PIN) && KILL_PIN > -1
	if( 0 == READ(KILL_PIN) )
		kill();
#endif
#if defined(SAFETY_TRIGGERED_PIN) && SAFETY_TRIGGERED_PIN > -1
	if (READ(SAFETY_TRIGGERED_PIN))
		{
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM("Safety Stop");
		LCD_ALERTMESSAGEPGM("Safety Stop");
		LED_GLOW_ERROR();
		forceMessage();
		Stop(STOP_REASON_SAFETY_TRIGGER);
		}
#endif

#ifdef EXTRUDER_RUNOUT_PREVENT
	if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 )
		if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
			{
			bool oldstatus=READ(E0_ENABLE_PIN);
			enable_e0();
			float oldepos=current_position[E_AXIS];
			float oldedes=destination[E_AXIS];
			plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
				current_position[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS],
				EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
			current_position[E_AXIS]=oldepos;
			destination[E_AXIS]=oldedes;
			plan_set_e_position(oldepos);
			previous_millis_cmd=millis();
			st_synchronize();
			WRITE(E0_ENABLE_PIN,oldstatus);
			}
#endif
		check_axes_activity();
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void kill()
	{
	cli(); // Stop interrupts
	disable_heater();

	disable_x();
	disable_y();
	disable_z();
	disable_e0();
	disable_e1();
	disable_e2();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
	pinMode(PS_ON_PIN,INPUT);
#endif
	SERIAL_ERROR_START;
	SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
	LCD_ALERTMESSAGEPGM(MSG_KILLED);
	LED_GLOW_ERROR();
	forceMessage();
	suicide();
	while(1) { /* Intentionally left empty */ } // Wait for reset
	}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Stop(uint8_t reasonNr)
	{
	disable_heater();
	if(Stopped == false)
		{
		Stopped = reasonNr;
		Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
		//   SERIAL_ERRORLN(TOPPED);
		LCD_MESSAGEPGM(MSG_STOPPED);
		forceMessage();
		led_glow = 31;
		delay(500);
		}
	LED_GLOW_ERROR();
	}


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
	{
	val &= 0x07;
	switch(digitalPinToTimer(pin))
		{

#if defined(TCCR0A)
		case TIMER0A:
		case TIMER0B:
			//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
			//         TCCR0B |= val;
			break;
#endif

#if defined(TCCR1A)
		case TIMER1A:
		case TIMER1B:
			//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
			//         TCCR1B |= val;
			break;
#endif

#if defined(TCCR2)
		case TIMER2:
		case TIMER2:
			TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
			TCCR2 |= val;
			break;
#endif

#if defined(TCCR2A)
		case TIMER2A:
		case TIMER2B:
			TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
			TCCR2B |= val;
			break;
#endif

#if defined(TCCR3A)
		case TIMER3A:
		case TIMER3B:
		case TIMER3C:
			TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
			TCCR3B |= val;
			break;
#endif

#if defined(TCCR4A)
		case TIMER4A:
		case TIMER4B:
		case TIMER4C:
			TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
			TCCR4B |= val;
			break;
#endif

#if defined(TCCR5A)
		case TIMER5A:
		case TIMER5B:
		case TIMER5C:
			TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
			TCCR5B |= val;
			break;
#endif

		}
	}
#endif //FAST_PWM_FAN



//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

