#pragma once

#define MIN_VOLTAGE 2.5
#define MAX_VOLTAGE 6.5

#define EXTERNAL_VOLTAGE_REFERENCE 
#define EXTERNAL_VOLTAGE_REFERENCE_VALUE 4.2


#define EXT_VOLT_DIVIDER_RATIO 0.045 // divider network is 100K and 4.7K, so that's a 0.045 divisor


extern float VCC;	// start by assuming 5.00 volts, until we read otherwise


extern "C"{ int freeMemory() ;} 

// DONT call this when printing -- it messes with the ADC and delays for *at least* 10ms
float read_INTERNAL_AVR_VCC(long voltage_reference = /*1125300L*/  1186680L);

// read a voltage divider to read PSU voltage for motors and heaters1111111111111
float readVoltage ();


float readAVR_VCC();		// we have an external Zener reference of 4.2volts connected to an analog pin
 