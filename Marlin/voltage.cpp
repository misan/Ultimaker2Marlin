#include "Marlin.h"
#include "voltage.h"



float VCC=5.0;	// start by assuming 5.00 volts, until we read otherwise

// reads the voltage divider connected to an analog pin, measuing the PSU voltage (24 v nominal)
float readVoltage ()
{
#if MAIN_VOLTAGE_MEASURE_PIN>-1
    float v = analogRead(MAIN_VOLTAGE_MEASURE_PIN);
    v = v  / 1024;
    v *= VCC;
    v /=EXT_VOLT_DIVIDER_RATIO;
    return v;
#endif
    return 0.0;
}



float readAVR_VCC()		// we have an external Zener reference of 4.2volts connected to an analog pin
{
#if defined(EXTERNAL_VOLTAGE_REFERENCE) && defined(EXTERNAL_VOLTAGE_REFERENCE_VALUE) && EXTERNAL_VOLTAGE_REFERENCE_PIN>-1
    float v_ref = analogRead(EXT_VOLTAGE_REF_PIN);
    v_ref /= 1024.0;		// normalize
    float result  = EXTERNAL_VOLTAGE_REFERENCE_VALUE / v_ref;

    if (result < MIN_VOLTAGE ||result > MAX_VOLTAGE ) return VCC;
    VCC = result ;
    return VCC;
#endif
    return read_INTERNAL_AVR_VCC();
}

// DONT call this when printing -- it messes with the ADC and delays for *at least* 10ms
float read_INTERNAL_AVR_VCC(long voltage_reference)
{

    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    ADCSRB &= ~_BV(MUX5); // Without this the function always returns -1 on the ATmega2560

#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
    static const float K1x = 1000.0f;


    // trying the 2V56 refernce -- maybe it's more reliable
    // 	  analogReference(INTERNAL2V56);
    // 	  voltage_reference = (2560UL )<< 10;

    ADCSRB = 0;

    delay(10); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    analogReference(DEFAULT);

    long result = (high<<8) | low;
    result = voltage_reference / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

    // clip the return value to a sane range -- we sometimes get bogus results
    if (result / K1x < MIN_VOLTAGE ||result / K1x > MAX_VOLTAGE ) return VCC;
    VCC = result/K1x;
    return VCC; // Vcc in millivolts
}



extern "C" {
    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void *__brkval;

    int freeMemory()
    {
        int free_memory;

        if((int)__brkval == 0)
            free_memory = ((int)&free_memory) - ((int)&__bss_end);
        else
            free_memory = ((int)&free_memory) - ((int)__brkval);

        return free_memory;
    }
}