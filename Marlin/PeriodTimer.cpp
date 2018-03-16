#include "PeriodTimer.h"



//-----------------------------------------------------------------------------------------------------------------
PeriodTimer::PeriodTimer( PeriodicTimerFunction pf, unsigned long interval )
{
    last_action=0;
    period = interval;
    paused = false;
    
    p_function = pf;
	rollover = false;
	last_action=random(interval);
}

//-----------------------------------------------------------------------------------------------------------------
void PeriodTimer::reset()
{
    last_action = millis();
}

//-----------------------------------------------------------------------------------------------------------------
bool PeriodTimer::tick()
{
    if (millis() - last_action >= period)
        {
            if (rollover) last_action += period;
            else last_action = millis();

            if (p_function && !paused)
                {
                    p_function();
                    return true;
                }
        }
    return false;

}
