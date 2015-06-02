#pragma once

#include "Marlin.h"



typedef void (*PeriodicTimerFunction)();

// Periodic timer
// pass in a function  Specify the interval in milliseconds.
// call tick() as often as you can, it will call the specified function   if the time has come.
// enable rollover if you want it to "catch up" if late (next tick will happen earlier, so that the total ticks 
// will be, on average, the correct number even if some are late). 
// if you do not enable rollover flag, the next tick will occur at exactly the specified milliseconds after,
// regardless of how much time actually transpired previously.
// if paused, it will still tick the timer (assuming you call tick()) such taht when you resume, it will remain 
// in sync with previous calls.  If you use start(), it will clear the timer

// note if you pause, then don't call tick() for a long time, then resume and rollover is enabled, it 
// may end up firing repeatedly until it catches up.  Use start() instead of resume to avoid this (or call tick() ) 
//  however, the user function will not be called more than once per tick()
//
// rollover and pause are disabled by default

class PeriodTimer
	{
		unsigned long last_action;
		unsigned long period;
	private:
		bool paused:1;
		bool rollover:1;
		
		PeriodicTimerFunction p_function;
	public:
		PeriodTimer( PeriodicTimerFunction pf,  unsigned long interval=1000);

		unsigned long getPeriod() const { return period; }
		void setPeriod(unsigned long val) { period = val; }
		bool getRollover() const { return rollover; }
		void setRollover(bool val) { rollover = val; }
		
		void start () { paused= false; reset();}
		void pause()  { paused= true;}
		void togglePause()  { paused= !paused;}
		bool isPaused () { return paused; }
		void resume ()  { paused= false;}
		
		void reset();

		bool tick();

	};

//-----------------------------------------------------------------------------------------------------------------
