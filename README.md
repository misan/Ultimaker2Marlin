==========================
Norpchen's Changes to Ultimaker2 Marlin 
==========================

These are changes I've made for my own use and benefit,  Use at your own risk.  If your Ultimaker2 explodes, bursts into flame, tries to kill you, etc. don't come crying to me or to Ultimaker about it.

Change log: 
---------------------
* May 28 2015* 
* TONS of stuff, including: 
* show date, time and layer count of files in detail view
* cleanup of menus, lcd cache, and marlin_main code
* split gcode, command processing, and menu help functions to separate files
* support DHT ambient sensor
* support for external voltage reference to check power supply 24V and VCC 5V levels
* supoport for head fan and motherboard fan
* moved bed height adjustments to system menu
* idle screen shows last print info, material settings, system info, system history metrics, etc.
* graphs for head and bed temperature when printing, heating or adjusting
* added a quality Q value to motion / extrusion commands allowing the planner to control movement based on quality.  If Q code is not included, there is no effect on output
* Added more options to print tune menu like nudge build platform height, rehome after pause, extrude during pause, adjust z lift, quick disable retraction or z lift, center head during pause, etc.
* Added more options to system menu, like relax motors, center head, test cooling / head / motherboard fans, adjust X and Y max limits dynamically, adjust idle timeout (dims LEDs, shows screensaver), toggle long menu wrapping, etc
* cooling fan speed override in print tune menu will ignore gcode fan commands.  can be turned off to release fan control back to gcode
* extruder overheat error resistant to glitches -- requires multiple readings before error will trip.
* printing screen shows z-height, heater power level, temperature graph, fan speed and override, buffer depth, extrusion rate in mm^3, head speed in mm/s, bed temp, head temp, currently printing filename, length of material printed and remaining, time so far and estimnated remaining time, flow and speed settings, progress bar, messages from GCode, the Tune/Abort menu -- ALL ON TTHE SAME SCREEN!  WHEW!
* functionality to spew raw temperature readings to the serial port for calibration of temp sensor
* a cute screensaver
* filenames longer than 20 chars will show a ... and the last 4 characters at the end
* pulled in various fixes and changes as appropriate from main Ultimaker line (done manually -- I've gone way too far to do an automatic merge anymore)
* memory usage report on start to serial port
* z-lift setting saved to eeprom
* there's more I can't remember at the moment...
*Jun 13,2014:*
* Added more material presets (nylon, PET, flex PLA) 
* Made it easier to add presets in the source code
* Longer material names
* Reset materials option in the advanced menu

*Jun 11,2014:*
* brought in Illuminarti's startup / priming fixes

*June 5, 2014:*
* Brought in sync with Ultimaker's master changes (lifetime stats, z axis homing) 

*June 3, 2014* 

*Display changes:*
* Head temperature, fan speed, XY movement speed, extrusion volume, and movement planner buffer depth shown while printing.  These are the most relevant values to keep an eye on during printing when trying to solve underextrusion, flow, and other quality issues.
* Time display now show one decimal point of hours when time is > 1 and < 10 hrs (ie: 2.3 hours vs 2 hours)
* Bed & head temperatures shown while preheating.
* Added hysteresis to print complete / cooldown display to prevent oscillations
* Added M117 support for g-code messages (shown on printing and USB comm screens) 
* Added symbols in the font for deg C, ^2 and ^3 and updated references in strings
* Show total print time & name of file at end

*Controls / UI changes:*
* Audible feedback (clicks) when moving encoder and navigating menu items
* Various beeps on startup, error, completion, etc.
* Faster SD card directory scrolling / listing
* Tired of having to turn the knob 200+ clicks when setting the temperature?  No more. Acceleration on encoder wheel when changing values (temperature, brightness, etc) with pitch changing beeps. <-- Be careful changing things like print speed / flow that are "live" until you are used to the acceleration!
* Menus no longer wrap around and single step scroll (but are faster!) -- It's a matter of personal perference, but I find this less confusing overall and easier to navigate.  
* LEDs will dim after thirty minutes of no user input (control knob) -- time and dim level set in configuration.h -- just rotate the control knob one click to wake up.  (I'll need to make that timeout an adjustable preference eventually).  The LEDs don't use a lot of power, but this is handy for overnight unattended printing. 
* LED RGB ring around encoder is more informative about state:
	* orange glow when pre-heating
	* blue glow when cooling
	* green glow when all done
	* flashes red in error (it used to cycle through purple and green) 
	* turns pink when the print buffer is running low while printing.  This is not an error but a warning that the planner / USB host is not able to process / send movement commands fast enough to keep the movement buffer filled. A low buffer does not cause a problem, but an empty one will cause the head to stop and wait for new commands, which can cause oozing and blobs.  Often many tiny moves can cause a low buffer situation, as the movement happens very quickly and the planner cannot keep up.  Marlin will compensate by automatically slowing down movement commands when the buffer is less than half full.  It is normal for the buffer to drain during "head cool lift", if you pause the USB host, and at the end of the print.
	* turns red when the print buffer is empty (stall) while printing
	* turns blue during retraction while printing

*Added GCode support:*
* support for M420 code for RGB led on controller wheel (r, e, b = 0-255 (yes, that's E and not G for the green) -- if set by g-code, normal opertating status colors will not be shown until 0,0,0 color is set by g-code (which can also be done with an M420 with no parameters)
* support for M421 code for LED lighting brightness (s = 0-255 brightness)
* support for M300 code to make a "beep"  (s = freq, p = duration) 
* Serial feedback when it encounters an unsupported command
* Previously mentioned M117 command for text messages on display.
* Timed wait / dwell commands show countdown (or M117 message)

*Other:*
* Fix for minimum fan speed "kickstart" logic not working when set too low 
* Bigger menu cache and movement planner buffer for Mega2560 based boards (UM2) -- smoother menus and smoother printing.  Reduced memory allocated to USB serial buffers, so I don't recommend this firmware for USB host printing.
* Moved lifting the bed and nozzle prime values at start of print to configuration.h file (defaults to no bed lift and a slower extrusion compared than the official version -- I was having trouble with it grinding the filament at the start of the print) 
* I set the default baud rate to 115200 because some linux configs have issues with 250000 (rasp pi, etc). Some debug serial consoles don't habdle 250000 either.  But again, this is not ideal for USB host printing.


==========================



==========================
Marlin 3D Printer Firmware
==========================

[![Flattr this git repo](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=ErikZalm&url=https://github.com/ErikZalm/Marlin&title=Marlin&language=&tags=github&category=software)

Quick Information
===================
This RepRap firmware is a mashup between <a href="https://github.com/kliment/Sprinter">Sprinter</a>, <a href="https://github.com/simen/grbl/tree">grbl</a> and many original parts.

Derived from Sprinter and Grbl by Erik van der Zalm.
Sprinters lead developers are Kliment and caru.
Grbls lead developer is Simen Svale Skogsrud. Sonney Jeon (Chamnit) improved some parts of grbl
A fork by bkubicek for the Ultimaker was merged, and further development was aided by him.
Some features have been added by:
Lampmaker, Bradley Feldman, and others...


Features:

*   Interrupt based movement with real linear acceleration
*   High steprate
*   Look ahead (Keep the speed high when possible. High cornering speed)
*   Interrupt based temperature protection
*   preliminary support for Matthew Roberts advance algorithm
    For more info see: http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
*   Full endstop support
*   SD Card support
*   SD Card folders (works in pronterface)
*   SD Card autostart support
*   LCD support (ideally 20x4)
*   LCD menu system for autonomous SD card printing, controlled by an click-encoder.
*   EEPROM storage of e.g. max-velocity, max-acceleration, and similar variables
*   many small but handy things originating from bkubicek's fork.
*   Arc support
*   Temperature oversampling
*   Dynamic Temperature setpointing aka "AutoTemp"
*   Support for QTMarlin, a very beta GUI for PID-tuning and velocity-acceleration testing. https://github.com/bkubicek/QTMarlin
*   Endstop trigger reporting to the host software.
*   Updated sdcardlib
*   Heater power reporting. Useful for PID monitoring.
*   PID tuning
*   CoreXY kinematics (www.corexy.com/theory.html)
*   Configurable serial port to support connection of wireless adaptors.
*   Automatic operation of extruder/cold-end cooling fans based on nozzle temperature
*   RC Servo Support, specify angle or duration for continuous rotation servos.

The default baudrate is 250000. This baudrate has less jitter and hence errors than the usual 115200 baud, but is less supported by drivers and host-environments.


Differences and additions to the already good Sprinter firmware:
================================================================

*Look-ahead:*

Marlin has look-ahead. While sprinter has to break and re-accelerate at each corner,
lookahead will only decelerate and accelerate to a velocity,
so that the change in vectorial velocity magnitude is less than the xy_jerk_velocity.
This is only possible, if some future moves are already processed, hence the name.
It leads to less over-deposition at corners, especially at flat angles.

*Arc support:*

Slic3r can find curves that, although broken into segments, were ment to describe an arc.
Marlin is able to print those arcs. The advantage is the firmware can choose the resolution,
and can perform the arc with nearly constant velocity, resulting in a nice finish.
Also, less serial communication is needed.

*Temperature Oversampling:*

To reduce noise and make the PID-differential term more useful, 16 ADC conversion results are averaged.

*AutoTemp:*

If your gcode contains a wide spread of extruder velocities, or you realtime change the building speed, the temperature should be changed accordingly.
Usually, higher speed requires higher temperature.
This can now be performed by the AutoTemp function
By calling M109 S<mintemp> T<maxtemp> F<factor> you enter the autotemp mode.

You can leave it by calling M109 without any F.
If active, the maximal extruder stepper rate of all buffered moves will be calculated, and named "maxerate" [steps/sec].
The wanted temperature then will be set to t=tempmin+factor*maxerate, while being limited between tempmin and tempmax.
If the target temperature is set manually or by gcode to a value less then tempmin, it will be kept without change.
Ideally, your gcode can be completely free of temperature controls, apart from a M109 S T F in the start.gcode, and a M109 S0 in the end.gcode.

*EEPROM:*

If you know your PID values, the acceleration and max-velocities of your unique machine, you can set them, and finally store them in the EEPROM.
After each reboot, it will magically load them from EEPROM, independent what your Configuration.h says.

*LCD Menu:*

If your hardware supports it, you can build yourself a LCD-CardReader+Click+encoder combination. It will enable you to realtime tune temperatures,
accelerations, velocities, flow rates, select and print files from the SD card, preheat, disable the steppers, and do other fancy stuff.
One working hardware is documented here: http://www.thingiverse.com/thing:12663
Also, with just a 20x4 or 16x2 display, useful data is shown.

*SD card folders:*

If you have an SD card reader attached to your controller, also folders work now. Listing the files in pronterface will show "/path/subpath/file.g".
You can write to file in a subfolder by specifying a similar text using small letters in the path.
Also, backup copies of various operating systems are hidden, as well as files not ending with ".g".

*SD card folders:*

If you place a file auto[0-9].g into the root of the sd card, it will be automatically executed if you boot the printer. The same file will be executed by selecting "Autostart" from the menu.
First *0 will be performed, than *1 and so on. That way, you can heat up or even print automatically without user interaction.

*Endstop trigger reporting:*

If an endstop is hit while moving towards the endstop, the location at which the firmware thinks that the endstop was triggered is outputed on the serial port.
This is useful, because the user gets a warning message.
However, also tools like QTMarlin can use this for finding acceptable combinations of velocity+acceleration.

*Coding paradigm:*

Not relevant from a user side, but Marlin was split into thematic junks, and has tried to partially enforced private variables.
This is intended to make it clearer, what interacts which what, and leads to a higher level of modularization.
We think that this is a useful prestep for porting this firmware to e.g. an ARM platform in the future.
A lot of RAM (with enabled LCD ~2200 bytes) was saved by storing char []="some message" in Program memory.
In the serial communication, a #define based level of abstraction was enforced, so that it is clear that
some transfer is information (usually beginning with "echo:"), an error "error:", or just normal protocol,
necessary for backwards compatibility.

*Interrupt based temperature measurements:*

An interrupt is used to manage ADC conversions, and enforce checking for critical temperatures.
This leads to less blocking in the heater management routine.

Implemented G Codes:
====================

*  G0  -> G1
*  G1  - Coordinated Movement X Y Z E
*  G2  - CW ARC
*  G3  - CCW ARC
*  G4  - Dwell S<seconds> or P<milliseconds>
*  G10 - retract filament according to settings of M207
*  G11 - retract recover filament according to settings of M208
*  G28 - Home all Axis
*  G90 - Use Absolute Coordinates
*  G91 - Use Relative Coordinates
*  G92 - Set current position to cordinates given

RepRap M Codes
*  M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
*  M1   - Same as M0
*  M104 - Set extruder target temp
*  M105 - Read current temp
*  M106 - Fan on
*  M107 - Fan off
*  M109 - Wait for extruder current temp to reach target temp.
*  M114 - Display current position

Custom M Codes
*  M17  - Enable/Power all stepper motors
*  M18  - Disable all stepper motors; same as M84
*  M20  - List SD card
*  M21  - Init SD card
*  M22  - Release SD card
*  M23  - Select SD file (M23 filename.g)
*  M24  - Start/resume SD print
*  M25  - Pause SD print
*  M26  - Set SD position in bytes (M26 S12345)
*  M27  - Report SD print status
*  M28  - Start SD write (M28 filename.g)
*  M29  - Stop SD write
*  M30  - Delete file from SD (M30 filename.g)
*  M31  - Output time since last M109 or SD card start to serial
*  M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
*  M80  - Turn on Power Supply
*  M81  - Turn off Power Supply
*  M82  - Set E codes absolute (default)
*  M83  - Set E codes relative while in Absolute Coordinates (G90) mode
*  M84  - Disable steppers until next move, or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
*  M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
*  M92  - Set axis_steps_per_unit - same syntax as G92
*  M114 - Output current position to serial port
*  M115 - Capabilities string
*  M117 - display message
*  M119 - Output Endstop status to serial port
*  M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
*  M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
*  M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
*  M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
*  M140 - Set bed target temp
*  M190 - Wait for bed current temp to reach target temp.
*  M200 - Set filament diameter
*  M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
*  M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
*  M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
*  M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
*  M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
*  M206 - set additional homeing offset
*  M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
*  M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
*  M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
*  M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
*  M220 S<factor in percent>- set speed factor override percentage
*  M221 S<factor in percent>- set extrude factor override percentage
*  M240 - Trigger a camera to take a photograph
*  M280 - Position an RC Servo P<index> S<angle/microseconds>, ommit S to report back current angle
*  M300 - Play beepsound S<frequency Hz> P<duration ms>
*  M301 - Set PID parameters P I and D
*  M302 - Allow cold extrudes
*  M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
*  M304 - Set bed PID parameters P I and D
*  M400 - Finish all moves
*  M500 - stores paramters in EEPROM
*  M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
*  M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
*  M503 - print the current settings (from memory not from eeprom)
*  M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
*  M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
*  M907 - Set digital trimpot motor current using axis codes.
*  M908 - Control digital trimpot directly.
*  M350 - Set microstepping mode.
*  M351 - Toggle MS1 MS2 pins directly.
*  M928 - Start SD logging (M928 filename.g) - ended by M29
*  M999 - Restart after being stopped by error


Configuring and compilation:
============================

Install the arduino software IDE/toolset v23 (Some configurations also work with 1.x.x)
   http://www.arduino.cc/en/Main/Software

For gen6/gen7 and sanguinololu the Sanguino directory in the Marlin dir needs to be copied to the arduino environment.
  copy ArduinoAddons\Arduino_x.x.x\sanguino <arduino home>\hardware\Sanguino

Copy the Marlin firmware
   https://github.com/ErikZalm/Marlin/tree/Marlin_v1
   (Use the download button)

Start the arduino IDE.
Select Tools -> Board -> Arduino Mega 2560    or your microcontroller
Select the correct serial port in Tools ->Serial Port
Open Marlin.pde

Click the Verify/Compile button

Click the Upload button
If all goes well the firmware is uploading

That's ok.  Enjoy Silky Smooth Printing.




