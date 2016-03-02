GPS_time
======
**GPS_time** is a module of Greenlogger project.
 GPS_time runs in a separate microcontroller.
 It manages power, startup, and shutdown of the GPS hardware
as well as keeping itself in deep sleep most of the time.
 It turns on the GPS, waits for a stable time signal, then
returns that as a set-time command to the main microcontroller.
 In this way, keeps the instrument's real time clock current
avoiding drift over months/years of unattended operation, or
recovery from solar power depletion during polar night.

#### Works on
* AVR microcontroller
initial development on ATmega1284P
will probably migrate to smaller chip to save cost and space,
a device that has only what's needed:
2 UARTs
I2C
about half a dozen general i/o lines


## Contact
#### Developer/Company
* Project of: http://www.rickshory.com/
* e-mail: rickshory@gmail.com
