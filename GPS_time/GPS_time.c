/*
 * GPS_time.c
 *
 * Created: 3/1/2016 1:47:59 PM
 *  Author: Rick Shory
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

// define what pins the LEDs are connected to.
// PA3 is pin 10 in the PDIP chip package
#define LED PA3

// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

int main(void)
{
	// initialize the direction of PORTD #6 to be an output
	set_output(DDRA, LED);  

    while(1)
    {
		// turn on the LED for 1000ms
		output_high(PORTA, LED);
		_delay_ms(1000);
		// now turn off the LED for another 200ms
		output_low(PORTA, LED);
		_delay_ms(1000);
		// now start over
    }
}