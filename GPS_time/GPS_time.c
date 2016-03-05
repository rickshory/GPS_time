/*
 * GPS_time.c
 *
 * Created: 3/1/2016 1:47:59 PM
 *  Author: Rick Shory
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

//pins by package
//    PDIP QFN
// PA0 13   5
// PA1 12   4
// PA2 11   3
// PA3 10   2
// PA4  9   1
// PA5  8  20
// PA6  7  16
// PA7  6  15
// PB0  2  11
// PB1  3  12
// PB2  5  14
// PB3  4  13

#define LED PA3

#define INIT_TIME PB7
#define GPS_PWR_ENAB PA7
#define PULSE_GPS PA1

// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

int main(void)
{
	// assure GPS power enable starts low
	output_low(PORTA, GPS_PWR_ENAB);
	set_output(DDRA, GPS_PWR_ENAB);
	// assure pulse signal to GPS starts low
	output_low(PORTA, PULSE_GPS);
	set_output(DDRA, PULSE_GPS);
	// wait 1 sec in case anything needs to stabilize
	_delay_ms(1000);
	// turn GPS power on
	output_high(PORTA, GPS_PWR_ENAB);
	// wait 1 sec to assure stable
	_delay_ms(1000);
	// send 200ms pulse to GPS, to turn on
	output_high(PORTA, PULSE_GPS);
	_delay_ms(200);
	output_low(PORTA, PULSE_GPS);
	
	// for testing, just blink the LED
	set_output(DDRA, LED);	
    while(1)
    {
		output_high(PORTA, LED);
		_delay_ms(500);
		output_low(PORTA, LED);
		_delay_ms(500);
    }
}