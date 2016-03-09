/*
 * GPS_time.c
 *
 * Created: 3/1/2016 1:47:59 PM
 *  Author: Rick Shory
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

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
#define TEST_LED PB0

#define TIME_REQ PB2
#define INIT_TIME PB7
#define GPS_PWR_ENAB PA7
#define PULSE_GPS PA1

// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

static volatile union Prog_status // Program status bit flags
{
    unsigned char status;
    struct
    {
        unsigned char gps_Request_Active:1;
        unsigned char gps_Power_Enabled:1;
        unsigned char gps_Being_Pulsed:1;
        unsigned char listen_To_GPS:1;
        unsigned char flag4:1;
        unsigned char flag5:1;
        unsigned char flag6:1;
        unsigned char flag7:1;
    };
} Prog_status = {0};

int main(void)
{
	// set up test blinker
	set_output(DDRB, TEST_LED);
	output_low(PORTB, TEST_LED);
	
	// assure time-request signal from main starts high
	set_input(DDRB, TIME_REQ); // set as input
	output_high(PORTB, TIME_REQ); // write to the port bit to enable the pull-up resistor
	
	// assure GPS power enable starts low
	output_low(PORTA, GPS_PWR_ENAB);
	set_output(DDRA, GPS_PWR_ENAB);
	// assure pulse signal to GPS starts low
	output_low(PORTA, PULSE_GPS);
	set_output(DDRA, PULSE_GPS);
	// set up the external interrupt INT0
	// clear interrupt flag; probably don't need to do this because flag is cleared on level interrupts
	GIFR |= (1<<INTF0);
	// Configure INT0 to trigger on level low
	MCUCR &= ~((1<<ISC01)|(1<<ISC00));
	// Enable the INT0 interrupt
	GIMSK |= (1<<6);
	// set the global interrupt enable bit.
	sei();
	_delay_ms(1000);
	// for testing, wait here till bit set by interrupt
//	while(Prog_status.gps_Request_Active == 0);
	// continue on from here
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

ISR(INT0_vect)
{
	// not supposed to do this kind of thing inside an ISR, but this
	// is to see if the ISR is ever entered
	output_high(PORTB, TEST_LED);
	_delay_ms(500);
	output_low(PORTB, TEST_LED);
//	_delay_ms(500);
	
	// if the Input Sense is to detect a low level
	if ((MCUCR & ((1<<ISC01)|(1<<ISC00))) == 0)
	{
		// we are waking up from sleep to get a time fix
		Prog_status.gps_Request_Active = 1;
		// (for testing, change this interrupt to next trigger on rising edge.
		// in final version, 'gps_Request_Active' will be reset to 0 by
		// completing the time-set sequence)
		// temporarily disable INT0 to prevent sense-change from triggering an interrupt
		GIMSK &= ~(1<<6);
		// change the sense
		MCUCR |= ((1<<ISC01)|(1<<ISC00));
		// clear the flag bit before enabling INT0 or interrupt will be triggered here
		// may not need to do this because flag is cleared on level interrupts
		GIFR |= (1<<INTF0);
		GIMSK |= (1<<6);
		
	} else {
		// we have been awake, and the signal now is to shut down
		// (this is only for testing, change this in final version)
		Prog_status.gps_Request_Active = 0;
		// change INT0 back to triggering on level low
		// temporarily disable INT0 to prevent sense-change from triggering an interrupt
		GIMSK &= ~(1<<6);
		// change the sense
		MCUCR &= ~((1<<ISC01)|(1<<ISC00));
		// clear the flag bit before enabling INT0 or interrupt will be triggered here
		// may not need to do this because flag is cleared on level interrupts
		GIFR |= (1<<INTF0);
		GIMSK |= (1<<6);		
	}
	
}