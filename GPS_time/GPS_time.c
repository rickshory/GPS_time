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
#include <avr/sleep.h>

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

#define TIME_REQ PB2
#define INIT_TIME PB7
#define GPS_PWR_ENAB PA7
#define PULSE_GPS PA1
#define RX_GPS_NMEA PA2
#define TX_SET_TIME PB1
#define CMD_OUT PA6

// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

// function prototypes
void wait200ms(void);
void wait1sec(void);
void sendSetTimeCommand(void);

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

static volatile uint8_t bitCount;

int main(void)
{
	
	// assure time-request signal from main starts high
	set_input(DDRB, TIME_REQ); // set as input
	output_high(PORTB, TIME_REQ); // write to the port bit to enable the pull-up resistor
	
	// assure GPS power enable starts low
	output_low(PORTA, GPS_PWR_ENAB);
	set_output(DDRA, GPS_PWR_ENAB);
	// assure pulse signal to GPS starts low
	output_low(PORTA, PULSE_GPS);
	set_output(DDRA, PULSE_GPS);
	set_input(DDRB, TIME_REQ);
	set_output(DDRA, LED);
	
	// set up the external interrupt INT0
	// clear interrupt flag; probably don't need to do this because flag is cleared on level interrupts
	GIFR |= (1<<INTF0);
	// Configure INT0 to trigger on level low
	MCUCR &= ~((1<<ISC01)|(1<<ISC00));
	// Enable the INT0 interrupt
	GIMSK |= (1<<6);
	// set the global interrupt enable bit.
	sei();
	
	while(1) {
		wait1sec();
		// wait for interrupt to set flag high
		while(Prog_status.gps_Request_Active == 0) { // blink slow
			// for testing, send a dummy string, as if the set-time command
			sendSetTimeCommand();
			wait1sec();
			output_high(PORTA, LED);
			wait1sec();
			output_low(PORTA, LED);
		}
		// INT0 interrupt has set flag high
		// wait 1 sec in case anything needs to stabilize
		wait1sec();
		// turn GPS power on
		output_high(PORTA, GPS_PWR_ENAB);
		// wait 1 sec to assure stable
		wait1sec();
		// send 200ms pulse to GPS, to turn on
		output_high(PORTA, PULSE_GPS);
		wait1sec();
		output_low(PORTA, PULSE_GPS);
		_delay_ms(1000);
		// wait for interrupt to set flag low
		while(Prog_status.gps_Request_Active == 1) { // blink fast
			wait200ms();
			output_high(PORTA, LED);
			wait200ms();
			output_low(PORTA, LED);
		}
		// INT0 interrupt has set flag low
		wait1sec();
		// send 200ms pulse to GPS, to turn off
		output_high(PORTA, PULSE_GPS);
		wait200ms();
		output_low(PORTA, PULSE_GPS);
		wait1sec(); // wait for GPS to shut down
		// turn GPS power off
		output_low(PORTA, GPS_PWR_ENAB);
		// continue the main loop
	}
}

void wait200ms(void){
	_delay_ms(200);
}

void wait1sec(void){
	uint8_t i;
	for (i = 0; i < 5; i++)	{
		wait200ms();
	}
}

void sendSetTimeCommand(void) {
	// send the set-time command
	// interrupt-driven, so this fn initiates, then transmission happens concurrently with wait states
	// message to send is in known buffer
	
	// set up 16-bit counter to time the RS-232 output
	// will always be 9600 baud, and F_CPU = 8MHz, so numbers are hardwired
	// can use input capture (IC..) reg to hold number because not using input capture mode
//	ICR1 = 833; // cycles for 9600 baud bit time, no prescaler
	ICR1 = 6250; // for testing at 0.05 sec per transition; use prescaler 64
//	ICR1 = 12500; // for testing at 0.1 sec per transition; use prescaler 64
//	ICR1 = 25000; // for testing at 0.2 sec per transition; use prescaler 64
//	ICR1 = 50000; // for testing at 0.4 sec per transition; use prescaler 64


	// note that WGM1[3:0] is split over TCCR1A and TCCR1B
	// TCCR1A – Timer/Counter1 Control Register A
	// 7 COM1A1
	// 6 COM1A0
	// 5 COM1B1 (not used here, default 0)
	// 4 COM1B0 (not used here, default 0)
	// (3:2 reserved)
	// 1 WGM11, use 0, see below
	// 0 WGM10, use 0, see below

	// Clear Timer on Compare Match (CTC) Mode
	// the counter is cleared to zero when the counter value (TCNT1) 
	// matches ICR1 (WGM1[3:0] = 12), ICFn Interrupt Flag Set
	// [to match OCR1A (WGM1[3:0] = 4), OCnA Interrupt Flag Set]
	// TOV1 flag is set in the same timer clock cycle that the counter counts from MAX to 0x0000
	// ? since timer clears on match, returns to 0, this never happens
	
	// in Compare Output Mode, non-PWM:
	//  for testing set COM1A[1:0], TCCR1A[7:6]=(0:1), Toggle OC1A on Compare Match
	//  in use, set as follows to output high and low serial bits
	//   COM1A[1:0], TCCR1A[7:6]=(1:0), Clear OC1A on Compare Match (Set output to low level)
	//   COM1A[1:0], TCCR1A[7:6]=(1:1), Set OC1A on Compare Match (Set output to high level)
//	TCCR1A = 0b0100000;
	// for testing, set high on match; see if it happens at all
	TCCR1A = 0b1100000;
	
	// TCCR1B – Timer/Counter1 Control Register B
	// 7 ICNC1: Input Capture Noise Canceler (not used here, default 0)
	// 6 ICES1: Input Capture Edge Select (not used here, default 0)
	// 5 (reserved)
	// 4 WGM13, use 1
	// 3 WGM12, use 1
	// 2 CS12 clock select bit 2, use 0
	// 1 CS11 clock select bit 1, use 0
	// 0 CS10 clock select bit 0, use 1
	
	// use CS1[2:0]=0b001, clkI/O/1 (No prescaling)
	// use WGM1[3:0] = 12 = 0b1100
//	TCCR1B = 0b00011001; // use for 9600 baud
// for testing use  CS1[2:0]=0b011, clkI/O/1 (prescaler 64)
	TCCR1B = 0b00011011; // use for testing at 0.1 sec per transition (prescaler 64)
	// TCCR1C – Timer/Counter1 Control Register C
	// for compatibility with future devices, set to zero when TCCR1A is written
	TCCR1C = 0;
	// TIMSK1 – Timer/Counter Interrupt Mask Register 1
	// (7:6 and 4:3 reserved)
	// 5 ICIE1: Timer/Counter1, Input Capture Interrupt Enable
	//  When this bit is written to one, and the I-flag in the Status Register is set (interrupts globally
	//  enabled), the Timer/Countern Input Capture interrupt is enabled. The
	//  corresponding Interrupt Vector (See “Interrupts” on page 66.) is executed when the
	//  ICF1 Flag, located in TIFR1, is set.
	// 2 OCIE1B: Timer/Counter1, Output Compare B Match Interrupt Enable (not used here)
	// 1 OCIE1A: Timer/Counter1, Output Compare A Match Interrupt Enable (not used here)
	// 0 TOIE1: Timer/Counter1, Overflow Interrupt Enable (not used here)
	// see if following works: OC1A shared with MOSI used for programming
	// set OC1A as output, PDIP pin 7, SOIC pin 16; PA6
	DDRA |= (1<<CMD_OUT);
	bitCount = 0;
	cli(); // temporarily disable interrupts
	// set the counter to zero
	TCNT1 = 0;
	// enable Input Capture 1 (serving as Output Compare) interrupt
//	TIMSK1 = 0b00100000;
//	TIMSK1 |= (1<<ICIE1);
	// enable all these interrupts
	TIMSK1 = 0b00100110;
	// clear the interrupt flag, write a 1 to the bit location
//	TIFR1 |= (1<<ICF1);
	// clear all these interrupt flags
	TIFR1 = 0b00100110;
	sei(); // re-enable interrupts
	
}

ISR(EXT_INT0_vect)
{
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

ISR(TIM1_CAPT_vect) {
	// with Input Capture disabled, serves same purpose as Output Compare
	// occurs when TCNT1 matches ICR1
	
	bitCount += 1;
	if (bitCount >= 4) { // for testing, only go 6 transitions
		// disable this interrupt so the transmission stops
//		TIMSK1 &= ~(0b00100000);
//		TIMSK1 &= ~(1<<ICIE1);
		TIMSK1 = 0;
	}
	// clear the flag so this interrupt can occur again
	// The ICF1 flag is automatically cleared when the interrupt is executed.
	// Alternatively the ICF1 flag can be cleared by software by writing a logical one to its I/O bit location.
//	TIFR1 &= ~(1<<ICF1); //OCF1A?
	// for testing, counter is set to toggle output, and to auto clear on match
}

// maybe the wrong interrupt is firing; service them all
ISR(TIM1_COMPA_vect) {
	bitCount += 1;
	if (bitCount >= 4) {
		TIMSK1 = 0; // mask them all
	}
}
ISR(TIM1_COMPB_vect) {
	bitCount += 1;
	if (bitCount >= 4) {
		TIMSK1 = 0; // mask them all
	}
}