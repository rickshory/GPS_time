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
// PA7  6  15 OC0B, used for set-time signal out
// PB0  2  11
// PB1  3  12
// PB2  5  14 pin reserved for INT0
// PB3  4  13

#define LED PA3

#define TIME_REQ PB2
#define GPS_PWR_ENAB PA0
#define PULSE_GPS PA1
#define RX_GPS_NMEA PA2
#define CMD_OUT PA7

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
        unsigned char cur_Rx_Bit:1;
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
//	set_input(DDRB, TIME_REQ);
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
		wait200ms();
		output_low(PORTA, PULSE_GPS);
		wait1sec();
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
	
	// set up 8-bit counter to time the RS-232 output
	// will always be 9600 baud, and F_CPU = 8MHz, so numbers are hardwired
//	OCR0A = 104; // normal use, 9600 baud; use prescaler 8
	OCR0A = 252; // for testing, 31 baud; use prescaler 1024
	// simultaneous match using B; 
	//can't use channel A as the output because it's the same pin as the external interrupt input
	// but only channel A will cause CTC
	OCR0B = 252;

	// note that WGM0[2:0] is split over TCCR0A and TCCR0B
	// WGM02 is TCCR0B[3] while WGM0[1:0] is TCCR0A[1:0]

	// TCCR0A � Timer/Counter0 Control Register A
	// 7 COM0A1
	// 6 COM0A0
	// 5 COM0B1
	// 4 COM0B0
	// (3:2 reserved)
	// 1 WGM01
	// 0 WGM00
	
	// Clear Timer on Compare Match (CTC) Mode WGM0[2:0]=(010)
	// the counter is cleared to zero when the counter value (TCNT0) 
	// matches OCR0A, OCFOA Interrupt Flag Set
	// TOV0 flag is set in the same timer clock cycle that the counter counts from MAX to 0x0000
	// ? since timer clears on match, returns to 0, this never happens
	
	// in Compare Output Mode, non-PWM:
	// set COM0A to just toggle, to enable CTC, which only happens by matches on OCR0A, but 
	//  do not allow output A to control its physical pin because it's the same as
	//  the external interrupt (PB2). Instead, output the signal on OC0B (PA7).
	//   COM0A[1:0], TCCR0A[7:6]=(0:1), Let OC0A just toggle on Compare Match	
	// We will use the OC0A interrupt, but within it, control the action of OC0B
	//  in ISR, set as follows to output high and low serial bits
	//   COM0B[1:0], TCCR0A[5:4]=(1:0), Clear OC0B on next Compare Match (Set output to low level)
	//   COM0B[1:0], TCCR0A[5:4]=(1:1), Set OC0B on next Compare Match (Set output to high level)
	
	// default is to set high, for serial idle, Stop Bit, and to be ready for low Start Bit
	TCCR0A = 0b01110010;
	
	// TCCR0B � Timer/Counter0 Control Register B
	// 7 FOC0A: Force Output Compare A
	// 6 FOC0B: Force Output Compare B
	// (5:4, reserved)
	// 3 WGM02: Waveform Generation Mode
	// 2 CS02 clock select bit 2
	// 1 CS01 clock select bit 1
	// 0 CS00 clock select bit 0
	
	// use FOC0A = 1 to strobe output, and force high for default idle
	// use WGM02 = 0 for CTC Mode
	// use CS0[2:0]=0b001, for no prescaling
	// use CS0[2:0]=0b010, (prescale 8) in normal 9600 baud output, OCR0A=104
//	TCCR0B = 0b11000010;
	// use CS0[2:0]=0b101, (prescale 1024) to test at 31 baud, OCR0A=252
	TCCR0B = 0b11000101;

	// TIMSK0 � Timer/Counter 0 Interrupt Mask Register
	// (7:3 reserved)
	//  When these bits are written to one, and the I-flag in the Status Register is set (interrupts globally
	//  enabled), the corresponding interrupt is enabled. The
	//  corresponding Interrupt Vector is executed when the
	//  corresponding bit is set in the Timer/Counter Interrupt Flag Register � TIFR0.
	// 2 OCIE0B: Timer/Counter Output Compare Match B Interrupt Enable (not used here)
	// 1 OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
	// 0 TOIE1: Timer/Counter1, Overflow Interrupt Enable (not used here)
	
	//The setup of the OC0x should be performed before setting the Data Direction Register for the
	//port pin to output. The easiest way of setting the OC0x value is to use the Force Output Compare
	//(0x) strobe bits in Normal mode. The OC0x Registers keep their values even when
	//changing between Waveform Generation modes.
	
	// Do not set OC0A as output (PDIP pin 5, SOIC pin 14) because it's the same pin (PB2) as the 
	//  external interrupt used for wake-up. Instead, use OC0B (PA7) for the actual signal (PDIP pin 6, SOIC pin 15)
	DDRA |= (1<<CMD_OUT);
	cli(); // temporarily disable interrupts
	// set the counter to zero
	TCNT0 = 0;
	// enable Output Compare 0A interrupt
	TIMSK0 |= (1<<OCIE0A);
	// clear the interrupt flag, write a 1 to the bit location
	TIFR0 |= (1<<OCF0A);
	bitCount = 0;
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

ISR(TIM0_COMPA_vect) {
	// occurs when TCNT0 matches OCR0A
	
	// pin transition has occurred as this interrupt was called, so 
	// we have plenty of time to set up the next one, no latency to worry about
	bitCount += 1;
	// Note that even though we are in the A interrupt, we are manipulating the B pin.
	// Cannot use the A pin because it's the same as the external interrupt input.
	if (bitCount <= 10) { // for testing, only go 10 transitions
		if (TCCR0A & (1<<COM0B0)) { // last match set the output high
			TCCR0A &= ~(1<<COM0B0); // next match will set output low
		} else { // last match set the output low
			TCCR0A |= (1<<COM0B0); // next match will set the output high
		}
	} else { // set high, idle
		TCCR0A |= (1<<COM0B0);
	}
	// clear the flag so this interrupt can occur again
	// The OCF0 flag is automatically cleared when the interrupt is executed.
	// Alternatively the OCF0 flag can be cleared by software by writing a logical one to its I/O bit location.
//	TIFR0 &= ~(1<<OCF0A);

	// interrupt keeps occurring though no change on output
	// bitCount keeps incrementing, but testing interval should reset it to zero before rollover
}



