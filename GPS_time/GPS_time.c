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
#include <string.h>
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

#define recBufLen 128

// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)

// function prototypes
void wait200ms(void);
void wait1sec(void);
void sendSetTimeCommand(void);
void setupRxCapture(void);
void restoreCmdDefault(void);
void copyNMEAtoCmd(void);
void signalNMEAtoCmd(void);
int parseNMEA(void);

/*
example
$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68

225446       Time of fix 22:54:46 UTC
A            Navigation receiver warning A = OK, V = warning
4916.45,N    Latitude 49 deg. 16.45 min North
12311.12,W   Longitude 123 deg. 11.12 min West
000.5        Speed over ground, Knots
054.7        Course Made Good, True
191194       Date of fix  19 November 1994
020.3,E      Magnetic variation 20.3 deg East
*68          mandatory checksum
*/

// NMEA sentence, for example:
//$GPRMC,110919.000,A,4532.1047,N,12234.3348,W,1.98,169.54,090316,,,A*77
enum NMEA_fields {sentenceType, timeStamp, isValid, curLat, isNorthOrSouth, curLon, isEastOrWest,
	speedKnots, trueCourse, dateStamp, magVar, varEastOrWest, checkSum};

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
        unsigned char cmd_Tx_ongoing:1;
        unsigned char new_NMEA_Field:1;
        unsigned char flag7:1;
    };
} Prog_status = {0};

static volatile uint8_t xMitBitCount=0, rCvBitCount = 0;
static volatile char receiveByte;
static volatile char recBuf[recBufLen];
static volatile char *recBufInPtr;
static volatile char cmdOut[recBufLen] = "t2016-03-19 20:30:01 -08\n\r\n\r\0";
static volatile char *cmdOutPtr;
static volatile char *NMEA_Ptrs[13]; // array of pointers to field positions within the captured NMEA sentence

int main(void)
{
	recBufInPtr = recBuf; // set pointer to start of buffer
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
		// start listening for serial data from GPS
		setupRxCapture();
		// wait for interrupt to set flag high
		while(Prog_status.gps_Request_Active == 0) { // blink slow
			// for testing, overwrite default output message with what's received in the NMEA buffer
//			copyNMEAtoCmd();
			signalNMEAtoCmd();
			// for testing, send a dummy string, as if the set-time command
			sendSetTimeCommand();
			wait1sec();
			restoreCmdDefault();
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
		while(Prog_status.gps_Request_Active == 1) { // blink slower
//			copyNMEAtoCmd();
			signalNMEAtoCmd();
			// for testing, send a dummy string, as if the set-time command
			sendSetTimeCommand();
			wait200ms();
			wait1sec();
			restoreCmdDefault();
			output_high(PORTA, LED);
			wait200ms();
			wait1sec();
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
	OCR0A = 104; // normal use, 9600 baud; use prescaler 8
//	OCR0A = 252; // for testing, 31 baud; use prescaler 1024
	OCR0B = OCR0A;	// simultaneous match using B
	//can't use channel A as the output because it's the same pin as the external interrupt input
	// but only channel A will cause CTC

	// note that WGM0[2:0] is split over TCCR0A and TCCR0B
	// WGM02 is TCCR0B[3] while WGM0[1:0] is TCCR0A[1:0]

	// TCCR0A – Timer/Counter0 Control Register A
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
	
	// TCCR0B – Timer/Counter0 Control Register B
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
	TCCR0B = 0b11000010;
	// use CS0[2:0]=0b101, (prescale 1024) to test at 31 baud, OCR0A=252
//	TCCR0B = 0b11000101;

	// TIMSK0 – Timer/Counter 0 Interrupt Mask Register
	// (7:3 reserved)
	//  When these bits are written to one, and the I-flag in the Status Register is set (interrupts globally
	//  enabled), the corresponding interrupt is enabled. The
	//  corresponding Interrupt Vector is executed when the
	//  corresponding bit is set in the Timer/Counter Interrupt Flag Register – TIFR0.
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
	xMitBitCount = 0;
	cmdOutPtr = cmdOut;
	Prog_status.cmd_Tx_ongoing = 1;
	sei(); // re-enable interrupts
}

void setupRxCapture(void) {
	// shut down Timer 1, if enabled
	TIMSK1 &= ~(1<<OCIE1A); // disable A match interrupt
//	TIMSK1 = 0; // maybe just disable everything
	rCvBitCount = 0;
	// enable pin-change interrupt
	PCMSK0 |= (1<<PCINT2); // enable only for PA2, RX_GPS_NMEA
	cli(); // temporarily disable interrupts
	GIMSK |= (1<<PCIE0); // enable pin change 0
	sei(); // re-enable interrupts
}

int parseNMEA(void) {
	char *endParsePtr, *parsePtr = recBuf;
	int fldCounter = sentenceType; // 0th NMEA field
	// cleanly get the current position of the NMEA buffer pointer
	cli();
	endParsePtr = recBufInPtr;
	sei();
	Prog_status.new_NMEA_Field = 1; // we are starting to work on the first field
	while (1) {
		if (parsePtr++ > endParsePtr) { // if we can't complete the parsing
			return 1; // exit, flag that it failed
		}
		if (*parsePtr == ',') { // the field delimiter
			if (Prog_status.new_NMEA_Field == 1) { // we just started working on a field and
				// encountered a comma indicating the next field, so the current field is empty
				NMEA_Ptrs[fldCounter] = NULL;
			}
			fldCounter++; // point to the next field
			Prog_status.new_NMEA_Field = 1; // working on a new field
		} else { // not a comma
			if (Prog_status.new_NMEA_Field == 1) { // we are starting a new field
				// we have a non-comma character, so the field contains something
				NMEA_Ptrs[fldCounter] = parsePtr; // plant the field pointer here
				Prog_status.new_NMEA_Field = 0; // the field is now no longer new
			}			
		}
		if (fldCounter > sentenceType) { // if we've got sentence-type complete, test for "GPRMC"
			// optimize test to fail early if invalid
			// for testing break down conditions 
			if (*(NMEA_Ptrs[sentenceType] + 4) != 'C') return 6;
			if (*(NMEA_Ptrs[sentenceType] + 3) != 'M') return 5;
			if (*(NMEA_Ptrs[sentenceType] + 2) != 'R') return 4;
			if (*(NMEA_Ptrs[sentenceType] + 1) != 'P') return 3;
			if (*(NMEA_Ptrs[sentenceType]) != 'G') return 2; 
			// not a sentence type we can use
		}
		if (fldCounter > isValid) { // if we've got the validity char, test it
			if (*(NMEA_Ptrs[isValid]) != 'A') {
				return 9; // GPS says data not valid
			}
		}
		if (fldCounter > dateStamp) { // don't need any fields after this
			return 0;
		}
	}
	
}

void restoreCmdDefault(void) {
	// restore the output buffer to its default 't2016-03-19 20:30:01 -08'
	cmdOut[0] = 't';
	cmdOut[1] = '2';
	cmdOut[2] = '0';
	cmdOut[3] = '1';
	cmdOut[4] = '6';
	cmdOut[5] = '-';
	cmdOut[6] = '0';
	cmdOut[7] = '3';
	cmdOut[8] = '-';
	cmdOut[9] = '1';
	cmdOut[10] = '0';
	cmdOut[11] = ' ';
	cmdOut[12] = '2';
	cmdOut[13] = '0';
	cmdOut[14] = ':';
	cmdOut[15] = '3';
	cmdOut[16] = '0';
	cmdOut[17] = ':';
	cmdOut[18] = '0';
	cmdOut[19] = '1';
	cmdOut[20] = ' ';
	cmdOut[21] = '-';
	cmdOut[22] = '0';
	cmdOut[23] = '8';
	cmdOut[24] = '\n';
	cmdOut[25] = '\r';
	cmdOut[26] = '\n';
	cmdOut[27] = '\r';
	cmdOut[28] = '\0';
}

void signalNMEAtoCmd(void) {
	// for testing, overwrite the 1st char in the output message with the results of the parsing function
	int parseResult = parseNMEA();
	cmdOut[0] = (char)(parseResult + '0');
}

void copyNMEAtoCmd(void) {
	// for testing, overwrite default output message with what's received in the NMEA buffer
	char *startPtr, *endPtr, *tmpCmdPtr;
	startPtr = recBuf; // point to start
	tmpCmdPtr = cmdOut;
	// cleanly copy the current position of the NMEA buffer pointer
	cli();
	endPtr = recBufInPtr;
	sei();
	if (endPtr == startPtr) return; // if nothing, leave as-is
	while (startPtr < endPtr) { // copy over the characters
		*tmpCmdPtr++ = *startPtr++;
	}
	// finish off the string with \n\r\n\r\0
	*tmpCmdPtr++ = '\n';
	*tmpCmdPtr++ = '\r';
	*tmpCmdPtr++ = '\n';
	*tmpCmdPtr++ = '\r';
	*tmpCmdPtr++ = '\0';

	// to emulate parsing, reset NMEA string
	cli();
	recBufInPtr = recBuf;
	sei();

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
	
	// Note that even though we are in the A interrupt, we are manipulating the B pin.
	// Cannot use the A pin because it's the same as the external interrupt input.
	
	if (!Prog_status.cmd_Tx_ongoing) {
		TCCR0A |= (1<<COM0B0); // output high, idle
		return;
	}
	// on first entry of this ISR, xMitBitCount = 0 and pin level is high
	// cmdOutPtr points to first byte in buffer
	if (xMitBitCount == 0) { // ready for start bit
		// set to go low next time
		TCCR0A &= ~(1<<COM0B0); // next match will set output low
	} else if (xMitBitCount >= 9) { // done with data bits
		// go back to idle state
		TCCR0A |= (1<<COM0B0); // next match will set the output high
	} else { // set up bit data
		if ((*cmdOutPtr) & (1<<(xMitBitCount-1))) { // if bit is 1
			TCCR0A |= (1<<COM0B0); // next match will set the output high
		} else { // bit is 0
			TCCR0A &= ~(1<<COM0B0); // next match will set output low
		}
	}
	
	// make decisions about next time this ISR is entered
	if (xMitBitCount > 10) { // maybe make this 9
		// next character
		*cmdOutPtr++;
		if (*cmdOutPtr == '\0') {
			// in real use, shut everything down
			// for testing, flag that we are done transmitting
			Prog_status.cmd_Tx_ongoing = 0; 
//			cmdOutPtr = cmdOut;
//			xMitBitCount = 0;
		} else {
			xMitBitCount = 0; // start next character
		}
	} else {
		xMitBitCount += 1; // ready for next bit of same character
	}
	
	// xMitBitCount keeps incrementing, but testing interval should reset it to zero before rollover	
	// The OCF0 flag is automatically cleared when the interrupt is executed.
	// interrupt keeps occurring though no change on output

}

ISR(PCINT0_vect) {
	// this should be the falling edge of the start bit of the received serial byte
	rCvBitCount = 0;
	if (PORTA & (1<<RX_GPS_NMEA)) { // if high, this is not a valid start bit
		setupRxCapture(); // reset and exit
		return;
	}
	GIMSK &= ~(1<<PCIE0); // turn off pin-change interrupt till done receiving this byte
	// load timer 1 match with one-half bit time
	OCR1A = 833; // 4800 baud; use prescaler 1, try different latency compensations
	TIMSK1 &= ~(1<<OCIE1A); // assure timer 1 A match interrupt is disabled
	
	// TCCR1A – Timer/Counter1 Control Register A
	// 7 COM1A1
	// 6 COM1A0
	// 5 COM1B1
	// 4 COM1B0
	// (3:2 reserved)
	// 1 WGM11
	// 0 WGM10
	
	// TCCR1B – Timer/Counter1 Control Register B
	// 7 ICNC1: Input Capture Noise Canceler
	// 6 ICES1: Input Capture Edge Select
	// (5 reserved)
	// 4 WGM13: Waveform Generation Mode
	// 3 WGM12: Waveform Generation Mode
	// 2 CS12 clock select bit 2
	// 1 CS11 clock select bit 1
	// 0 CS10 clock select bit 0	
	
	// note that WGM1[3:0] is in two parts, WGM1[3:2]=TCCR1B[4:3] while WGM1[1:0]=TCCR1A[1:0]
	
	// WGM1[3:0] = 0100, CTC (Clear Timer on Compare) of OCR1A so it auto-clears
	// CS1[2:0] = 001, clkI/O/1 (No pre-scaling)
	TCCR1A = 0b00000000;
	TCCR1B = 0b00001000; // disable the timer
	TCCR1C - 0; // assure compatibility
	TCNT1 = 0; // clear counter
	TCCR1B = 0b00001001; // enable the timer
	cli(); // temporarily disable interrupts
	// clear the interrupt flag, write a 1 to the bit location
	TIFR1 |= (1<<OCF1A);
	TIMSK1 |= (1<<OCIE1A); // enable timer 1 A match interrupt
	sei(); // re-enable interrupts
}

ISR(TIM1_COMPA_vect) {
	// occurs when TCNT1 matches OCR1A
	// first, capture the pin state into a bit, whether we end up using it or not,
	// to keep latency minimal and consistent
	if (PINA & (1<<RX_GPS_NMEA)) {
		Prog_status.cur_Rx_Bit = 1;
	} else {
		Prog_status.cur_Rx_Bit = 0;
	}
	if (rCvBitCount == 0) {
		receiveByte = 0;
		// we have timed the first half-bit and should be in the middle of the start bit
		if (Prog_status.cur_Rx_Bit == 1) {
			// invalid, pin should still be low
			setupRxCapture(); // reset and exit
			return;
		} else { // pin is still low, assume OK
			// set timer period to full bit time for the rest of the byte
			// deal with any latency in first half-bit time
			OCR1A = 1667; // 4800 baud; use prescaler 1
			// C the compiler handles the 16-bit access.
			rCvBitCount++;
		}
	} else if (rCvBitCount > 8) { // done receiving byte, this should be the stop bit
		if (Prog_status.cur_Rx_Bit == 0) { // not a valid stop bit
//			// for testing, display whatever byte we got
//			cmdOut[21] = receiveByte;
			setupRxCapture(); // reset and exit
			return;			
		} else { // assume a valid byte
			// store character
			if (receiveByte == '$') { // beginning of NMEA sentence
				recBufInPtr = recBuf; // point to start of buffer
			}
			*recBufInPtr++ = receiveByte; // put character in buffer
			// if overrun for some reason, wrap; probably bad data anyway and will be repeated
			if (recBufInPtr >= (recBuf + recBufLen)) {
				recBufInPtr = recBuf; // wrap overflow to start of buffer
			}
//			// for testing, just copy it to the start of the output buffer
//			cmdOut[0] = receiveByte;
			setupRxCapture(); // reset and exit
			return;
		}
	} else { // one of the data bits
		// byte starts as all zeros, so only set ones
		if (Prog_status.cur_Rx_Bit == 1) {
			receiveByte |= (1<<(rCvBitCount-1));
//			// diagnostics
//			cmdOut[20 - rCvBitCount] = '1';
		} else {
//			// diagnostics
//			cmdOut[20 - rCvBitCount] = '0';			
		}
		rCvBitCount++;
	}
}

