/*
 * GPS_time.c
 *
 * Created: 3/1/2016 1:47:59 PM
 *  Author: Rick Shory
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
  // assure port B bit 0 starts out low	
  PORTB &= ~(1<<PB0);
  // Set the bit 0 of port B as output
  DDRB |= (1<<PB0);
  // assure port B bit 1 starts out low	
  PORTB &= ~(1<<PB1);  
  // Set the bit 1 of port B as output
  DDRB |= (1<<PB1);
  DDRB |= (1<<PB2); // Port B bit 2 output
  // wait 1 second for supply power to stabilize
  _delay_ms(1000);
  // simulate turning on power to GPS
  PORTB |= (1<<PB1);
  // wait 1 second for GPS power to stabilize
  _delay_ms(1000);
  // simulate 200ms pulse to wake GPS
  PORTB |= (1<<PB0); // on
  _delay_ms(200);
  PORTB &= ~(1<<PB0); // off
  while(1) {   
	  // for now, blink bit2 to show standby 
    PORTB |= (1<<PB2);
    _delay_ms(1000);
    PORTB &= ~(1<<PB2);
    _delay_ms(1000);
  }
}