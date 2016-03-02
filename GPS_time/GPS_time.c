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
  // Set the bit 0 of port B as output
  DDRB |= (1<<PB0);
  // Set the bit 1 of port B as output
  DDRB |= (1<<PB1);
  // wait 1 second for supply power to stabilize
  _delay_ms(1000);
  // simulate turning on power to GPS
  PORTB |= (1<<PB1);
  // wait 1 second for GPS power to stabilize
  _delay_ms(1000);  
  while(1) {    
    // Turn led on by setting corresponding bit high in the PORTB register.
    PORTB |= (1<<PB0);

    _delay_ms(1000);

    // Turn led off by setting corresponding bit low in the PORTB register.
    PORTB &= ~(1<<PB0);

    _delay_ms(1000);

  }
}