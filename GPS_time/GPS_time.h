/*
 * GPS_time.h
 *
 * Created: 3/9/2016 8:32:29 PM
 *  Author: Rick Shory
 */ 


#ifndef GPS_TIME_H_
#define GPS_TIME_H_

/*
 * Global register variables.
 */
#ifdef __ASSEMBLER__

#  define sreg_save	r2
#  define flags		r16
#  define counter_hi    r4

#else  /* !ASSEMBLER */

#include <stdint.h>

register uint8_t sreg_save asm("r2");
register uint8_t flags     asm("r16");
register uint8_t counter_hi asm("r4");

#endif /* ASSEMBLER */

#endif /* GPS_TIME_H_ */