
/*
 * isrs.S
 *
 * Created: 3/9/2016 8:27:10 PM
 *  Author: Rick Shory
 */ 

 #include <avr/io.h>
 #include "GPS_time.h"
 #if defined(__AVR_ATtiny84A__)

/*
 * External Interrupt Request 0
 */
 /*
.global	EXT_INT0_vect
EXT_INT0_vect:
	in	sreg_save, _SFR_IO_ADDR(SREG)
;;	inc	counter_hi
;;	ser	flags
	out	_SFR_IO_ADDR(SREG), sreg_save
	reti
 */

 /*
 * Timer/Counter1 Compare Match A
 */
.global TIM0_COMPA_vect
TIM0_COMPA_vect:
	in	sreg_save, _SFR_IO_ADDR(SREG)
;;	inc	counter_hi
;;	clr	flags
	out	_SFR_IO_ADDR(SREG), sreg_save
	reti

/*
 * Timer/Counter1 Overflow
 */
.global	TIM0_OVF_vect
TIM0_OVF_vect:
	in	sreg_save, _SFR_IO_ADDR(SREG)
;;	inc	counter_hi
;;	ser	flags
	out	_SFR_IO_ADDR(SREG), sreg_save
	reti

/*
 * Pin Change Interrupt Request 0
 */
.global PCINT0_vect
PCINT0_vect:
	in	sreg_save, _SFR_IO_ADDR(SREG)

	;; save our working registers
	push	r18
	push	r19
	push	r20
	push	r21

	rjmp	99f		; we are done here

99:
	pop	r21
	pop	r20
	pop	r19
	pop	r18

	out	_SFR_IO_ADDR(SREG), sreg_save
	reti

 #endif  /* ATtiny84A */