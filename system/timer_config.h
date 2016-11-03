#pragma once
// Copyright 2016 Thomas Fritz.
//
// Author: Thomas Fritz (frithomas@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// -----------------------------------------------------------------------------

#include <avr/io.h>
#include <avr/interrupt.h>

/*
 *  constants and macros
 */


#if defined(__AVR_AT90S2313__) \
 || defined(__AVR_AT90S4414__) || defined(__AVR_AT90S4434__) \
 || defined(__AVR_AT90S8515__) || defined(__AVR_AT90S8535__) \
 || defined(__AVR_ATmega103__)
	/* old AVR classic or ATmega103 with one timer */
	#define HAS_TIMER0
#elif defined(__AVR_AT90S2333__) || defined(__AVR_AT90S4433__)
	/* old AVR classic with one 8-bit timer and one 16-bit timer */
	#define HAS_TIMER0
	#define HAS_TIMER1
	#define HAS_TIMER1_OC
#elif  defined(__AVR_ATmega8__)  || defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__) \
  || defined(__AVR_ATmega323__)
	/* ATmega with two 8-bit and one 16-bit timers */
	#define HAS_TIMER0
	#define HAS_TIMER1
	#define HAS_TIMER1_OCA
	#define HAS_TIMER1_OCB
	#define HAS_TIMER2
	#define HAS_TIMER1_OC
#elif defined(__AVR_ATmega8U2__) || defined(__AVR_ATmega16U2__) || defined(__AVR_ATmega16U4__) || \
      defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega32U6__)
	/* ATmega with one 8-bit and one 16-bit timers */
	#define HAS_TIMER0
	#define HAS_TIMER0_OCA
	#define HAS_TIMER0_OCB
	#define HAS_TIMER1
	#define HAS_TIMER1_OCA
	#define HAS_TIMER1_OCB
	#define HAS_TIMER1_OCC
#elif  defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__)
	#define HAS_TIMER0
	#define HAS_TIMER0_OC
	#define HAS_TIMER1
	#define HAS_TIMER1_OCA
	#define HAS_TIMER1_OCB
#elif defined(__AVR_ATmega163__) 
	#define HAS_TIMER0
	#define HAS_TIMER1
	#define HAS_TIMER1_OCA
	#define HAS_TIMER1_OCB
	#define HAS_TIMER2
	#define HAS_TIMER2_OC
#elif defined(__AVR_ATmega162__) 
	#define HAS_TIMER0
	#define HAS_TIMER0_OC
	#define HAS_TIMER1
	#define HAS_TIMER1_OCA
	#define HAS_TIMER1_OCB
	#define HAS_TIMER2
	#define HAS_TIMER2_OC
	#define HAS_TIMER3
	#define HAS_TIMER3_OCA
	#define HAS_TIMER3_OCB
#elif defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__) 
	/* ATmega with two USART */
	#define HAS_TIMER0
	#define HAS_TIMER0_OC
	#define HAS_TIMER1
	#define HAS_TIMER1_OCA
	#define HAS_TIMER1_OCB
	#define HAS_TIMER1_OCC
	#define HAS_TIMER2
	#define HAS_TIMER2_OC
	#define HAS_TIMER3
	#define HAS_TIMER3_OCA
	#define HAS_TIMER3_OCB
	#define HAS_TIMER3_OCC
#elif defined(__AVR_ATmega161__)
	/* ATmega with UART */
	#error "AVR ATmega161 currently not supported by this libaray !"
#elif defined(__AVR_ATmega169__) 
	/* ATmega with one USART */
#elif defined(__AVR_ATmega48__) ||defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega48P__) ||defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || \
      defined(__AVR_ATmega328P__) 
	#define HAS_TIMER0
	#define HAS_TIMER0_OCA
	#define HAS_TIMER0_OCB
	#define HAS_TIMER1
	#define HAS_TIMER1_OCA
	#define HAS_TIMER1_OCB
	#define HAS_TIMER2
	#define HAS_TIMER2_OCA
	#define HAS_TIMER2_OCB
#elif defined(__AVR_ATmega328BP__) 
	#define HAS_TIMER0
	#define HAS_TIMER0_OCA
	#define HAS_TIMER0_OCB
	#define HAS_TIMER1
	#define HAS_TIMER1_OCA
	#define HAS_TIMER1_OCB
	#define HAS_TIMER2
	#define HAS_TIMER2_OCA
	#define HAS_TIMER2_OCB
	#define HAS_TIMER3
	#define HAS_TIMER3_OCA
	#define HAS_TIMER3_OCB
	#define HAS_TIMER4
	#define HAS_TIMER4_OCA
	#define HAS_TIMER4_OCB
#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__) || defined(__AVR_ATtiny4313__)
	#define HAS_TIMER0
	#define HAS_TIMER0_OCA
	#define HAS_TIMER0_OCB
	#define HAS_TIMER1
	#define HAS_TIMER1_OCA
	#define HAS_TIMER1_OCB
#elif defined(__AVR_ATmega329__)
/* */
#elif defined(__AVR_ATmega329__) ||\
      defined(__AVR_ATmega649__) ||\
      defined(__AVR_ATmega325__) ||defined(__AVR_ATmega3250__) ||\
      defined(__AVR_ATmega645__) ||defined(__AVR_ATmega6450__)
	/* */
#elif defined(__AVR_ATmega3290__) ||\
      defined(__AVR_ATmega6490__)
	/* */
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega640__)
	/* */
#elif defined(__AVR_ATmega644__)
	/* */
#elif defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || \
      defined(__AVR_ATmega1284P__)
	/* */
#else
	#error "no UART definition for MCU available"
#endif

