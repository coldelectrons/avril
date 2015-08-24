// Copyright 2009 Olivier Gillet.
//
// Author: Olivier Gillet (ol.gillet@gmail.com)
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
//
// Basic ATmega initialization.

#ifndef AVRIL_BOOT_H_
#define AVRIL_BOOT_H_

#include "avril/adc.h"
#include "avril/avrlib.h"
#include "avril/time.h"
#include "avril/timer.h"

namespace avril {

inline void Boot(bool init_timers) {
  sei();

  if (init_timers) {
    Timer<1>::set_prescaler(3);
    Timer<1>::set_mode(TIMER_PWM_PHASE_CORRECT);

    Timer<2>::set_prescaler(3);
    Timer<2>::set_mode(TIMER_PWM_PHASE_CORRECT);
    InitClock();
  }

  // Neuter the UARTs.
#ifdef HAS_USART0
  UCSR0B = 0;
#endif

#ifdef HAS_USART1
  UCSR1B = 0;
#endif

#ifdef HAS_USART2
  UCSR2B = 0;
#endif

#ifdef HAS_USART3
  UCSR3B = 0;
#endif
}

}  // avr

#endif  // AVRIL_BOOT_H_
