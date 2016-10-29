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
// Definitions of timer and related PWM registers.

#ifndef AVRIL_TIMER_H_
#define AVRIL_TIMER_H_

// interrupt.h is not strictly needed here, but .cc files including the timer
// classes are likely to also define interrupt handlers (and we have macros for
// that).
#include <avr/interrupt.h>
#include <avr/io.h>

#include "../avril.h"

namespace avril {

SpecialFunctionRegister( TCCR0A );
SpecialFunctionRegister( TCCR0B );
SpecialFunctionRegister( TCCR1A );
SpecialFunctionRegister( TCCR1B );
SpecialFunctionRegister( TCCR2A );
SpecialFunctionRegister( TCCR2B );
SpecialFunctionRegister( TIMSK0 );
SpecialFunctionRegister( TIMSK1 );
SpecialFunctionRegister( TIMSK2 );
SpecialFunctionRegister( TCNT0 );
SpecialFunctionRegister16( TCNT1 );
SpecialFunctionRegister( TCNT2 );
SpecialFunctionRegister( OCR0A );
SpecialFunctionRegister( OCR0B );
SpecialFunctionRegister( OCR1A );
SpecialFunctionRegister( OCR1B );
SpecialFunctionRegister( OCR2A );
SpecialFunctionRegister( OCR2B );

#ifdef HAS_TIMER3
SpecialFunctionRegister( TCCR3A );
SpecialFunctionRegister( TCCR3B );
SpecialFunctionRegister( TIMSK3 );
SpecialFunctionRegister( TCNT3 );
SpecialFunctionRegister( OCR3A );
SpecialFunctionRegister( OCR3B );
#endif  // HAS_TIMER3

enum TimerMode {
    TIMER_NORMAL = 0,
    TIMER_PWM_PHASE_CORRECT = 1,
    TIMER_CTC = 2,
    TIMER_FAST_PWM = 3,
    TIMER_FAST_PWM_TOP = 7,
};

template <typename ControlRegisterA, typename ControlRegisterB,
          typename InterruptRegister, typename ValueRegister>
struct TimerImpl {
    typedef ControlRegisterA A;
    typedef ControlRegisterB B;

    static inline uint8_t Value() const { return *ValueRegister::ptr(); }
    static inline void Start() { *InterruptRegister::ptr() |= _BV( 0 ); }
    static inline void Stop() { *InterruptRegister::ptr() &= ~( _BV( 0 ) ); }
    static inline void StartInputCapture()
    {
        *InterruptRegister::ptr() |= _BV( 5 );
    }
    static inline void StopInputCapture()
    {
        *InterruptRegister::ptr() &= ~( _BV( 5 ) );
    }
    static inline void StartCompare() { *InterruptRegister::ptr() |= _BV( 1 ); }
    static inline void StopCompare()
    {
        *InterruptRegister::ptr() &= ~( _BV( 1 ) );
    }

    static inline void SetMode( TimerMode mode )
    {
        // Sets the mode registers.
        *ControlRegisterA::ptr() = ( *ControlRegisterA::ptr() & 0xfc ) | mode;
    }

    static inline void SetMode( uint8_t wg_mode_1, uint8_t wg_mode_2,
                                uint8_t prescaler )
    {
        // Sets the mode registers.
        *ControlRegisterA::ptr() = wg_mode_1;
        *ControlRegisterB::ptr() = wg_mode_2 | prescaler;
    }

    static inline void SetValue( uint16_t value )
    {
        *ValueRegister::ptr() = value;
    }

    /// set tick rate in hertz
    /// returns actual rate achieved
    static inline uint32_t SetTickRate( uint32_t hertz )
    {
        uint32_t result = 0;
        cli();
        // Need to set prescaler and count for correct Hz operation ot ticks
        // compare match register = [ F_CPU / (prescaler * desired interrupt
        // frequency) ] - 1
        const uint32_t psclr[5] = {1, 8, 64, 256, 1024};
        const uint32_t cmr[5] = {
            ( F_CPU / 1 * hertz ) - 1, ( F_CPU / 8 * hertz ) - 1,
            ( F_CPU / 64 * hertz ) - 1, ( F_CPU / 256 * hertz ) - 1,
            ( F_CPU / 1024 * hertz ) - 1};
        for ( uint8_t i = 0; i < 5; ++i ) {
            if ( cmr[i] < 255 ) {
                SetMode( TIMER_FAST_PWM_TOP );
                SetPrescaler( i );
                //SetMode(_BV(), _BV(), i);
                SetValue( cmr[i] );
                result = ( F_CPU / psclr[i] ) / ( cmr[i] + 1 );
                break;
            }
        }
        sei();

        return result;
    }

    // These are the values for MCUs clocked at 20 MHz
    // 64 prescaler
    //
    // Timer speed
    // value | fast        | accurate
    // --------------------------------------
    // 1     | 78.125 kHz  | 39.215 kHz
    // 2     | 9.765 kHz   | 4.901 kHz
    // 3     | 1220.7 Hz   | 612.7 Hz
    // 4     | 305.2 Hz    | 153.2 Hz
    // 5     | 76.3 Hz     | 38.3 Hz
    //
    // These are the values for MCUs clocked at 16 MHz
    // 64 prescaler
    //
    // Timer speed
    // value | fast        | accurate
    // --------------------------------------
    // 1     | 62.500 kHz  | 32.250 kHz
    // 2     |             |
    // 3     |             |
    // 4     |             |
    // 5     |             |
    static inline void SetPrescaler( uint8_t prescaler )
    {
        *ControlRegisterB::ptr() =
            ( *ControlRegisterB::ptr() & 0xf8 ) | prescaler;
    }
};

template <int n>
struct NumberedTimer {
};

template <>
struct NumberedTimer<0> {
    typedef TimerImpl<TCCR0ARegister, TCCR0BRegister, TIMSK0Register,
                      TCNT0Register>
        Impl;
};

template <>
struct NumberedTimer<1> {
    typedef TimerImpl<TCCR1ARegister, TCCR1BRegister, TIMSK1Register,
                      TCNT1Register>
        Impl;
};

template <>
struct NumberedTimer<2> {
    typedef TimerImpl<TCCR2ARegister, TCCR2BRegister, TIMSK2Register,
                      TCNT2Register>
        Impl;
};

#ifdef HAS_TIMER3
template <>
struct NumberedTimer<3> {
    typedef TimerImpl<TCCR3ARegister, TCCR3BRegister, TIMSK3Register,
                      TCNT3Register>
        Impl;
};
#endif  // HAS_TIMER3

template <int n>
struct Timer {
    typedef typename NumberedTimer<n>::Impl Impl;
    static inline uint8_t Value() const { return Impl::Value(); }
    static inline void Start() { Impl::Start(); }
    static inline void Stop() { Impl::Stop(); }
    static inline void StartInputCapture() { Impl::StartInputCapture(); }
    static inline void StopInputCapture() { Impl::StopInputCapture(); }
    static inline void StartCompare() { Impl::StartCompare(); }
    static inline void StopCompare() { Impl::StopCompare(); }
    static inline void SetMode( TimerMode mode ) { Impl::SetMode( mode ); }
    static inline void SetMode( uint8_t a, uint8_t b, uint8_t c )
    {
        Impl::SetMode( a, b, c );
    }
    static inline void SetPrescaler( uint8_t prescaler )
    {
        Impl::SetPrescaler( prescaler );
    }
    static inline uint32_t SetTickRate( uint32_t hertz )
    {
        Impl::SetTickRate( hertz );
    }
};

template <typename Timer, uint8_t enabled_flag, typename PwmRegister>
struct PwmChannel {
    typedef BitInRegister<typename Timer::Impl::A, enabled_flag> EnabledBit;
    enum { has_pwm = 1 };
    static inline void Start() { EnabledBit::set(); }
    static inline void Stop() { EnabledBit::clear(); }
    static inline void Write( uint8_t value ) { *PwmRegister::ptr() = value; }
    static inline void SetFrequency( uint16_t f )
    {
        // TODO verify what this is actually doing.
        // XXX it doesn't follow anything I read in "Secrets of AVR PWM"
        OCR1A = f;
        OCR1B = f >> 1;
    }
    static inline void SetFrequencyPulse( uint16_t f )
    {
        // TODO verify what this is actually doing.
        // XXX it doesn't follow anything I read in "Secrets of AVR PWM"
        OCR1A = f;
        OCR1B = f - ( f >> 2 );
    }
};

struct NoPwmChannel {
    enum { has_pwm = 0 };
    static inline void Start() {}
    static inline void Stop() {}
    static inline void Write( uint8_t value ) {}
};

typedef PwmChannel<Timer<0>, COM0A1, OCR0ARegister> PwmChannel0A;
typedef PwmChannel<Timer<0>, COM0B1, OCR0BRegister> PwmChannel0B;
typedef PwmChannel<Timer<1>, COM1A1, OCR1ARegister> PwmChannel1A;
typedef PwmChannel<Timer<1>, COM1B1, OCR1BRegister> PwmChannel1B;
typedef PwmChannel<Timer<2>, COM2A1, OCR2ARegister> PwmChannel2A;
typedef PwmChannel<Timer<2>, COM2B1, OCR2BRegister> PwmChannel2B;

// Readable aliases for timer interrupts.
#define TIMER_0_TICK ISR( TIMER0_OVF_vect )
#define TIMER_1_TICK ISR( TIMER1_OVF_vect )
#define TIMER_2_TICK ISR( TIMER2_OVF_vect )

#ifdef HAS_TIMER3
#define TIMER_3_TICK ISR( TIMER3_OVF_vect )
#endif  // HAS_TIMER3

}  // namespace avril

#endif  // AVRIL_TIMER_H_
