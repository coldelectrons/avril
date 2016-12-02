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


enum TimerMode {
    TIMER_NORMAL = 0,
    TIMER_PWM_PHASE_CORRECT = 1,
    TIMER_CTC = 2,
    TIMER_FAST_PWM = 3,
    TIMER_FAST_PWM_TOP = 7,
};


struct AtomicGuard {
    uint8_t sreg;

    AtomicGuard()
    {
        sreg = SREG;
        cli();
    }
    ~AtomicGuard() { SREG = sreg; }
};

namespace Register {
#if defined( TCCR0A )
SpecialFunctionRegister8( TCCR0A );
SpecialFunctionRegister8( TCCR0B );
#endif
#if defined( TCCR0C )
SpecialFunctionRegister8( TCCR0C );
#endif
#if defined( TCCR0 )
SpecialFunctionRegister8( TCCR0 );
#endif
#if defined( TIMSK0 )
SpecialFunctionRegister8( TIMSK0 );
#endif
#if defined( TIMSK )
SpecialFunctionRegister8( TIMSK );
#endif
SpecialFunctionRegister8( TCNT0 );
SpecialFunctionRegister8( OCR0A );
SpecialFunctionRegister8( OCR0B );

SpecialFunctionRegister8( TCCR1A );
SpecialFunctionRegister8( TCCR1B );
SpecialFunctionRegister8( TIMSK1 );
SpecialFunctionRegister16( TCNT1 );
SpecialFunctionRegister16( OCR1A );
SpecialFunctionRegister16( OCR1B );

SpecialFunctionRegister8( TCCR2A );
SpecialFunctionRegister8( TCCR2B );
SpecialFunctionRegister8( TIMSK2 );
SpecialFunctionRegister8( TCNT2 );
SpecialFunctionRegister8( OCR2A );
SpecialFunctionRegister8( OCR2B );


#ifdef HAS_TIMER3
SpecialFunctionRegister8( TCCR3A );
SpecialFunctionRegister8( TCCR3B );
SpecialFunctionRegister8( TIMSK3 );
SpecialFunctionRegister16( TCNT3 );
SpecialFunctionRegister16( OCR3A );
SpecialFunctionRegister16( OCR3B );
#endif  // HAS_TIMER3

#ifdef HAS_TIMER4
SpecialFunctionRegister8( TCCR4A );
SpecialFunctionRegister8( TCCR4B );
SpecialFunctionRegister8( TIMSK4 );
SpecialFunctionRegister16( TCNT4 );
SpecialFunctionRegister16( OCR4A );
SpecialFunctionRegister16( OCR4B );
#endif  // HAS_TIMER4

#ifdef HAS_TIMER5
SpecialFunctionRegister8( TCCR5A );
SpecialFunctionRegister8( TCCR5B );
SpecialFunctionRegister8( TIMSK5 );
SpecialFunctionRegister16( TCNT5 );
SpecialFunctionRegister16( OCR5A );
SpecialFunctionRegister16( OCR5B );
#endif  // HAS_TIMER5
} // namespace Register

template <typename ControlRegisterA, typename ControlRegisterB,
          typename ControlRegisterC>
struct TimerControlABC {
    using A = ControlRegisterA;
    using B = ControlRegisterB;
    using C = ControlRegisterC;
};

template <typename ControlRegisterA, typename ControlRegisterB>
struct TimerControlAB {
    using A = ControlRegisterA;
    using B = ControlRegisterB;
};

template <typename ControlRegister>
struct TimerControl {
    using A = ControlRegister;
};

#if defined( TCCR0C )
using TimerControl0 = TimerControlABC<>;
#elif defined( TCCR0B )
#elif defined( TCCR0 )
#else
#error
#endif

template <typename ControlRegisterA, typename ControlRegisterB,
          typename InterruptRegister, typename ValueRegister>
struct TimerImpl {
    using A = ControlRegisterA;
    using B = ControlRegisterB;
    using ValueType = typename ValueRegister::type;

    static inline uint8_t Value() { return *ValueRegister::ptr(); }
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

    static inline void SetValue( uint8_t value )
    {
        *ValueRegister::ptr() = value;
    }

    static inline void SetPrescaler( uint8_t prescaler )
    {
        *ControlRegisterB::ptr() =
            ( *ControlRegisterB::ptr() & 0xf8 ) | prescaler;
    }
    /// set tick rate of OVF interrupt in hertz
    /// returns actual rate achieved
    /// assumes FAST_PWM mode
    static inline uint32_t SetTickRate( uint32_t hertz )
    {
        uint32_t result = 0;
        // Need to set prescaler and count for correct Hz operation ot ticks
        // compare match register = [ F_CPU / (prescaler * desired interrupt
        // frequency) ] - 1
        const uint32_t psclr[5] = {1, 8, 64, 256, 1024};
        // const uint32_t psclr[7] = {1, 8, 32, 64, 128, 256, 1024};
        const uint64_t cmr[5] = {
            ( F_CPU / 1 * hertz ) - 1, ( F_CPU / 8 * hertz ) - 1,
            ( F_CPU / 64 * hertz ) - 1, ( F_CPU / 256 * hertz ) - 1,
            ( F_CPU / 1024 * hertz ) - 1};
        for ( uint8_t i = 0; i < 5; ++i ) {
            if ( cmr[i] < 256 ) {
                SetMode( TIMER_FAST_PWM_TOP );
                SetPrescaler( i );
                SetValue( cmr[i] );
                result = ( F_CPU / psclr[i] ) / ( cmr[i] + 1 );
                break;
            }
        }

        return result;
    }
};

template <typename ControlRegisterA, typename ControlRegisterB,
          typename InterruptRegister, typename ValueRegister>
struct Timer16Impl {
    using A = ControlRegisterA;
    using B = ControlRegisterB;
    using ValueType = typename ValueRegister::type;

    static inline uint8_t Value()
    {
        AtomicGuard ag;
        return *ValueRegister::ptr();
    }
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

    static inline void SetValue( uint8_t value )
    {
        AtomicGuard ag;
        *ValueRegister::ptr() = value;
    }

    static inline void SetPrescaler( uint8_t prescaler )
    {
        *ControlRegisterB::ptr() =
            ( *ControlRegisterB::ptr() & 0xf8 ) | prescaler;
    }
    /// set tick rate of OVF interrupt in hertz
    /// returns actual rate achieved
    /// assumes FAST_PWM mode
    static inline uint32_t SetTickRate( uint32_t hertz )
    {
        uint32_t result = 0;
        // Need to set prescaler and count for correct Hz operation ot ticks
        // compare match register = [ F_CPU / (prescaler * desired interrupt
        // frequency) ] - 1
        const uint32_t psclr[5] = {1, 8, 64, 256, 1024};
        // const uint32_t psclr[7] = {1, 8, 32, 64, 128, 256, 1024};
        const uint64_t cmr[5] = {
            ( F_CPU / 1 * hertz ) - 1, ( F_CPU / 8 * hertz ) - 1,
            ( F_CPU / 64 * hertz ) - 1, ( F_CPU / 256 * hertz ) - 1,
            ( F_CPU / 1024 * hertz ) - 1};
        for ( uint8_t i = 0; i < 5; ++i ) {
            if ( cmr[i] < 256 ) {
                SetMode( TIMER_FAST_PWM_TOP );
                SetPrescaler( i );
                SetValue( cmr[i] );
                result = ( F_CPU / psclr[i] ) / ( cmr[i] + 1 );
                break;
            }
        }

        return result;
    }
};

template <typename ControlRegisterA, typename ControlRegisterB,
          typename InterruptRegister, typename ValueRegister>
struct TimerAsyncImpl {
    using A = ControlRegisterA;
    using B = ControlRegisterB;
    using ValueType = typename ValueRegister::type;

    static inline uint8_t Value() { return *ValueRegister::ptr(); }
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

    static inline void SetValue( uint8_t value )
    {
        *ValueRegister::ptr() = value;
    }

    static inline void SetPrescaler( uint8_t prescaler )
    {
        *ControlRegisterB::ptr() =
            ( *ControlRegisterB::ptr() & 0xf8 ) | prescaler;
    }
    /// set tick rate of OVF interrupt in hertz
    /// returns actual rate achieved
    /// assumes FAST_PWM mode
    static inline uint32_t SetTickRate( uint32_t hertz )
    {
        uint32_t result = 0;
        // Need to set prescaler and count for correct Hz operation ot ticks
        // compare match register = [ F_CPU / (prescaler * desired interrupt
        // frequency) ] - 1
        const uint32_t psclr[7] = {1, 8, 32, 64, 128, 256, 1024};
        const uint64_t cmr[7] = {
            ( F_CPU / 1 * hertz ) - 1,   ( F_CPU / 8 * hertz ) - 1,
            ( F_CPU / 32 * hertz ) - 1,  ( F_CPU / 64 * hertz ) - 1,
            ( F_CPU / 128 * hertz ) - 1, ( F_CPU / 256 * hertz ) - 1,
            ( F_CPU / 1024 * hertz ) - 1};
        for ( uint8_t i = 0; i < 7; ++i ) {
            if ( cmr[i] < 256 ) {
                SetMode( TIMER_FAST_PWM_TOP );
                SetPrescaler( i );
                SetValue( cmr[i] );
                result = ( F_CPU / psclr[i] ) / ( cmr[i] + 1 );
                break;
            }
        }

        return result;
    }
};



template <int n>
struct NumberedTimer {
};

// timers can be 8bit, 16bit
// timers can be sync or async

// TCCRs can be 1-3 bytes

template <>
struct NumberedTimer<0> {
    using Impl = TimerImpl<Register::_TCCR0A, Register::_TCCR0B, Register::_TIMSK0,
                           Register::_TCNT0>;
};

template <>
struct NumberedTimer<1> {
    using Impl = Timer16Impl<Register::_TCCR1A, Register::_TCCR1B, Register::_TIMSK1,
                             Register::_TCNT1>;
};

template <>
struct NumberedTimer<2> {
    using Impl = TimerAsyncImpl<Register::_TCCR2A, Register::_TCCR2B, Register::_TIMSK2,
                                Register::_TCNT2>;
};

#ifdef HAS_TIMER3
template <>
struct NumberedTimer<3> {
    using Impl = Timer16Impl<Register::_TCCR3A, Register::_TCCR3B, Register::_TIMSK3,
                             Register::_TCNT3>;
};
#endif  // HAS_TIMER3

#ifdef HAS_TIMER4
template <>
struct NumberedTimer<4> {
    using Impl = Timer16Impl<Register::_TCCR4A, Register::_TCCR4B, Register::_TIMSK4,
                             Register::_TCNT4>;
};
#endif  // HAS_TIMER4

#ifdef HAS_TIMER5
template <>
struct NumberedTimer<5> {
    using Impl = Timer16Impl<Register::_TCCR5A, Register::_TCCR5B, Register::_TIMSK5,
                             Register::_TCNT5>;
};
#endif  // HAS_TIMER5

template <int n>
struct Timer {
    using Impl = typename NumberedTimer<n>::Impl;
    using ValueType = typename NumberedTimer<n>::Impl::ValueType;
    static inline uint8_t Value() { return Impl::Value(); }
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
        return Impl::SetTickRate( hertz );
    }
};

template <int n>
struct Timer16 {
    typedef typename NumberedTimer<n>::Impl Impl;
    static inline uint16_t Value() { return Impl::Value(); }
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
        return Impl::SetTickRate( hertz );
    }
};

template <typename Timer, uint8_t enabled_flag, typename PwmRegister>
struct PwmChannel {
    using EnabledBit = BitInRegister<typename Timer::Impl::A, enabled_flag>;
    enum { has_pwm = 1 };
    static inline void Start() { EnabledBit::Set(); }
    static inline void Stop() { EnabledBit::Clear(); }
    static inline void Write( uint8_t value ) { *PwmRegister::ptr() = value; }
};

template <typename Timer, uint8_t enabled_flag, typename PwmRegister>
struct PwmChannel16 {
    using EnabledBit = BitInRegister<typename Timer::Impl::A, enabled_flag>;
    enum { has_pwm = 1 };
    static inline void Start() { EnabledBit::Set(); }
    static inline void Stop() { EnabledBit::Clear(); }
    static inline void Write( uint16_t value )
    {
        AtomicGuard ag;
        *PwmRegister::ptr() = value;
    }
};

struct NoPwmChannel {
    enum { has_pwm = 0 };
    static inline void Start() {}
    static inline void Stop() {}
    static inline void Write( uint8_t ) {}
};

// TODO create config header like for serial to create proper number of
// pwm channels

// These are 8-bit pwm channels
typedef PwmChannel<Timer<0>, COM0A1, Register::_OCR0A> PwmChannel0A;
typedef PwmChannel<Timer<0>, COM0B1, Register::_OCR0B> PwmChannel0B;
typedef PwmChannel<Timer<2>, COM2A1, Register::_OCR2A> PwmChannel2A;
typedef PwmChannel<Timer<2>, COM2B1, Register::_OCR2B> PwmChannel2B;

// These are 16-bit pwm channels
// TODO Need to specialize these channels to handle 16-bit registers
typedef PwmChannel16<Timer16<1>, COM1A1, Register::_OCR1A> PwmChannel1A;
typedef PwmChannel16<Timer16<1>, COM1B1, Register::_OCR1B> PwmChannel1B;

#ifdef HAS_TIMER3
typedef PwmChannel<Timer16<3>, COM3A1, Register::_OCR3A> PwmChannel3A;
typedef PwmChannel<Timer16<3>, COM3B1, Register::_OCR3B> PwmChannel3B;
typedef PwmChannel<Timer16<3>, COM3C1, Register::_OCR3C> PwmChannel3C;
#endif  // HAS_TIMER3
#ifdef HAS_TIMER4
typedef PwmChannel<Timer16<4>, COM4A1, Register::_OCR4A> PwmChannel4A;
typedef PwmChannel<Timer16<4>, COM4B1, Register::_OCR4B> PwmChannel4B;
typedef PwmChannel<Timer16<4>, COM4C1, Register::_OCR4C> PwmChannel4C;
#endif  // HAS_TIMER4
#ifdef HAS_TIMER5
typedef PwmChannel<Timer16<5>, COM5A1, Register::_OCR5A> PwmChannel5A;
typedef PwmChannel<Timer16<5>, COM5B1, Register::_OCR5B> PwmChannel5B;
typedef PwmChannel<Timer16<5>, COM5C1, Register::_OCR5C> PwmChannel5C;
#endif  // HAS_TIMER5


// Readable aliases for timer interrupts.
#define TIMER_0_TICK ISR( TIMER0_OVF_vect )
#define TIMER_1_TICK ISR( TIMER1_OVF_vect )
#define TIMER_2_TICK ISR( TIMER2_OVF_vect )

#ifdef HAS_TIMER3
#define TIMER_3_TICK ISR( TIMER3_OVF_vect )
#define TIMER_4_TICK ISR( TIMER4_OVF_vect )
#define TIMER_5_TICK ISR( TIMER5_OVF_vect )
#endif  // HAS_TIMER3

}  // namespace avril

#endif  // AVRIL_TIMER_H_
