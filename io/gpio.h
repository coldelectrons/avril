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
// An alternative gpio library based on templates.
//
// Examples of use:
//
// NumberedGpio<3>::SetMode(DIGITAL_INPUT)
// NumberedGpio<4>::SetMode(DIGITAL_OUTPUT)
// NumberedGpio<3>::Value()
// NumberedGpio<4>::High()
// NumberedGpio<4>::Low()
// NumberedGpio<4>::SetValue(1)
// NumberedGpio<4>::SetValue(0)

#ifndef AVRIL_GPIO_H_
#define AVRIL_GPIO_H_

#include <avr/io.h>

#include "../avril.h"
#include "../system/timer.h"

namespace avril {

enum PinMode { DIGITAL_INPUT = 0, DIGITAL_OUTPUT = 1, PWM_OUTPUT = 2 };

// Represents a i/o port, which has input, output and mode registers.
template <typename InputRegister, typename OutputRegister,
          typename ModeRegister>
struct Port {
    using Input = InputRegister;
    using Output = OutputRegister;
    using Mode = ModeRegister;
};

// All the registers used in the following definitions are wrapped here.
// Definition of I/O ports.

#if defined( PORTA )
namespace Register {
IORegister8( PINA );
IORegister8( PORTA );
IORegister8( DDRA );
}  // namespace Register
using PortA = Port<Register::_PINA, Register::_PORTA, Register::_DDRA>;
#endif
#if defined( PORTB )
namespace Register {
IORegister8( PINB );
IORegister8( PORTB );
IORegister8( DDRB );
}  // namespace Register
using PortB = Port<Register::_PINB, Register::_PORTB, Register::_DDRB>;
#endif
#if defined( PORTC )
namespace Register {
IORegister8( PINC );
IORegister8( PORTC );
IORegister8( DDRC );
}  // namespace Register
using PortC = Port<Register::_PINC, Register::_PORTC, Register::_DDRC>;
#endif
#if defined( PORTD )
namespace Register {
IORegister8( PIND );
IORegister8( PORTD );
IORegister8( DDRD );
}  // namespace Register
using PortD = Port<Register::_PIND, Register::_PORTD, Register::_DDRD>;
#endif
#if defined( PORTE )
namespace Register {
IORegister8( PINE );
IORegister8( PORTE );
IORegister8( DDRE );
}  // namespace Register
using PortE = Port<Register::_PINE, Register::_PORTE, Register::_DDRE>;
#endif
#if defined( PORTF )
namespace Register {
IORegister8( PINF );
IORegister8( PORTF );
IORegister8( DDRF );
}  // namespace Register
using PortF = Port<Register::_PINF, Register::_PORTF, Register::_DDRF>;
#endif
#if defined( PORTG )
namespace Register {
IORegister8( PING );
IORegister8( PORTG );
IORegister8( DDRG );
}  // namespace Register
using PortG = Port<Register::_PING, Register::_PORTG, Register::_DDRG>;
#endif
#if defined( PORTH )
namespace Register {
IORegister8( PINH );
IORegister8( PORTH );
IORegister8( DDRH );
}  // namespace Register
using PortH = Port<Register::_PINH, Register::_PORTH, Register::_DDRH>;
#endif
#if defined( PORTJ )
namespace Register {
IORegister8( PINJ );
IORegister8( PORTJ );
IORegister8( DDRJ );
}  // namespace Register
using PortJ = Port<Register::_PINJ, Register::_PORTJ, Register::_DDRJ>;
#endif
#if defined( PORTK )
namespace Register {
IORegister8( PINK );
IORegister8( PORTK );
IORegister8( DDRK );
}  // namespace Register
using PortK = Port<Register::_PINK, Register::_PORTK, Register::_DDRK>;
#endif
#if defined( PORTL )
namespace Register {
IORegister8( PINL );
IORegister8( PORTL );
IORegister8( DDRL );
}  // namespace Register
using PortL = Port<Register::_PINL, Register::_PORTL, Register::_DDRL>;
#endif

#if defined( PORTA )
using PortA = Port<Register::_PINA, Register::_PORTA, Register::_DDRA>;
#endif
#if defined( PORTB )
using PortB = Port<Register::_PINB, Register::_PORTB, Register::_DDRB>;
#endif
#if defined( PORTC )
using PortC = Port<Register::_PINC, Register::_PORTC, Register::_DDRC>;
#endif
#if defined( PORTD )
using PortD = Port<Register::_PIND, Register::_PORTD, Register::_DDRD>;
#endif
#if defined( PORTE )
using PortE = Port<Register::_PINE, Register::_PORTE, Register::_DDRE>;
#endif
#if defined( PORTF )
using PortF = Port<Register::_PINF, Register::_PORTF, Register::_DDRF>;
#endif
#if defined( PORTG )
using PortG = Port<Register::_PING, Register::_PORTG, Register::_DDRG>;
#endif
#if defined( PORTH )
using PortH = Port<Register::_PINH, Register::_PORTH, Register::_DDRH>;
#endif
#if defined( PORTJ )
using PortJ = Port<Register::_PINJ, Register::_PORTJ, Register::_DDRJ>;
#endif
#if defined( PORTK )
using PortK = Port<Register::_PINK, Register::_PORTK, Register::_DDRK>;
#endif
#if defined( PORTL )
using PortL = Port<Register::_PINL, Register::_PORTL, Register::_DDRL>;
#endif

// The actual implementation of a pin, not very convenient to use because it
// requires the actual parameters of the pin to be passed as template
// arguments.
template <typename Port, typename PwmChannel, uint8_t bit>
struct GpioImpl {
    using ModeBit = BitInRegister<typename Port::Mode, bit>;
    using OutputBit = BitInRegister<typename Port::Output, bit>;
    using InputBit = BitInRegister<typename Port::Input, bit>;
    using Pwm = PwmChannel;

    static inline void SetMode( uint8_t mode )
    {
        if ( mode == DIGITAL_INPUT ) {
            ModeBit::Clear();
        }
        else if ( mode == DIGITAL_OUTPUT || mode == PWM_OUTPUT ) {
            ModeBit::Set();
        }
        if ( mode == PWM_OUTPUT ) {
            PwmChannel::Start();
        }
        else {
            PwmChannel::Stop();
        }
    }

    static inline void High() __attribute__( ( always_inline ) )
    {
        OutputBit::Set();
    }
    static inline void Low() __attribute__( ( always_inline ) )
    {
        OutputBit::Clear();
    }
    static inline void Toggle() __attribute__( ( always_inline ) )
    {
        OutputBit::Toggle();
    }
    static inline void FastToggle() __attribute__( ( always_inline ) )
    {
        InputBit::Set();
    }
    static inline void SetValue( uint8_t value )
        __attribute__( ( always_inline ) )
    {
        if ( value == 0 ) {
            Low();
        }
        else {
            High();
        }
    }

    static inline void SetPwmValue( uint8_t value )
    {
        if ( PwmChannel::has_pwm ) {
            PwmChannel::Write( value );
        }
        else {
            SetValue( value );
        }
    }

    static inline uint8_t Value() __attribute__( ( always_inline ) )
    {
        return InputBit::Value();
    }
    static inline uint8_t IsHigh() __attribute__( ( always_inline ) )
    {
        return InputBit::Value();
    }
    static inline uint8_t IsLow() __attribute__( ( always_inline ) )
    {
        return InputBit::Value() == 0;
    }
};


template <typename port, uint8_t bit>
struct Gpio {
    using Impl = GpioImpl<port, NoPwmChannel, bit>;
    static inline void High() __attribute__( ( always_inline ) )
    {
        Impl::High();
    }
    static inline void Low() __attribute__( ( always_inline ) ) { Impl::Low(); }
    static inline void Toggle() __attribute__( ( always_inline ) )
    {
        Impl::Toggle();
    }
    static inline void FastToggle() __attribute__( ( always_inline ) )
    {
        Impl::FastToggle();
    }
    static inline void SetMode( uint8_t mode ) { Impl::SetMode( mode ); }
    static inline void SetValue( uint8_t value )
        __attribute__( ( always_inline ) )
    {
        Impl::SetValue( value );
    }
    static inline void SetPwmValue( uint8_t value )
    {
        Impl::SetPwmValue( value );
    }
    static inline uint8_t Value() __attribute__( ( always_inline ) )
    {
        return Impl::Value();
    }
    static inline uint8_t IsLow() __attribute__( ( always_inline ) )
    {
        return Impl::IsLow();
    }
    static inline uint8_t IsHigh() __attribute__( ( always_inline ) )
    {
        return Impl::IsHigh();
    }
};

struct DummyGpio {
    static inline void High() {}
    static inline void Low() {}
    static inline void Toggle() {}
    static inline void FastToggle() {}
    static inline void SetMode( uint8_t ) {}
    static inline void SetValue( uint8_t ) {}
    static inline void SetPwmValue( uint8_t ) {}
    static inline uint8_t Value() { return 0; }
    static inline uint8_t IsLow() { return 0; }
    static inline uint8_t IsHigh() { return 0; }
};

template <typename Gpio>
struct Inverter {
    static inline void High() { Gpio::Low(); }
    static inline void Low() { Gpio::High(); }
    static inline void Toggle() { Gpio::Toggle(); }
    static inline void FastToggle() { Gpio::FastToggle(); }
    static inline void SetMode( uint8_t mode ) { Gpio::SetMode( mode ); }
    static inline void SetValue( uint8_t value ) { Gpio::SetValue( !value ); }
    static inline void SetPwmValue( uint8_t value ) { Gpio::SetPwmValue( ~value ); }
    static inline uint8_t Value() { return !Gpio::Value(); }
    static inline uint8_t IsLow() { return !Gpio::IsLow(); }
    static inline uint8_t IsHigh() { return !Gpio::IsHigh(); }
};

template <typename gpio>
struct DigitalInput {
    enum {
        buffer_size = 0,
        data_size = 1,
    };
    static void Init() { gpio::SetMode( DIGITAL_INPUT ); }
    static void EnablePullUpResistor() { gpio::High(); }
    static void DisablePullUpResistor() { gpio::Low(); }
    static uint8_t Read() { return gpio::Value(); }
};

// A template that will be specialized for each pin, allowing the pin number to
// be specified as a template parameter.
// XXX Fritz - how useful is this in practice?  The numbering is arbitrary,
// and what we do here doesn't match Arduino numbering, and non-Arduino
// MCUs there is no 'official' guide to follow.
// Physical layout between packages is also a problem.
// XXX Fritz - to reiterate:  How does numbering the GPIO help us?
// If I go by datasheets, where PORTA is at 0x20, and things number upwards
// following ABCD...
template <int n>
struct NumberedGpioInternal {
};

// Macro to make the pin definitions (template specializations) easier to read.
#define SetupGpio( n, port, timer, bit )         \
    template <>                                  \
    struct NumberedGpioInternal<n> {             \
        using Impl = GpioImpl<port, timer, bit>; \
    }

// Pin definitions for ATmega lineup

#if defined( __AVR_ATmega48P__ ) || defined( __AVR_ATmega88P__ ) || \
    defined( __AVR_ATmega168P__ ) || defined( __AVR_ATmega328P__ )

SetupGpio( 0, PortD, NoPwmChannel, 0 );
SetupGpio( 1, PortD, NoPwmChannel, 1 );
SetupGpio( 2, PortD, NoPwmChannel, 2 );
SetupGpio( 3, PortD, PwmChannel2B, 3 );
SetupGpio( 4, PortD, NoPwmChannel, 4 );
SetupGpio( 5, PortD, PwmChannel0B, 5 );
SetupGpio( 6, PortD, PwmChannel0A, 6 );
SetupGpio( 7, PortD, NoPwmChannel, 7 );
//
SetupGpio( 8, PortB, NoPwmChannel, 0 );
SetupGpio( 9, PortB, PwmChannel1A, 1 );
SetupGpio( 10, PortB, PwmChannel1B, 2 );
SetupGpio( 11, PortB, PwmChannel2A, 3 );
SetupGpio( 12, PortB, NoPwmChannel, 4 );
SetupGpio( 13, PortB, NoPwmChannel, 5 );
//
SetupGpio( 14, PortC, NoPwmChannel, 0 );
SetupGpio( 15, PortC, NoPwmChannel, 1 );
SetupGpio( 16, PortC, NoPwmChannel, 2 );
SetupGpio( 17, PortC, NoPwmChannel, 3 );
SetupGpio( 18, PortC, NoPwmChannel, 4 );
SetupGpio( 19, PortC, NoPwmChannel, 5 );
//
SetupGpio( 255, PortB, NoPwmChannel, 0 );

//
using SpiSCK = Gpio<PortB, 5>;
using SpiMISO = Gpio<PortB, 4>;
using SpiMOSI = Gpio<PortB, 3>;
using SpiSS = Gpio<PortB, 2>;

using UartSpi0XCK = Gpio<PortD, 4>;
using UartSpi0TX = Gpio<PortD, 1>;
using UartSpi0RX = Gpio<PortD, 0>;

#elif defined( __AVR_ATmega164P__ ) || defined( __AVR_ATmega324P__ ) || \
    defined( __AVR_ATmega644P__ ) || defined( __AVR_ATmega1284P__ )

SetupGpio( 0, PortB, NoPwmChannel, 0 );
SetupGpio( 1, PortB, NoPwmChannel, 1 );
SetupGpio( 2, PortB, NoPwmChannel, 2 );
SetupGpio( 3, PortB, PwmChannel0A, 3 );
SetupGpio( 4, PortB, PwmChannel0B, 4 );
SetupGpio( 5, PortB, NoPwmChannel, 5 );
SetupGpio( 6, PortB, NoPwmChannel, 6 );
SetupGpio( 7, PortB, NoPwmChannel, 7 );

SetupGpio( 8, PortD, NoPwmChannel, 0 );
SetupGpio( 9, PortD, NoPwmChannel, 1 );
SetupGpio( 10, PortD, NoPwmChannel, 2 );
SetupGpio( 11, PortD, NoPwmChannel, 3 );
SetupGpio( 12, PortD, PwmChannel1B, 4 );
SetupGpio( 13, PortD, PwmChannel1A, 5 );
SetupGpio( 14, PortD, PwmChannel2B, 6 );
SetupGpio( 15, PortD, PwmChannel2A, 7 );

SetupGpio( 16, PortC, NoPwmChannel, 0 );
SetupGpio( 17, PortC, NoPwmChannel, 1 );
SetupGpio( 18, PortC, NoPwmChannel, 2 );
SetupGpio( 19, PortC, NoPwmChannel, 3 );
SetupGpio( 20, PortC, NoPwmChannel, 4 );
SetupGpio( 21, PortC, NoPwmChannel, 5 );
SetupGpio( 22, PortC, NoPwmChannel, 6 );
SetupGpio( 23, PortC, NoPwmChannel, 7 );

SetupGpio( 24, PortA, NoPwmChannel, 0 );
SetupGpio( 25, PortA, NoPwmChannel, 1 );
SetupGpio( 26, PortA, NoPwmChannel, 2 );
SetupGpio( 27, PortA, NoPwmChannel, 3 );
SetupGpio( 28, PortA, NoPwmChannel, 4 );
SetupGpio( 29, PortA, NoPwmChannel, 5 );
SetupGpio( 30, PortA, NoPwmChannel, 6 );
SetupGpio( 31, PortA, NoPwmChannel, 7 );


SetupGpio( 255, PortB, NoPwmChannel, 0 );

using SpiSCK = Gpio<PortB, 7>;
using SpiMISO = Gpio<PortB, 6>;
using SpiMOSI = Gpio<PortB, 5>;
using SpiSS = Gpio<PortB, 4>;

using UartSpi0XCK = Gpio<PortB, 0>;
using UartSpi0TX = Gpio<PortD, 1>;
using UartSpi0RX = Gpio<PortD, 0>;

using UartSpi1XCK = Gpio<PortD, 4>;
using UartSpi1TX = Gpio<PortD, 3>;
using UartSpi1RX = Gpio<PortD, 2>;

#elif defined( __AVR_ATmega640__ ) || defined( __AVR_ATmega1280__ ) || \
    defined( __AVR_ATmega2560__ )

SetupGpio( 0, PortB, NoPwmChannel, 0 );
SetupGpio( 1, PortB, NoPwmChannel, 1 );
SetupGpio( 2, PortB, NoPwmChannel, 2 );
SetupGpio( 3, PortB, NoPwmChannel, 3 );
SetupGpio( 4, PortB, PwmChannel2A, 4 );
SetupGpio( 5, PortB, PwmChannel1A, 5 );
SetupGpio( 6, PortB, PwmChannel1B, 6 );
SetupGpio( 7, PortB, PwmChannel0A, 7 );

SetupGpio( 8, PortD, NoPwmChannel, 0 );
SetupGpio( 9, PortD, NoPwmChannel, 1 );
SetupGpio( 10, PortD, NoPwmChannel, 2 );
SetupGpio( 11, PortD, NoPwmChannel, 3 );
SetupGpio( 12, PortD, NoPwmChannel, 4 );
SetupGpio( 13, PortD, NoPwmChannel, 5 );
SetupGpio( 14, PortD, NoPwmChannel, 6 );
SetupGpio( 15, PortD, NoPwmChannel, 7 );

SetupGpio( 16, PortC, NoPwmChannel, 0 );
SetupGpio( 17, PortC, NoPwmChannel, 1 );
SetupGpio( 18, PortC, NoPwmChannel, 2 );
SetupGpio( 19, PortC, NoPwmChannel, 3 );
SetupGpio( 20, PortC, NoPwmChannel, 4 );
SetupGpio( 21, PortC, NoPwmChannel, 5 );
SetupGpio( 22, PortC, NoPwmChannel, 6 );
SetupGpio( 23, PortC, NoPwmChannel, 7 );

SetupGpio( 24, PortE, NoPwmChannel, 0 );
SetupGpio( 25, PortE, NoPwmChannel, 1 );
SetupGpio( 26, PortE, NoPwmChannel, 2 );
SetupGpio( 27, PortE, NoPwmChannel, 3 );
SetupGpio( 28, PortE, NoPwmChannel, 4 );
SetupGpio( 29, PortE, NoPwmChannel, 5 );
SetupGpio( 30, PortE, NoPwmChannel, 6 );
SetupGpio( 31, PortE, NoPwmChannel, 7 );

SetupGpio( 32, PortF, NoPwmChannel, 0 );
SetupGpio( 33, PortF, NoPwmChannel, 1 );
SetupGpio( 34, PortF, NoPwmChannel, 2 );
SetupGpio( 35, PortF, NoPwmChannel, 3 );
SetupGpio( 36, PortF, NoPwmChannel, 4 );
SetupGpio( 37, PortF, NoPwmChannel, 5 );
SetupGpio( 38, PortF, NoPwmChannel, 6 );
SetupGpio( 39, PortF, NoPwmChannel, 7 );

SetupGpio( 40, PortG, NoPwmChannel, 0 );
SetupGpio( 41, PortG, NoPwmChannel, 1 );
SetupGpio( 42, PortG, NoPwmChannel, 2 );
SetupGpio( 43, PortG, NoPwmChannel, 3 );
SetupGpio( 44, PortG, NoPwmChannel, 4 );
SetupGpio( 45, PortG, NoPwmChannel, 5 );
SetupGpio( 46, PortG, NoPwmChannel, 6 );
SetupGpio( 47, PortG, NoPwmChannel, 7 );

SetupGpio( 48, PortH, NoPwmChannel, 0 );
SetupGpio( 49, PortH, NoPwmChannel, 1 );
SetupGpio( 50, PortH, NoPwmChannel, 2 );
SetupGpio( 51, PortH, NoPwmChannel, 3 );
SetupGpio( 52, PortH, NoPwmChannel, 4 );
SetupGpio( 53, PortH, NoPwmChannel, 5 );
SetupGpio( 54, PortH, NoPwmChannel, 6 );
SetupGpio( 55, PortH, NoPwmChannel, 7 );

SetupGpio( 56, PortJ, NoPwmChannel, 0 );
SetupGpio( 57, PortJ, NoPwmChannel, 1 );
SetupGpio( 58, PortJ, NoPwmChannel, 2 );
SetupGpio( 59, PortJ, NoPwmChannel, 3 );
SetupGpio( 60, PortJ, NoPwmChannel, 4 );
SetupGpio( 61, PortJ, NoPwmChannel, 5 );
SetupGpio( 62, PortJ, NoPwmChannel, 6 );
SetupGpio( 63, PortJ, NoPwmChannel, 7 );

SetupGpio( 64, PortK, NoPwmChannel, 0 );
SetupGpio( 65, PortK, NoPwmChannel, 1 );
SetupGpio( 66, PortK, NoPwmChannel, 2 );
SetupGpio( 67, PortK, NoPwmChannel, 3 );
SetupGpio( 68, PortK, NoPwmChannel, 4 );
SetupGpio( 69, PortK, NoPwmChannel, 5 );
SetupGpio( 70, PortK, NoPwmChannel, 6 );
SetupGpio( 71, PortK, NoPwmChannel, 7 );

SetupGpio( 72, PortL, NoPwmChannel, 0 );
SetupGpio( 73, PortL, NoPwmChannel, 1 );
SetupGpio( 74, PortL, NoPwmChannel, 2 );
SetupGpio( 75, PortL, NoPwmChannel, 3 );
SetupGpio( 76, PortL, NoPwmChannel, 4 );
SetupGpio( 77, PortL, NoPwmChannel, 5 );
SetupGpio( 78, PortL, NoPwmChannel, 6 );
SetupGpio( 79, PortL, NoPwmChannel, 7 );

SetupGpio( 80, PortA, NoPwmChannel, 0 );
SetupGpio( 81, PortA, NoPwmChannel, 1 );
SetupGpio( 82, PortA, NoPwmChannel, 2 );
SetupGpio( 83, PortA, NoPwmChannel, 3 );
SetupGpio( 84, PortA, NoPwmChannel, 4 );
SetupGpio( 85, PortA, NoPwmChannel, 5 );
SetupGpio( 86, PortA, NoPwmChannel, 6 );
SetupGpio( 87, PortA, NoPwmChannel, 7 );

using SpiSS = Gpio<PortB, 0>;
using SpiSCK = Gpio<PortB, 1>;
using SpiMOSI = Gpio<PortB, 2>;
using SpiMISO = Gpio<PortB, 3>;

using UartSpi0XCK = Gpio<PortE, 2>;
using UartSpi0TX = Gpio<PortE, 1>;
using UartSpi0RX = Gpio<PortE, 0>;

using UartSpi1XCK = Gpio<PortD, 5>;
using UartSpi1TX = Gpio<PortD, 3>;
using UartSpi1RX = Gpio<PortD, 2>;

using UartSpi2XCK = Gpio<PortH, 2>;
using UartSpi2TX = Gpio<PortH, 1>;
using UartSpi2RX = Gpio<PortH, 0>;

using UartSpi3XCK = Gpio<PortJ, 2>;
using UartSpi3TX = Gpio<PortJ, 1>;
using UartSpi3RX = Gpio<PortJ, 0>;

#elif defined( __AVR_ATmega1281__ ) || defined( __AVR_ATmega2561__ )

SetupGpio( 0, PortB, NoPwmChannel, 0 );
SetupGpio( 1, PortB, NoPwmChannel, 1 );
SetupGpio( 2, PortB, NoPwmChannel, 2 );
SetupGpio( 3, PortB, NoPwmChannel, 3 );
SetupGpio( 4, PortB, PwmChannel2A, 4 );
SetupGpio( 5, PortB, PwmChannel1A, 5 );
SetupGpio( 6, PortB, PwmChannel1B, 6 );
SetupGpio( 7, PortB, PwmChannel0A, 7 );

SetupGpio( 8, PortD, NoPwmChannel, 0 );
SetupGpio( 9, PortD, NoPwmChannel, 1 );
SetupGpio( 10, PortD, NoPwmChannel, 2 );
SetupGpio( 11, PortD, NoPwmChannel, 3 );
SetupGpio( 12, PortD, NoPwmChannel, 4 );
SetupGpio( 13, PortD, NoPwmChannel, 5 );
SetupGpio( 14, PortD, NoPwmChannel, 6 );
SetupGpio( 15, PortD, NoPwmChannel, 7 );

SetupGpio( 16, PortC, NoPwmChannel, 0 );
SetupGpio( 17, PortC, NoPwmChannel, 1 );
SetupGpio( 18, PortC, NoPwmChannel, 2 );
SetupGpio( 19, PortC, NoPwmChannel, 3 );
SetupGpio( 20, PortC, NoPwmChannel, 4 );
SetupGpio( 21, PortC, NoPwmChannel, 5 );
SetupGpio( 22, PortC, NoPwmChannel, 6 );
SetupGpio( 23, PortC, NoPwmChannel, 7 );

SetupGpio( 24, PortE, NoPwmChannel, 0 );
SetupGpio( 25, PortE, NoPwmChannel, 1 );
SetupGpio( 26, PortE, NoPwmChannel, 2 );
SetupGpio( 27, PortE, NoPwmChannel, 3 );
SetupGpio( 28, PortE, NoPwmChannel, 4 );
SetupGpio( 29, PortE, NoPwmChannel, 5 );
SetupGpio( 30, PortE, NoPwmChannel, 6 );
SetupGpio( 31, PortE, NoPwmChannel, 7 );

SetupGpio( 32, PortF, NoPwmChannel, 0 );
SetupGpio( 33, PortF, NoPwmChannel, 1 );
SetupGpio( 34, PortF, NoPwmChannel, 2 );
SetupGpio( 35, PortF, NoPwmChannel, 3 );
SetupGpio( 36, PortF, NoPwmChannel, 4 );
SetupGpio( 37, PortF, NoPwmChannel, 5 );
SetupGpio( 38, PortF, NoPwmChannel, 6 );
SetupGpio( 39, PortF, NoPwmChannel, 7 );

SetupGpio( 40, PortG, NoPwmChannel, 0 );
SetupGpio( 41, PortG, NoPwmChannel, 1 );
SetupGpio( 42, PortG, NoPwmChannel, 2 );
SetupGpio( 43, PortG, NoPwmChannel, 3 );
SetupGpio( 44, PortG, NoPwmChannel, 4 );
SetupGpio( 45, PortG, NoPwmChannel, 5 );
SetupGpio( 46, PortG, NoPwmChannel, 6 );
SetupGpio( 47, PortG, NoPwmChannel, 7 );

SetupGpio( 48, PortA, NoPwmChannel, 0 );
SetupGpio( 49, PortA, NoPwmChannel, 1 );
SetupGpio( 50, PortA, NoPwmChannel, 2 );
SetupGpio( 51, PortA, NoPwmChannel, 3 );
SetupGpio( 52, PortA, NoPwmChannel, 4 );
SetupGpio( 53, PortA, NoPwmChannel, 5 );
SetupGpio( 54, PortA, NoPwmChannel, 6 );
SetupGpio( 55, PortA, NoPwmChannel, 7 );

using SpiSS = Gpio<PortB, 0>;
using SpiSCK = Gpio<PortB, 1>;
using SpiMOSI = Gpio<PortB, 2>;
using SpiMISO = Gpio<PortB, 3>;

using UartSpi0XCK = Gpio<PortE, 2>;
using UartSpi0TX = Gpio<PortE, 1>;
using UartSpi0RX = Gpio<PortE, 0>;

using UartSpi1XCK = Gpio<PortD, 5>;
using UartSpi1TX = Gpio<PortD, 3>;
using UartSpi1RX = Gpio<PortD, 2>;

#else

#error Unsupported MCU type

#endif

// Two specializations of the numbered pin template, one which clears the timer
// for each access to the PWM pins, as does the original Arduino wire lib,
// the other that does not (use with care!).
template <int n>
struct NumberedGpio {
    using Impl = typename NumberedGpioInternal<n>::Impl;
    static inline void High() __attribute__( ( always_inline ) )
    {
        Impl::High();
    }
    static inline void Low() __attribute__( ( always_inline ) ) { Impl::Low(); }
    static inline void Toggle() __attribute__( ( always_inline ) )
    {
        Impl::Toggle();
    }
    static inline void SetMode( uint8_t mode )
        __attribute__( ( always_inline ) )
    {
        Impl::SetMode( mode );
    }
    static inline void SetValue( uint8_t value )
        __attribute__( ( always_inline ) )
    {
        Impl::SetValue( value );
    }
    static inline void SetPwmValue( uint8_t value )
        __attribute__( ( always_inline ) )
    {
        Impl::SetPwmValue( value );
    }
    static inline uint8_t Value() __attribute__( ( always_inline ) )
    {
        return Impl::Value();
    }
};

template <int n>
struct PwmOutput {
    enum {
        buffer_size = 0,
        data_size = 8,
    };
    static void Init() { NumberedGpio<n>::SetMode( PWM_OUTPUT ); }
    static void Write( uint8_t value )
    {
        return NumberedGpio<n>::SetPwmValue( value );
    }
    static void Stop() { NumberedGpio<n>::Impl::Pwm::Stop(); }
    static void Start() { NumberedGpio<n>::Impl::Pwm::Start(); }
};

}  // namespace avril

#endif  // AVRIL_GPIO_H_
