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
//
// Fast serial (for the onboard UART), using compile time optimizations.
//
// Can be used in buffered mode, or in polled mode (to save space or avoid
// interruptions if there is an even higher priority task than the serial I/O).
//
// Usage:
//
// In your code, include serial.h:
// #include "avril/io/serial.h"
//
// Classes will be created according to the capabilities of the -mmcu= parameter
// given to the compiler.
//
// If any serial device is used in a buffered mode, you will need to invoke
// the convenience macros in your main.cpp to create the needed ISRs:
//
// CREATE_USARTn_ISRS()
//
// Set-up:
//
// create a typedef or alias for the device class:
// using Serial = avril::Serial<avril::SerialPort0, 31250, avril::BUFFERED,
// avril::POLLED>;
//
// then, in your setup code, call Serial::Init()
//
// Write:
// Serial::Write(40)  // Will block until data is written.
// write_has_succeeded = Serial::NonBlockingWrite(40)  // Will not block.
//
// Buffer manipulation (for buffered I/O):
// Serial::Readable()  // Number of bytes ready to be read. For polled read too
// my_value = Serial::Read()  // Will wait until data arrives.
// my_value = Serial::NonBlockingRead()  // Will return -1 if no data is there.
// my_value = Serial::ImmediateRead()  // Assumes you are sure about what you
//   are doing and you know that data is here.
//
// Flushing a buffer:
// Serial::InputBuffer::Flush()
//
//
#ifndef AVRIL_SERIAL_H_
#define AVRIL_SERIAL_H_

#include <avr/interrupt.h>

#include "../avril.h"
#include "gpio.h"
#include "ring_buffer.h"


namespace avril {

const uint8_t kSerialOutputBufferSize = 32;
const uint8_t kSerialInputBufferSize = 32;

// Low-level interface to the low-level UART registers. Several specializations
// may be declared for each serial port. This class could theoretically be used
// for non-blocking write or polling reads.
template <typename TxEnableBit, typename TxReadyBit, typename TxInterruptBit,
          typename RxEnableBit, typename RxReadyBit, typename RxInterruptBit,
          typename TurboBit, typename PrescalerRegisterH,
          typename PrescalerRegisterL, typename DataRegister,
          uint8_t input_buffer_size_, uint8_t output_buffer_size_>
struct SerialPort {
    using Tx = TxEnableBit;
    using Rx = RxEnableBit;
    using TxInterrupt = TxInterruptBit;
    using RxInterrupt = RxInterruptBit;
    using Turbo = TurboBit;
    enum {
        input_buffer_size = input_buffer_size_,
        output_buffer_size = output_buffer_size_
    };
    static inline void SetPrescaler( uint16_t value )
    {
        *PrescalerRegisterH::ptr() = value >> 8;
        *PrescalerRegisterL::ptr() = value;
    }
    static inline uint8_t TxReady() { return TxReadyBit::Value(); }
    static inline uint8_t RxReady() { return RxReadyBit::Value(); }
    static inline uint8_t Data() { return *DataRegister::ptr(); }
    static inline void SetData( uint8_t value )
    {
        *DataRegister::ptr() = value;
    }
};

template <typename SerialPort>
struct SerialInput : public Input {
    enum { buffer_size = SerialPort::input_buffer_size, data_size = 8 };
    typedef uint8_t Value;

    // Blocking!
    static inline Value Read()
    {
        while ( !Readable() ) {
        }
        return ImmediateRead();
    }

    // Number of bytes available for read.
    static inline uint8_t Readable() { return SerialPort::RxReady(); }
    // A byte, or -1 if reading failed.
    static inline int16_t NonBlockingRead() { return Readable() ? Read() : -1; }
    // No check for ready state.
    static inline Value ImmediateRead() { return SerialPort::Data(); }
    // Called in data reception interrupt.
    static inline void Received()
    {
        if ( !Readable() ) {
            return;
        }
        // This will discard data if the buffer is full.
        RingBuffer<SerialInput<SerialPort>>::NonBlockingWrite(
            ImmediateRead() );
    }
};


template <typename SerialPort>
struct SerialOutput : public Output {
    enum { buffer_size = SerialPort::output_buffer_size, data_size = 8 };
    typedef uint8_t Value;

    // Blocking!
    static inline void Write( Value v )
    {
        while ( !Writable() ) {
        }
        Overwrite( v );
    }

    // Number of bytes that can be fed.
    static inline uint8_t Writable() { return SerialPort::TxReady(); }
    // 1 if success.
    static inline bool NonBlockingWrite( Value v )
    {
        if ( !Writable() ) {
            return false;
        }
        Overwrite( v );
        return true;
    }

    // No check for ready state.
    static inline void Overwrite( Value v ) { SerialPort::SetData( v ); }
    // Called in data emission interrupt.
    static inline Value Requested()
    {
        auto v = RingBuffer<SerialOutput<SerialPort>>::NonBlockingRead();
        if ( v != -1 ) {
            Overwrite( v );
        }
        return v;
    }
};



template <typename SerialPort, PortMode input = POLLED,
          PortMode output = POLLED>
struct SerialImplementation {
};

template <typename SerialPort>
struct SerialImplementation<SerialPort, DISABLED, DISABLED> {
    typedef InputOutput<DisabledInput, DisabledOutput> IO;
};
template <typename SerialPort>
struct SerialImplementation<SerialPort, DISABLED, POLLED> {
    typedef InputOutput<DisabledInput, SerialOutput<SerialPort>> IO;
};
template <typename SerialPort>
struct SerialImplementation<SerialPort, DISABLED, BUFFERED> {
    typedef RingBuffer<SerialOutput<SerialPort>> OutputBuffer;
    typedef InputOutput<DisabledInput, OutputBuffer> IO;
};
template <typename SerialPort>
struct SerialImplementation<SerialPort, POLLED, DISABLED> {
    typedef InputOutput<SerialInput<SerialPort>, DisabledOutput> IO;
};
template <typename SerialPort>
struct SerialImplementation<SerialPort, POLLED, POLLED> {
    typedef InputOutput<SerialInput<SerialPort>, SerialOutput<SerialPort>> IO;
};
template <typename SerialPort>
struct SerialImplementation<SerialPort, POLLED, BUFFERED> {
    typedef RingBuffer<SerialOutput<SerialPort>> OutputBuffer;
    typedef InputOutput<SerialInput<SerialPort>, OutputBuffer> IO;
};
template <typename SerialPort>
struct SerialImplementation<SerialPort, BUFFERED, DISABLED> {
    typedef RingBuffer<SerialInput<SerialPort>> InputBuffer;
    typedef InputOutput<InputBuffer, DisabledOutput> IO;
};
template <typename SerialPort>
struct SerialImplementation<SerialPort, BUFFERED, POLLED> {
    typedef RingBuffer<SerialInput<SerialPort>> InputBuffer;
    typedef InputOutput<InputBuffer, SerialOutput<SerialPort>> IO;
};
template <typename SerialPort>
struct SerialImplementation<SerialPort, BUFFERED, BUFFERED> {
    typedef RingBuffer<SerialInput<SerialPort>> InputBuffer;
    typedef RingBuffer<SerialOutput<SerialPort>> OutputBuffer;
    typedef InputOutput<InputBuffer, OutputBuffer> IO;
};

template <typename SerialPort, uint32_t baud_rate, PortMode input = POLLED,
          PortMode output = POLLED, bool turbo = false>
struct Serial {
    using Impl = SerialImplementation<SerialPort, input, output>;
    using Value = uint8_t;
    using Input = typename Impl::IO::Input;
    using Output = typename Impl::IO::Output;
    static inline void Init() { Init<baud_rate>(); }
    template <uint32_t new_baud_rate>
    static inline void Init()
    {
        if ( turbo ) {
            SerialPort::Turbo::Set();
            uint16_t prescaler = F_CPU / ( 8L * baud_rate ) - 1;
            SerialPort::SetPrescaler( prescaler );
        }
        else {
            SerialPort::Turbo::Clear();
            uint16_t prescaler = F_CPU / ( 16 * baud_rate ) - 1;
            SerialPort::SetPrescaler( prescaler );
        }
        if ( output != DISABLED ) {
            SerialPort::Tx::Set();
        }
        if ( input != DISABLED ) {
            SerialPort::Rx::Set();
        }
        if ( input == BUFFERED ) {
            SerialPort::RxInterrupt::Set();
        }
        if ( output == BUFFERED ) {
            SerialPort::TxInterrupt::Set();
        }
    }

    static inline void Disable()
    {
        SerialPort::Tx::Clear();
        SerialPort::Rx::Clear();
        SerialPort::RxInterrupt::Clear();
        SerialPort::TxInterrupt::Clear();
    }

    static inline void Write( Value v ) { Impl::IO::Write( v ); }
    static inline uint8_t Writable() { return Impl::IO::Writable(); }
    static inline bool NonBlockingWrite( Value v )
    {
        return Impl::IO::NonBlockingWrite( v );
    }
    static inline void Overwrite( Value v ) { Impl::IO::Overwrite( v ); }
    static inline Value Read() { return Impl::IO::Read(); }
    static inline uint8_t Readable() { return Impl::IO::Readable(); }
    static inline int16_t NonBlockingRead()
    {
        return Impl::IO::NonBlockingRead();
    }
    static inline Value ImmediateRead() { return Impl::IO::ImmediateRead(); }
};

#if defined( UCSRA )
namespace Register {
IORegister8( UBRRH );
IORegister8( UBRRL );
IORegister16( UBRR );
IORegister8( UCSRA );
IORegister8( UCSRB );
IORegister8( UCSRC );
IORegister8( UDR );
} // namespace Register

using SerialPort0 = SerialPort<BitInRegister<Register::_UCSRB, TXEN>,  /**/
                               BitInRegister<Register::_UCSRA, UDRE>,  /**/
                               BitInRegister<Register::_UCSRB, UDRIE>, /**/
                               BitInRegister<Register::_UCSRB, RXEN>,  /**/
                               BitInRegister<Register::_UCSRA, RXC>,   /**/
                               BitInRegister<Register::_UCSRB, RXCIE>, /**/
                               BitInRegister<Register::_UCSRA, U2X>,   /**/
                               Register::_UBRRH,                        /**/
                               Register::_UBRRL,                        /**/
                               Register::_UDR,                          /**/
                               kSerialOutputBufferSize,                  /**/
                               kSerialInputBufferSize>;                  /**/
#elif defined( UCSR0A )
namespace Register {
IORegister8( UBRR0H );
IORegister8( UBRR0L );
IORegister16( UBRR0 );
IORegister8( UCSR0A );
IORegister8( UCSR0B );
IORegister8( UCSR0C );
IORegister8( UDR0 );
} // namespace Register


using SerialPort0 = SerialPort<BitInRegister<Register::_UCSR0B, TXEN0>,  /**/
                               BitInRegister<Register::_UCSR0A, UDRE0>,  /**/
                               BitInRegister<Register::_UCSR0B, UDRIE0>, /**/
                               BitInRegister<Register::_UCSR0B, RXEN0>,  /**/
                               BitInRegister<Register::_UCSR0A, RXC0>,   /**/
                               BitInRegister<Register::_UCSR0B, RXCIE0>, /**/
                               BitInRegister<Register::_UCSR0A, U2X0>,   /**/
                               Register::_UBRR0H,                        /**/
                               Register::_UBRR0L,                        /**/
                               Register::_UDR0,                          /**/
                               kSerialOutputBufferSize,                  /**/
                               kSerialInputBufferSize>;                  /**/
#else
#error "target MCU has no USART"
#endif

#if defined( UCSR1A )
namespace Register {
IORegister8( UBRR0H );
IORegister8( UBRR1H );
IORegister8( UBRR1L );
IORegister16( UBRR1 );
IORegister8( UCSR1A );
IORegister8( UCSR1B );
IORegister8( UCSR1C );
IORegister8( UDR1 );
} // namespace Register


using SerialPort1 = SerialPort<BitInRegister<Register::_UCSR1B, TXEN1>,  /**/
                               BitInRegister<Register::_UCSR1A, UDRE1>,  /**/
                               BitInRegister<Register::_UCSR1B, UDRIE1>, /**/
                               BitInRegister<Register::_UCSR1B, RXEN1>,  /**/
                               BitInRegister<Register::_UCSR1A, RXC1>,   /**/
                               BitInRegister<Register::_UCSR1B, RXCIE1>, /**/
                               BitInRegister<Register::_UCSR1A, U2X1>,   /**/
                               Register::_UBRR1H,                        /**/
                               Register::_UBRR1L,                        /**/
                               Register::_UDR1,                          /**/
                               kSerialOutputBufferSize,                  /**/
                               kSerialInputBufferSize>;                  /**/
#endif

#if defined( UCSR2A )
namespace Register {
IORegister8( UBRR0H );
IORegister8( UBRR2H );
IORegister8( UBRR2L );
IORegister16( UBRR2 );
IORegister8( UCSR2A );
IORegister8( UCSR2B );
IORegister8( UCSR2C );
IORegister8( UDR2 );
} // namespace Register


using SerialPort2 = SerialPort<BitInRegister<Register::_UCSR2B, TXEN2>,  /**/
                               BitInRegister<Register::_UCSR2A, UDRE2>,  /**/
                               BitInRegister<Register::_UCSR2B, UDRIE2>, /**/
                               BitInRegister<Register::_UCSR2B, RXEN2>,  /**/
                               BitInRegister<Register::_UCSR2A, RXC2>,   /**/
                               BitInRegister<Register::_UCSR2B, RXCIE2>, /**/
                               BitInRegister<Register::_UCSR2A, U2X2>,   /**/
                               Register::_UBRR2H,                        /**/
                               Register::_UBRR2L,                        /**/
                               Register::_UDR2,                          /**/
                               kSerialOutputBufferSize,                  /**/
                               kSerialInputBufferSize>;                  /**/
#endif


#if defined( UCSR3A )
namespace Register {
IORegister8( UBRR0H );
IORegister8( UBRR3H );
IORegister8( UBRR3L );
IORegister16( UBRR3 );
IORegister8( UCSR3A );
IORegister8( UCSR3B );
IORegister8( UCSR3C );
IORegister8( UDR3 );
} // namespace Register


using SerialPort3 = SerialPort<BitInRegister<Register::_UCSR3B, TXEN3>,  /**/
                               BitInRegister<Register::_UCSR3A, UDRE3>,  /**/
                               BitInRegister<Register::_UCSR3B, UDRIE3>, /**/
                               BitInRegister<Register::_UCSR3B, RXEN3>,  /**/
                               BitInRegister<Register::_UCSR3A, RXC3>,   /**/
                               BitInRegister<Register::_UCSR3B, RXCIE3>, /**/
                               BitInRegister<Register::_UCSR3A, U2X3>,   /**/
                               Register::_UBRR3H,                        /**/
                               Register::_UBRR3L,                        /**/
                               Register::_UDR3,                          /**/
                               kSerialOutputBufferSize,                  /**/
                               kSerialInputBufferSize>;                  /**/
#endif


// Macros to define or instantiate ISRs

#if defined( USART_RX_vect )
#define CREATE_USART0_ISRS()                                      \
    ISR( USART_RX_vect )                                \
    {                                                             \
        ::avril::SerialInput<::avril::SerialPort0>::Received();   \
    }                                                             \
    ISR( USART_UDRE_vect )                               \
    {                                                             \
        ::avril::SerialOutput<::avril::SerialPort0>::Requested(); \
    }
#elif defined( USART0_RX_vect )
#define CREATE_USART0_ISRS()                                      \
    ISR( USART0_RX_vect )                                \
    {                                                             \
        ::avril::SerialInput<::avril::SerialPort0>::Received();   \
    }                                                             \
    ISR( USART0_UDRE_vect )                               \
    {                                                             \
        ::avril::SerialOutput<::avril::SerialPort0>::Requested(); \
    }
#else
#define CREATE_USART0_ISRS()
#endif

#if defined( USART1_RX_vect )
#define CREATE_USART1_ISRS()                                      \
    ISR( USART1_RX_vect )                                \
    {                                                             \
        ::avril::SerialInput<::avril::SerialPort1>::Received();   \
    }                                                             \
    ISR( USART1_UDRE_vect )                               \
    {                                                             \
        ::avril::SerialOutput<::avril::SerialPort1>::Requested(); \
    }
#else
#define CREATE_USART1_ISRS()
#endif

#if defined( USART2_RX_vect )
#define CREATE_USART2_ISRS()                                      \
    ISR( USART2_RX_vect )                                \
    {                                                             \
        ::avril::SerialInput<::avril::SerialPort2>::Received();   \
    }                                                             \
    ISR( USART2_UDRE_vect )                               \
    {                                                             \
        ::avril::SerialOutput<::avril::SerialPort2>::Requested(); \
    }
#else
#define CREATE_USART2_ISRS()
#endif

#if defined( USART3_RX_vect )
#define CREATE_USART3_ISRS()                                      \
    ISR( USART3_RX_vect )                                \
    {                                                             \
        ::avril::SerialInput<::avril::SerialPort3>::Received();   \
    }                                                             \
    ISR( USART3_UDRE_vect )                               \
    {                                                             \
        ::avril::SerialOutput<::avril::SerialPort3>::Requested(); \
    }
#else
#define CREATE_USART3_ISRS()
#endif



}  // namespace avril


#endif  // AVRIL_SERIAL_H_
