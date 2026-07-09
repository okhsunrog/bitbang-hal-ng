//! This is a [bit banging] implementation of the [`embedded-hal`] 1.0 traits.
//!
//! [bit banging]: https://en.wikipedia.org/wiki/Bit_banging
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//! ## Provided implementations
//!
//! - [`spi::SPI`] — full-duplex SPI master implementing [`embedded_hal::spi::SpiBus`].
//!   All four SPI modes and both bit orders are supported. Wrap it in
//!   `embedded_hal_bus::spi::ExclusiveDevice` to get an
//!   [`embedded_hal::spi::SpiDevice`] with managed CS.
//! - [`i2c::I2cBB`] — I2C master implementing [`embedded_hal::i2c::I2c`]
//!   (no clock stretching support).
//! - [`serial::Serial`] — 8-N-1 UART. With the `embedded-io` feature it
//!   implements [`embedded_io::Read`] and [`embedded_io::Write`].
//!
//! All implementations are timed with an [`embedded_hal::delay::DelayNs`]
//! provider; the target frequency / baud rate is set at construction time.
//!
//! ## Choosing a delay provider
//!
//! The delay provider decides the *actual* bus speed:
//!
//! ```text
//! f_actual ≈ 1 / (2 × (delay resolution + GPIO overhead))
//! ```
//!
//! Timer-queue delays with coarse ticks are a trap: `embassy_time::Delay`
//! at the common `tick-hz-32_768` rounds every half-period up to 30.5 µs
//! and silently drags a 500 kHz bus to ~16 kHz. Use a busy-wait provider
//! instead — [`delay::AsmDelay`] (feature `cortex-m`) counts CPU cycles,
//! [`delay::NoopDelay`] runs the bus at raw GPIO speed. See [`delay`] for
//! the full guidance.

#![cfg_attr(not(test), no_std)]
#![deny(missing_docs)]

pub mod delay;
pub mod i2c;
pub mod serial;
pub mod spi;
