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

#![cfg_attr(not(test), no_std)]
#![deny(missing_docs)]

pub mod i2c;
pub mod serial;
pub mod spi;
