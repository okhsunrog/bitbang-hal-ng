//! Delay providers for driving the bit-bang timing.
//!
//! Every bus in this crate is timed by an [`embedded_hal::delay::DelayNs`],
//! and the choice of provider decides the *actual* bus speed:
//!
//! ```text
//! f_actual ≈ 1 / (2 × (delay resolution + GPIO overhead))
//! ```
//!
//! A provider with coarse resolution silently caps the frequency far below
//! the configured target. The classic trap is `embassy_time::Delay`: it
//! rounds every wait up to a timer tick (30.5 µs at the common
//! `tick-hz-32_768`), so a bus configured for 500 kHz ends up running at
//! ~16 kHz. Timer-queue based delays are for milliseconds, not bus
//! half-periods.
//!
//! What to use instead:
//!
//! - [`AsmDelay`] (feature `cortex-m`) — busy-waits on CPU cycles, precise
//!   to tens of nanoseconds. The right default on Cortex-M targets.
//! - [`NoopDelay`] — no waiting at all: the bus runs as fast as the GPIO
//!   calls allow. Fine whenever the slave's maximum clock comfortably
//!   exceeds what bit-banging can reach.
//! - A HAL/board delay that busy-waits (e.g. one driving SysT/DWT
//!   directly), if it offers sub-microsecond resolution.

use embedded_hal::delay::DelayNs;

/// A [`DelayNs`] that does not wait at all.
///
/// Every bus half-period collapses to the GPIO and loop overhead — the bus
/// runs at the maximum speed bit-banging can reach on the platform
/// (typically hundreds of kHz to a few MHz). Use it when the slave's
/// maximum clock is comfortably above that; the configured frequency is
/// then only a label.
///
/// Not to be confused with `embedded_hal_bus::spi::NoDelay`, which *panics*
/// when used — this one is a genuine zero-cost no-op.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoopDelay;

impl NoopDelay {
    /// Create a no-op delay provider.
    pub const fn new() -> Self {
        Self
    }
}

impl DelayNs for NoopDelay {
    #[inline]
    fn delay_ns(&mut self, _ns: u32) {}
}

/// A [`DelayNs`] that busy-waits on CPU cycles via [`cortex_m::asm::delay`]
/// (feature `cortex-m`).
///
/// Precise to tens of nanoseconds and needs no hardware timer, which makes
/// it the right provider for bus half-periods on Cortex-M targets. Waits
/// are rounded up, and `cortex_m::asm::delay` itself guarantees *at least*
/// the requested cycles — flash wait states and interrupts only stretch
/// them, so a bus never runs faster than configured.
///
/// ```rust,ignore
/// use bitbang_hal_ng::delay::AsmDelay;
///
/// // STM32F405 at 168 MHz
/// let delay = AsmDelay::new(168_000_000);
/// ```
#[cfg(feature = "cortex-m")]
#[derive(Debug, Clone, Copy)]
pub struct AsmDelay {
    cpu_hz: u32,
}

#[cfg(feature = "cortex-m")]
impl AsmDelay {
    /// Create a delay provider for a core running at `cpu_hz`.
    pub const fn new(cpu_hz: u32) -> Self {
        Self { cpu_hz }
    }
}

#[cfg(feature = "cortex-m")]
impl DelayNs for AsmDelay {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        let cycles = (u64::from(ns) * u64::from(self.cpu_hz)).div_ceil(1_000_000_000);
        cortex_m::asm::delay(cycles as u32);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noop_delay_is_usable_as_delay_ns() {
        fn takes_delay(d: &mut impl DelayNs) {
            d.delay_ns(1_000);
            d.delay_us(1);
            d.delay_ms(1);
        }
        takes_delay(&mut NoopDelay::new());
    }
}
