//! Serial Peripheral Interface
//!
//! This implementation consumes the following hardware resources:
//! - A delay provider (implements [`embedded_hal::delay::DelayNs`]) for timing clock cycles
//! - Output GPIO pin for clock signal (SCLK)
//! - Output GPIO pin for data transmission (Master Output Slave Input - MOSI)
//! - Input GPIO pin for data reception (Master Input Slave Output - MISO)
//!
//! SS/CS (slave select) must be handled independently. To get an
//! [`embedded_hal::spi::SpiDevice`] (manages CS for you), wrap this bus with
//! `embedded_hal_bus::spi::ExclusiveDevice`.
//!
//! All four SPI modes and MSB-first/LSB-first bit orders are supported.
//!
//! ## Example
//!
//! ```rust,ignore
//! use embedded_hal::spi::MODE_1;
//! use bitbang_hal_ng::spi::{SPI, SpiConfig};
//!
//! let config = SpiConfig::new(MODE_1).with_frequency_hz(500_000);
//! let spi = SPI::new(miso, mosi, sck, delay, config);
//! ```

use core::cmp::max;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    spi::{ErrorType, MODE_0, MODE_1, MODE_2, MODE_3, Mode, Polarity, SpiBus},
};

/// Error type
#[derive(Debug)]
pub enum Error<E> {
    /// Communication error
    Bus(E),
}

impl<E: core::fmt::Debug> embedded_hal::spi::Error for Error<E> {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match self {
            Error::Bus(_) => embedded_hal::spi::ErrorKind::Other,
        }
    }
}

/// Transmission bit order
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum BitOrder {
    /// Most significant bit first
    #[default]
    MSBFirst,
    /// Least significant bit first
    LSBFirst,
}

/// Configuration of the SPI interface.
///
/// Built with [`SpiConfig::new`] and the `with_*` builder methods:
///
/// ```rust
/// use embedded_hal::spi::MODE_1;
/// use bitbang_hal_ng::spi::{SpiConfig, BitOrder};
///
/// let config = SpiConfig::new(MODE_1)
///     .with_frequency_hz(500_000)
///     .with_bit_order(BitOrder::MSBFirst);
/// ```
#[derive(Debug, Clone)]
pub struct SpiConfig {
    mode: Mode,
    bit_order: BitOrder,
    /// Value clocked out on MOSI when the bus is reading more bytes than
    /// were provided for writing. Usually `0x00`.
    empty_write_value: u8,
    /// Half of the SCLK period. The resulting clock frequency is
    /// `f = 1_000_000_000 / (2 * half_period_duration_ns)` (plus GPIO overhead).
    half_period_duration_ns: u32,
}

impl Default for SpiConfig {
    /// Mode 0, MSB first, 100 kHz clock
    fn default() -> Self {
        Self::new(MODE_0)
    }
}

impl SpiConfig {
    /// Create a configuration with the given SPI mode, MSB-first bit order
    /// and a 100 kHz clock.
    pub fn new(mode: Mode) -> Self {
        Self {
            mode,
            bit_order: BitOrder::default(),
            empty_write_value: 0x00,
            half_period_duration_ns: 5_000, // 100 kHz
        }
    }

    /// Set the transmission bit order.
    pub fn with_bit_order(mut self, bit_order: BitOrder) -> Self {
        self.bit_order = bit_order;
        self
    }

    /// Set the value clocked out on MOSI when reading without write data.
    pub fn with_empty_write_value(mut self, value: u8) -> Self {
        self.empty_write_value = value;
        self
    }

    /// Set the target clock frequency in Hz.
    ///
    /// The actual frequency will be lower due to GPIO and delay overhead.
    pub fn with_frequency_hz(mut self, frequency_hz: u32) -> Self {
        self.half_period_duration_ns = 1_000_000_000 / (2 * frequency_hz);
        self
    }

    /// Set the half period of the clock signal in nanoseconds directly.
    pub fn with_half_period_ns(mut self, half_period_ns: u32) -> Self {
        self.half_period_duration_ns = half_period_ns;
        self
    }
}

/// A Full-Duplex SPI implementation, takes 3 pins and a delay provider.
pub struct SPI<Miso, Mosi, Sck, Delay>
where
    Miso: InputPin,
    Mosi: OutputPin,
    Sck: OutputPin,
    Delay: DelayNs,
{
    miso: Miso,
    mosi: Mosi,
    sck: Sck,
    delay: Delay,
    config: SpiConfig,
}

impl<Miso, Mosi, Sck, Delay, E> SPI<Miso, Mosi, Sck, Delay>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Delay: DelayNs,
{
    /// Create an instance. The clock pin is immediately driven to its idle
    /// level according to the configured mode's polarity.
    pub fn new(miso: Miso, mosi: Mosi, sck: Sck, delay: Delay, config: SpiConfig) -> Self {
        let mut spi = SPI {
            miso,
            mosi,
            sck,
            delay,
            config,
        };

        match spi.config.mode.polarity {
            Polarity::IdleLow => spi.sck.set_low(),
            Polarity::IdleHigh => spi.sck.set_high(),
        }
        .unwrap_or(());

        spi
    }

    /// Release the pins and delay provider.
    pub fn free(self) -> (Miso, Mosi, Sck, Delay) {
        (self.miso, self.mosi, self.sck, self.delay)
    }

    fn read_bit(&mut self, read_val: &mut u8) -> Result<(), Error<E>> {
        let is_miso_high = self.miso.is_high().map_err(Error::Bus)?;
        let shifted_value = *read_val << 1;
        if is_miso_high {
            *read_val = shifted_value | 1;
        } else {
            *read_val = shifted_value;
        }
        Ok(())
    }

    #[inline]
    fn set_clk_high(&mut self) -> Result<(), Error<E>> {
        self.sck.set_high().map_err(Error::Bus)
    }

    #[inline]
    fn set_clk_low(&mut self) -> Result<(), Error<E>> {
        self.sck.set_low().map_err(Error::Bus)
    }

    #[inline]
    fn wait_for_delay(&mut self) {
        self.delay.delay_ns(self.config.half_period_duration_ns);
    }

    #[inline]
    fn rw_byte(&mut self, clock_out: u8, read_in: &mut u8) -> Result<(), Error<E>> {
        for bit_offset in 0..8 {
            let out_bit = match self.config.bit_order {
                BitOrder::MSBFirst => (clock_out >> (7 - bit_offset)) & 0b1,
                BitOrder::LSBFirst => (clock_out >> bit_offset) & 0b1,
            };

            if out_bit == 1 {
                self.mosi.set_high().map_err(Error::Bus)?;
            } else {
                self.mosi.set_low().map_err(Error::Bus)?;
            }

            match self.config.mode {
                MODE_0 => {
                    self.wait_for_delay();
                    self.set_clk_high()?;
                    self.read_bit(read_in)?;
                    self.wait_for_delay();
                    self.set_clk_low()?;
                }
                MODE_1 => {
                    self.set_clk_high()?;
                    self.wait_for_delay();
                    self.read_bit(read_in)?;
                    self.set_clk_low()?;
                    self.wait_for_delay();
                }
                MODE_2 => {
                    self.wait_for_delay();
                    self.set_clk_low()?;
                    self.read_bit(read_in)?;
                    self.wait_for_delay();
                    self.set_clk_high()?;
                }
                MODE_3 => {
                    self.set_clk_low()?;
                    self.wait_for_delay();
                    self.read_bit(read_in)?;
                    self.set_clk_high()?;
                    self.wait_for_delay();
                }
            };
        }
        Ok(())
    }
}

impl<Miso, Mosi, Sck, Delay, E> ErrorType for SPI<Miso, Mosi, Sck, Delay>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    type Error = Error<E>;
}

impl<Miso, Mosi, Sck, Delay, E> SpiBus<u8> for SPI<Miso, Mosi, Sck, Delay>
where
    Miso: InputPin<Error = E>,
    Mosi: OutputPin<Error = E>,
    Sck: OutputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    #[inline]
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            self.rw_byte(self.config.empty_write_value, word)?;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut ignored_read = 0u8;
        for byte in words {
            self.rw_byte(*byte, &mut ignored_read)?;
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut ignored_read = 0u8;
        for i in 0..max(read.len(), write.len()) {
            let read_in_byte = read.get_mut(i).unwrap_or(&mut ignored_read);
            let clock_out_byte = write
                .get(i)
                .copied()
                .unwrap_or(self.config.empty_write_value);
            self.rw_byte(clock_out_byte, read_in_byte)?;
        }

        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut current_read_byte = 0u8;
        for clock_out_byte in words {
            self.rw_byte(*clock_out_byte, &mut current_read_byte)?;
            *clock_out_byte = current_read_byte;
        }

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // Every operation is performed synchronously bit by bit. Nothing to do here.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::spi::MODE_0;
    use embedded_hal_mock::eh1::delay::NoopDelay as MockDelay;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };
    use std::vec::Vec;

    fn waveform(string: &str) -> Vec<PinState> {
        let mut transactions = Vec::new();
        let mut last_state = None;
        let mut last_action = string.chars().next().unwrap();
        for step in string.chars() {
            let step = if step == '.' { last_action } else { step };
            match step {
                '0' => {
                    transactions.push(PinState::Low);
                    last_state = Some(PinState::Low);
                }
                '1' => {
                    transactions.push(PinState::High);
                    last_state = Some(PinState::High);
                }
                'p' | 'P' => {
                    let next_state = if last_state == Some(PinState::Low) {
                        PinState::High
                    } else {
                        PinState::Low
                    };
                    transactions.push(next_state);
                    last_state = Some(next_state);
                }
                'n' | 'N' => {
                    let next_state = if last_state == Some(PinState::High) {
                        PinState::Low
                    } else {
                        PinState::High
                    };
                    transactions.push(next_state);
                    last_state = Some(next_state);
                }
                _ => panic!("Invalid binary literal"),
            };
            last_action = step;
        }
        transactions
    }

    #[test]
    fn test_states() {
        let res = waveform("p..");
        assert_eq!(res, vec![PinState::Low, PinState::High, PinState::Low]);

        let res = waveform("P..");
        assert_eq!(res, vec![PinState::Low, PinState::High, PinState::Low]);

        let res = waveform("n..");
        assert_eq!(res, vec![PinState::High, PinState::Low, PinState::High]);

        let res = waveform("N..");
        assert_eq!(res, vec![PinState::High, PinState::Low, PinState::High]);

        let res = waveform("n.0");
        assert_eq!(res, vec![PinState::High, PinState::Low, PinState::Low]);
    }

    fn input_waveform(string: &str) -> Vec<PinTransaction> {
        waveform(string)
            .into_iter()
            .map(PinTransaction::get)
            .collect()
    }

    fn output_waveform(string: &str) -> Vec<PinTransaction> {
        waveform(string)
            .into_iter()
            .map(PinTransaction::set)
            .collect()
    }

    #[test]
    fn test_spi_read_single_byte() {
        let miso = PinMock::new(&input_waveform("10101010"));
        // write default value (0x00) to mosi
        let mosi = PinMock::new(&output_waveform("00000000"));
        let sck = PinMock::new(&output_waveform("01010101010101010"));
        let delay = MockDelay::new();

        let mut spi = SPI::new(miso, mosi, sck, delay, SpiConfig::default());
        let mut data = [0x00];
        spi.read(&mut data).expect("SPI read failed");

        spi.mosi.done();
        spi.miso.done();
        spi.sck.done();
        assert_eq!(data[0], 0b10101010);
    }

    #[test]
    fn test_spi_write_single_byte() {
        // this is ignored when reading
        let miso = PinMock::new(&input_waveform("00000000"));
        let mosi = PinMock::new(&output_waveform("01010101"));
        let sck = PinMock::new(&output_waveform("01010101010101010"));
        let delay = MockDelay::new();

        let mut spi = SPI::new(miso, mosi, sck, delay, SpiConfig::default());
        let data = [0b01010101];
        spi.write(&data).expect("SPI write failed");

        // Verify that all transactions were completed
        spi.mosi.done();
        spi.sck.done();
        spi.miso.done();
    }

    #[test]
    fn test_spi_mode1_transfer() {
        // MODE_1: clock idles low, data is shifted out on the rising edge
        // and sampled on the falling edge.
        let miso = PinMock::new(&input_waveform("11001100"));
        let mosi = PinMock::new(&output_waveform("10110001"));
        // Initial idle low + 8 high/low clock pulses
        let sck = PinMock::new(&output_waveform("01010101010101010"));
        let delay = MockDelay::new();

        let config = SpiConfig::new(MODE_1).with_frequency_hz(1_000_000);
        let mut spi = SPI::new(miso, mosi, sck, delay, config);
        let mut read_data = [0x00];
        spi.transfer(&mut read_data, &[0b10110001])
            .expect("SPI transfer failed");

        spi.miso.done();
        spi.mosi.done();
        spi.sck.done();
        assert_eq!(read_data[0], 0b11001100);
    }

    #[test]
    fn test_spi_lsb_first_write() {
        let miso = PinMock::new(&input_waveform("00000000"));
        // 0b1000_0011 LSB first: 1,1,0,0,0,0,0,1
        let mosi = PinMock::new(&output_waveform("11000001"));
        let sck = PinMock::new(&output_waveform("01010101010101010"));
        let delay = MockDelay::new();

        let config = SpiConfig::new(MODE_0).with_bit_order(BitOrder::LSBFirst);
        let mut spi = SPI::new(miso, mosi, sck, delay, config);
        spi.write(&[0b1000_0011]).expect("SPI write failed");

        spi.miso.done();
        spi.mosi.done();
        spi.sck.done();
    }

    /// Based on https://www.analog.com/en/resources/analog-dialogue/articles/introduction-to-spi-interface.html
    /// Figure https://www.analog.com/en/_/media/images/analog-dialogue/en/volume-52/number-3/articles/introduction-to-spi-interface/205973_fig_02.png?la=en&rev=c19f52f7fc014bbda34df6bf7c2a18fe&sc_lang=en
    #[test]
    fn analog_com_example_figure_2() {
        let miso = PinMock::new(&input_waveform("10111010"));
        let mosi = PinMock::new(&output_waveform("10100101"));
        let sck = PinMock::new(&output_waveform("01010101010101010"));
        let delay = MockDelay::new();

        let mut spi = SPI::new(miso, mosi, sck, delay, SpiConfig::default());
        let mut read_data = [0x00];
        let write_data = [0xA5];

        spi.transfer(&mut read_data, &write_data)
            .expect("SPI transfer failed");

        spi.miso.done();
        spi.mosi.done();
        spi.sck.done();
        assert_eq!(read_data[0], 0xBA); // Received bits in opposite phase
    }
}
