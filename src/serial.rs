//! Serial communication (UART)
//!
//! 8-N-1 (8 data bits, no parity, 1 stop bit), LSB first.
//!
//! This implementation consumes the following hardware resources:
//! - Output GPIO pin for transmission (TX)
//! - Input GPIO pin for reception (RX)
//! - A delay provider (implements [`embedded_hal::delay::DelayNs`]) for bit timing
//!
//! The baud rate is configured at construction time. With the `embedded-io`
//! feature enabled, the [`embedded_io::Read`] and [`embedded_io::Write`]
//! traits are implemented on top of the inherent [`Serial::read_byte`] and
//! [`Serial::write_byte`] methods. Per the `embedded_io` contract,
//! [`embedded_io::Read::read`] blocks only until one byte has been received
//! and then returns, rather than waiting to fill the whole buffer.
//!
//! Note: reception is blocking and starts by busy-waiting for a start bit.
//! Accurate timing depends on the delay provider; at high baud rates GPIO
//! and delay overhead will dominate. A stop bit that samples low is
//! reported as [`Error::FrameFormat`].

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};

/// Serial communication error type
#[derive(Debug)]
pub enum Error<E> {
    /// Bus error
    Bus(E),
    /// The stop bit sampled low: the line is not idle where the frame should
    /// end (baud-rate mismatch or line noise).
    FrameFormat,
}

/// Bit banging serial communication (UART) device
pub struct Serial<TX, RX, Delay>
where
    TX: OutputPin,
    RX: InputPin,
    Delay: DelayNs,
{
    tx: TX,
    rx: RX,
    delay: Delay,
    bit_duration_ns: u32,
}

impl<TX, RX, Delay, E> Serial<TX, RX, Delay>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Delay: DelayNs,
{
    /// Create an instance with the given baud rate (e.g. `9600`).
    ///
    /// # Panics
    ///
    /// Panics if `baud_rate` is zero.
    pub fn new(tx: TX, rx: RX, delay: Delay, baud_rate: u32) -> Self {
        assert!(baud_rate > 0, "baud rate must be non-zero");
        Serial {
            tx,
            rx,
            delay,
            bit_duration_ns: 1_000_000_000 / baud_rate,
        }
    }

    /// Release the pins and delay provider.
    pub fn free(self) -> (TX, RX, Delay) {
        (self.tx, self.rx, self.delay)
    }

    #[inline]
    fn wait_bit(&mut self) {
        self.delay.delay_ns(self.bit_duration_ns);
    }

    /// Transmit a single byte (blocking).
    pub fn write_byte(&mut self, byte: u8) -> Result<(), Error<E>> {
        let mut data_out = byte;
        self.tx.set_low().map_err(Error::Bus)?; // start bit
        self.wait_bit();
        for _ in 0..8 {
            // LSB first
            if data_out & 1 == 1 {
                self.tx.set_high().map_err(Error::Bus)?;
            } else {
                self.tx.set_low().map_err(Error::Bus)?;
            }
            data_out >>= 1;
            self.wait_bit();
        }
        self.tx.set_high().map_err(Error::Bus)?; // stop bit
        self.wait_bit();
        Ok(())
    }

    /// Receive a single byte (blocking).
    ///
    /// Busy-waits for a start bit, then samples each data bit at its
    /// mid-point. Returns [`Error::FrameFormat`] if the stop bit samples
    /// low.
    pub fn read_byte(&mut self) -> Result<u8, Error<E>> {
        let mut data_in = 0u8;

        // wait for falling edge of the start bit
        while self.rx.is_high().map_err(Error::Bus)? {}

        // skip the rest of the start bit and land in the middle of the
        // first data bit
        self.delay
            .delay_ns(self.bit_duration_ns + self.bit_duration_ns / 2);

        for bit in 0..8 {
            // LSB first
            if self.rx.is_high().map_err(Error::Bus)? {
                data_in |= 1 << bit;
            }
            self.wait_bit();
        }

        // we are now in the middle of the stop bit; sample it to detect
        // framing errors, then let the rest play out while the caller
        // returns so back-to-back reads do not miss the next start bit
        if !self.rx.is_high().map_err(Error::Bus)? {
            return Err(Error::FrameFormat);
        }
        Ok(data_in)
    }
}

#[cfg(feature = "embedded-io")]
impl<E: core::fmt::Debug> embedded_io::Error for Error<E> {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Error::Bus(_) => embedded_io::ErrorKind::Other,
            Error::FrameFormat => embedded_io::ErrorKind::InvalidData,
        }
    }
}

#[cfg(feature = "embedded-io")]
impl<TX, RX, Delay, E> embedded_io::ErrorType for Serial<TX, RX, Delay>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    type Error = Error<E>;
}

#[cfg(feature = "embedded-io")]
impl<TX, RX, Delay, E> embedded_io::Read for Serial<TX, RX, Delay>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // The embedded_io contract is to block only until some data is
        // available, not until the buffer is full. A bitbang UART cannot
        // detect pending data without blocking, so read exactly one byte.
        let Some(slot) = buf.first_mut() else {
            return Ok(0);
        };
        *slot = self.read_byte()?;
        Ok(1)
    }
}

#[cfg(feature = "embedded-io")]
impl<TX, RX, Delay, E> embedded_io::Write for Serial<TX, RX, Delay>
where
    TX: OutputPin<Error = E>,
    RX: InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for &byte in buf {
            self.write_byte(byte)?;
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // No internal buffering, so nothing to do
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::delay::NoopDelay as MockDelay;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };
    use std::vec::Vec;

    /// Expected TX pin transactions for a byte: start bit, 8 data bits
    /// (LSB first), stop bit.
    fn tx_waveform(byte: u8) -> Vec<PinTransaction> {
        let mut transactions = Vec::new();
        transactions.push(PinTransaction::set(PinState::Low)); // start bit
        let mut data = byte;
        for _ in 0..8 {
            let state = if data & 1 == 1 {
                PinState::High
            } else {
                PinState::Low
            };
            transactions.push(PinTransaction::set(state));
            data >>= 1;
        }
        transactions.push(PinTransaction::set(PinState::High)); // stop bit
        transactions
    }

    /// RX pin reads for a byte: one read for the start-bit busy-wait,
    /// 8 data bit samples (LSB first), then the stop bit sample.
    fn rx_waveform(byte: u8) -> Vec<PinTransaction> {
        let mut transactions = Vec::new();
        transactions.push(PinTransaction::get(PinState::Low)); // start bit detected
        let mut data = byte;
        for _ in 0..8 {
            let state = if data & 1 == 1 {
                PinState::High
            } else {
                PinState::Low
            };
            transactions.push(PinTransaction::get(state));
            data >>= 1;
        }
        transactions.push(PinTransaction::get(PinState::High)); // stop bit
        transactions
    }

    #[test]
    fn test_serial_write_byte() {
        let byte = 0b0101_0101;
        let tx = PinMock::new(&tx_waveform(byte));
        let rx = PinMock::new(&[]);
        let delay = MockDelay::new();

        let mut serial = Serial::new(tx, rx, delay, 9600);
        serial.write_byte(byte).expect("write failed");

        serial.tx.done();
        serial.rx.done();
    }

    #[test]
    fn test_serial_read_byte() {
        let byte = 0b1010_1010;
        let tx = PinMock::new(&[]);
        let rx = PinMock::new(&rx_waveform(byte));
        let delay = MockDelay::new();

        let mut serial = Serial::new(tx, rx, delay, 9600);
        let read = serial.read_byte().expect("read failed");
        assert_eq!(read, byte);

        serial.tx.done();
        serial.rx.done();
    }

    #[cfg(feature = "embedded-io")]
    #[test]
    fn test_serial_embedded_io_roundtrip() {
        use embedded_io::{Read, Write};

        let bytes = [0x42, 0xA7];
        let mut tx_transactions = tx_waveform(bytes[0]);
        tx_transactions.extend(tx_waveform(bytes[1]));
        let mut rx_transactions = rx_waveform(bytes[0]);
        rx_transactions.extend(rx_waveform(bytes[1]));

        let tx = PinMock::new(&tx_transactions);
        let rx = PinMock::new(&rx_transactions);
        let delay = MockDelay::new();

        let mut serial = Serial::new(tx, rx, delay, 115_200);
        let written = serial.write(&bytes).expect("write failed");
        assert_eq!(written, 2);

        // embedded_io::Read returns after one byte, so two calls are needed
        let mut buf = [0u8; 2];
        let read = serial.read(&mut buf).expect("read failed");
        assert_eq!(read, 1);
        let read = serial.read(&mut buf[1..]).expect("read failed");
        assert_eq!(read, 1);
        assert_eq!(buf, bytes);

        // empty buffer returns immediately without touching the pins
        assert_eq!(serial.read(&mut []).expect("empty read failed"), 0);

        serial.tx.done();
        serial.rx.done();
    }

    #[test]
    fn test_serial_framing_error() {
        let byte = 0b0101_0101;
        let mut transactions = rx_waveform(byte);
        // corrupt the stop bit
        transactions.pop();
        transactions.push(PinTransaction::get(PinState::Low));

        let tx = PinMock::new(&[]);
        let rx = PinMock::new(&transactions);
        let delay = MockDelay::new();

        let mut serial = Serial::new(tx, rx, delay, 9600);
        assert!(matches!(serial.read_byte(), Err(Error::FrameFormat)));

        serial.tx.done();
        serial.rx.done();
    }
}
