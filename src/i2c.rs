/*!
  # Synchronous implementation of embedded-hal 1.0 I2C traits based on GPIO bitbang

  This implementation consumes the following hardware resources:
  - A delay provider (implements [`embedded_hal::delay::DelayNs`]) for timing clock cycles
  - Two GPIO pins for SDA and SCL lines.

  Note: This implementation does not support I2C clock stretching.

  ## Hardware requirements

  1. Configure GPIO pins as Open-Drain outputs.
  2. The clock frequency is configured at construction time.

  ## Example

  ```rust,ignore
    // 100 kHz bus
    let i2c = bitbang_hal_ng::i2c::I2cBB::new(scl, sda, delay, 100_000);
    let mut sensor = Lm75::new(i2c, SlaveAddr::default());
    let temp = sensor.read_temperature().unwrap();
  ```
*/

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::{ErrorType, I2c, Operation};

/// I2C error
#[derive(Debug, Eq, PartialEq)]
pub enum Error<E> {
    /// GPIO error
    Bus(E),
    /// No ack received
    NoAck,
}

impl<E: core::fmt::Debug> embedded_hal::i2c::Error for Error<E> {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::Bus(_) => embedded_hal::i2c::ErrorKind::Other,
            Error::NoAck => embedded_hal::i2c::ErrorKind::NoAcknowledge(
                embedded_hal::i2c::NoAcknowledgeSource::Unknown,
            ),
        }
    }
}

/// Bit banging I2C device
pub struct I2cBB<SCL, SDA, Delay, E>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    scl: SCL,
    sda: SDA,
    delay: Delay,
    half_period_ns: u32,
}

impl<SCL, SDA, Delay, E> I2cBB<SCL, SDA, Delay, E>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    /// Create an instance with the given clock frequency in Hz (e.g. `100_000`
    /// for a 100 kHz bus).
    ///
    /// The actual frequency will be lower due to GPIO and delay overhead.
    ///
    /// # Panics
    ///
    /// Panics if `frequency_hz` is zero.
    pub fn new(scl: SCL, sda: SDA, delay: Delay, frequency_hz: u32) -> Self {
        assert!(frequency_hz > 0, "I2C clock frequency must be non-zero");
        I2cBB {
            scl,
            sda,
            delay,
            half_period_ns: 1_000_000_000 / (2 * frequency_hz),
        }
    }

    /// Release the pins and delay provider.
    pub fn free(self) -> (SCL, SDA, Delay) {
        (self.scl, self.sda, self.delay)
    }

    fn set_scl_high(&mut self) -> Result<(), Error<E>> {
        self.scl.set_high().map_err(Error::Bus)
    }
    fn set_scl_low(&mut self) -> Result<(), Error<E>> {
        self.scl.set_low().map_err(Error::Bus)
    }
    fn set_sda_high(&mut self) -> Result<(), Error<E>> {
        self.sda.set_high().map_err(Error::Bus)
    }
    fn set_sda_low(&mut self) -> Result<(), Error<E>> {
        self.sda.set_low().map_err(Error::Bus)
    }
    fn wait_for_clk(&mut self) {
        self.delay.delay_ns(self.half_period_ns);
    }
    /// Quarter-period wait used as SDA setup/hold time around SCL edges.
    /// SDA must never change close to an SCL rising edge: with open-drain
    /// outputs both lines rise slowly through their pull-ups, and an SDA
    /// transition racing the clock edge can be seen by slaves as a spurious
    /// START or STOP condition.
    fn wait_quarter_period(&mut self) {
        self.delay.delay_ns(self.half_period_ns / 2);
    }

    /// Generate a (repeated) START condition: SDA falls while SCL is high.
    ///
    /// SDA is first released while SCL is low so that this sequence is valid
    /// both from the idle state and in the middle of a transaction
    /// (repeated start).
    ///
    /// **This is a low-level control function** for not-quite-I2C devices.
    /// For normal I2C devices, use the [`embedded_hal::i2c::I2c`] trait.
    pub fn raw_i2c_start(&mut self) -> Result<(), Error<E>> {
        self.set_sda_high()?;
        self.wait_for_clk();
        self.set_scl_high()?;
        self.wait_for_clk();
        self.set_sda_low()?;
        self.wait_for_clk();
        self.set_scl_low()?;
        self.wait_for_clk();
        Ok(())
    }

    /// Generate a STOP condition: SDA rises while SCL is high.
    ///
    /// **This is a low-level control function** for not-quite-I2C devices.
    /// For normal I2C devices, use the [`embedded_hal::i2c::I2c`] trait.
    pub fn raw_i2c_stop(&mut self) -> Result<(), Error<E>> {
        self.set_sda_low()?;
        self.wait_for_clk();
        self.set_scl_high()?;
        self.wait_for_clk();
        self.set_sda_high()?;
        self.wait_for_clk();
        Ok(())
    }

    fn i2c_is_ack(&mut self) -> Result<bool, Error<E>> {
        self.set_sda_high()?; // release SDA so the slave can drive the ACK
        self.wait_quarter_period();
        self.set_scl_high()?;
        self.wait_for_clk();
        let ack = self.sda.is_low().map_err(Error::Bus)?;
        self.set_scl_low()?;
        self.wait_quarter_period();
        Ok(ack)
    }

    fn i2c_write_byte(&mut self, byte: u8) -> Result<(), Error<E>> {
        for bit_offset in 0..8 {
            let out_bit = (byte >> (7 - bit_offset)) & 0b1;
            if out_bit == 1 {
                self.set_sda_high()?;
            } else {
                self.set_sda_low()?;
            }
            self.wait_quarter_period();
            self.set_scl_high()?;
            self.wait_for_clk();
            self.set_scl_low()?;
            self.wait_quarter_period();
        }
        Ok(())
    }

    fn i2c_read_byte(&mut self, should_send_ack: bool) -> Result<u8, Error<E>> {
        let mut byte: u8 = 0;
        self.set_sda_high()?; // release SDA so the slave can drive data
        self.wait_quarter_period();
        for bit_offset in 0..8 {
            self.set_scl_high()?;
            self.wait_for_clk();
            if self.sda.is_high().map_err(Error::Bus)? {
                byte |= 1 << (7 - bit_offset);
            }
            self.set_scl_low()?;
            self.wait_for_clk();
        }
        if should_send_ack {
            self.set_sda_low()?;
        } else {
            self.set_sda_high()?;
        }
        self.wait_quarter_period();
        self.set_scl_high()?;
        self.wait_for_clk();
        self.set_scl_low()?;
        self.wait_quarter_period();
        Ok(byte)
    }

    fn check_ack(&mut self) -> Result<(), Error<E>> {
        if !self.i2c_is_ack()? {
            Err(Error::NoAck)
        } else {
            Ok(())
        }
    }

    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error<E>> {
        for byte in bytes {
            self.i2c_write_byte(*byte)?;
            self.check_ack()?;
        }
        Ok(())
    }

    /// Read bytes from the bus. If `nack_last` is true, the final byte is
    /// NACKed (end of the read phase); otherwise every byte is ACKed
    /// (more reads follow in the same transaction).
    fn read_bytes(&mut self, buffer: &mut [u8], nack_last: bool) -> Result<(), Error<E>> {
        let len = buffer.len();
        for (i, slot) in buffer.iter_mut().enumerate() {
            let should_send_ack = !(nack_last && i == len - 1);
            *slot = self.i2c_read_byte(should_send_ack)?;
        }
        Ok(())
    }

    /// Write raw bytes to the bus, checking the slave's ACK after each byte.
    /// No START/STOP conditions and no address byte are generated.
    ///
    /// **This is a low-level control function** for not-quite-I2C devices.
    /// For normal I2C devices, use the [`embedded_hal::i2c::I2c`] trait.
    #[inline]
    pub fn raw_write_to_slave(&mut self, output: &[u8]) -> Result<(), Error<E>> {
        self.write_bytes(output)
    }

    /// Read raw bytes from the bus, ACKing every byte except the last one,
    /// which is NACKed. No START/STOP conditions and no address byte are
    /// generated.
    ///
    /// **This is a low-level control function** for not-quite-I2C devices.
    /// For normal I2C devices, use the [`embedded_hal::i2c::I2c`] trait.
    #[inline]
    pub fn raw_read_from_slave(&mut self, input: &mut [u8]) -> Result<(), Error<E>> {
        self.read_bytes(input, true)
    }

    /// Body of [`I2c::transaction`], without the trailing STOP condition.
    fn execute_operations(
        &mut self,
        addr: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Error<E>> {
        fn is_read(op: &Operation<'_>) -> bool {
            matches!(op, Operation::Read(_))
        }
        fn is_empty(op: &Operation<'_>) -> bool {
            match op {
                Operation::Read(buf) => buf.is_empty(),
                Operation::Write(buf) => buf.is_empty(),
            }
        }

        // As per the embedded-hal contract: a (repeated) START + address byte
        // is only sent when the operation direction changes; consecutive
        // operations of the same direction are treated as one continuous
        // data stream.
        let mut current_type: Option<bool> = None;

        for i in 0..operations.len() {
            let op_is_read = is_read(&operations[i]);

            if current_type != Some(op_is_read) {
                self.raw_i2c_start()?;
                let rw_bit = if op_is_read { 0x1 } else { 0x0 };
                self.i2c_write_byte((addr << 1) | rw_bit)?;
                self.check_ack()?;
                current_type = Some(op_is_read);

                // Once addressed for read, the slave starts driving data, so
                // at least one byte must be clocked out and NACKed before the
                // bus can be released. If every read in this run has an empty
                // buffer, read and discard a dummy byte.
                if op_is_read {
                    let run_is_empty = operations[i..]
                        .iter()
                        .take_while(|op| is_read(op))
                        .all(is_empty);
                    if run_is_empty {
                        self.i2c_read_byte(false)?;
                    }
                }
            }

            // NACK only the very last byte actually read in a run of
            // consecutive reads, i.e. when no later read in the same run
            // still has data to receive (empty buffers don't count).
            let more_read_data_follows = operations[i + 1..]
                .iter()
                .take_while(|op| is_read(op))
                .any(|op| !is_empty(op));
            let nack_last = !more_read_data_follows;

            match &mut operations[i] {
                Operation::Read(buf) => self.read_bytes(buf, nack_last)?,
                Operation::Write(buf) => self.write_bytes(buf)?,
            }
        }

        Ok(())
    }
}

impl<SCL, SDA, Delay, E> ErrorType for I2cBB<SCL, SDA, Delay, E>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    type Error = Error<E>;
}

impl<SCL, SDA, Delay, E> I2c<u8> for I2cBB<SCL, SDA, Delay, E>
where
    SCL: OutputPin<Error = E>,
    SDA: OutputPin<Error = E> + InputPin<Error = E>,
    Delay: DelayNs,
    E: core::fmt::Debug,
{
    fn transaction(
        &mut self,
        addr: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }

        let result = self.execute_operations(addr, operations);
        // Always send STOP so the bus is released even when an operation
        // failed mid-transaction; the operation error takes precedence.
        let stop_result = self.raw_i2c_stop();
        result.and(stop_result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::i2c::{I2c, Operation};
    use embedded_hal_mock::eh1::delay::NoopDelay as MockDelay;
    use embedded_hal_mock::eh1::digital::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };
    use std::vec::Vec;

    fn pin_transactions(states: &[PinState]) -> Vec<PinTransaction> {
        states.iter().map(|&s| PinTransaction::set(s)).collect()
    }

    #[test]
    fn test_raw_i2c_start_and_stop() {
        let scl = PinMock::new(&pin_transactions(&[
            PinState::High, // start: scl high after sda released
            PinState::Low,  // start: scl low
            PinState::High, // stop: scl high
        ]));
        let sda = PinMock::new(&pin_transactions(&[
            PinState::High, // start: release sda while scl low
            PinState::Low,  // start: sda falls while scl high (START)
            PinState::Low,  // stop: ensure sda low while scl low
            PinState::High, // stop: sda rises while scl high (STOP)
        ]));
        let delay = MockDelay::new();

        let mut i2c = I2cBB::new(scl, sda, delay, 100_000);
        i2c.raw_i2c_start().expect("start failed");
        i2c.raw_i2c_stop().expect("stop failed");

        i2c.scl.done();
        i2c.sda.done();
    }

    #[test]
    fn test_write_and_ack() {
        // 0b10101010
        let scl = PinMock::new(&vec![
            // Each bit: set_scl_high, set_scl_low
            PinTransaction::set(PinState::High), // bit 7 clock high
            PinTransaction::set(PinState::Low),  // bit 7 clock low
            PinTransaction::set(PinState::High), // bit 6 clock high
            PinTransaction::set(PinState::Low),  // bit 6 clock low
            PinTransaction::set(PinState::High), // bit 5 clock high
            PinTransaction::set(PinState::Low),  // bit 5 clock low
            PinTransaction::set(PinState::High), // bit 4 clock high
            PinTransaction::set(PinState::Low),  // bit 4 clock low
            PinTransaction::set(PinState::High), // bit 3 clock high
            PinTransaction::set(PinState::Low),  // bit 3 clock low
            PinTransaction::set(PinState::High), // bit 2 clock high
            PinTransaction::set(PinState::Low),  // bit 2 clock low
            PinTransaction::set(PinState::High), // bit 1 clock high
            PinTransaction::set(PinState::Low),  // bit 1 clock low
            PinTransaction::set(PinState::High), // bit 0 clock high
            PinTransaction::set(PinState::Low),  // bit 0 clock low
        ]);
        let sda = PinMock::new(&pin_transactions(&[
            // One set_sda per bit, MSB first
            PinState::High, // 1
            PinState::Low,  // 0
            PinState::High, // 1
            PinState::Low,  // 0
            PinState::High, // 1
            PinState::Low,  // 0
            PinState::High, // 1
            PinState::Low,  // 0
        ]));
        let delay = MockDelay::new();

        let mut i2c = I2cBB::new(scl, sda, delay, 100_000);
        i2c.i2c_write_byte(0b10101010).expect("write failed");
        i2c.scl.done();
        i2c.sda.done();
    }

    #[derive(Debug)]
    struct DummyPin;
    impl embedded_hal::digital::ErrorType for DummyPin {
        type Error = core::convert::Infallible;
    }
    impl embedded_hal::digital::OutputPin for DummyPin {
        fn set_high(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
        fn set_low(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
    impl embedded_hal::digital::InputPin for DummyPin {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(false)
        }
        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(true)
        }
    }
    struct DummyDelay;
    impl embedded_hal::delay::DelayNs for DummyDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    #[test]
    fn test_i2c_trait_write() {
        let mut i2c = I2cBB::new(DummyPin, DummyPin, DummyDelay, 100_000);
        let mut ops = [Operation::Write(&[0xAB])];
        // I2cBB implements the I2c trait directly (and `&mut I2cBB` via the
        // embedded-hal blanket impl)
        i2c.transaction(0x50, &mut ops).expect("i2c write failed");
        I2c::transaction(&mut &mut i2c, 0x50, &mut ops).expect("i2c write failed");
    }

    #[test]
    fn test_raw_slave_access() {
        let mut i2c = I2cBB::new(DummyPin, DummyPin, DummyDelay, 100_000);
        // DummyPin reads low, so ACK checks pass and read bytes are zero
        i2c.raw_i2c_start().expect("start failed");
        i2c.raw_write_to_slave(&[0xDE, 0xAD]).expect("write failed");
        let mut buf = [0xFFu8; 2];
        i2c.raw_read_from_slave(&mut buf).expect("read failed");
        i2c.raw_i2c_stop().expect("stop failed");
        assert_eq!(buf, [0, 0]);
    }

    #[test]
    fn test_i2c_empty_operations() {
        let mut i2c = I2cBB::new(DummyPin, DummyPin, DummyDelay, 100_000);

        // Probe-style empty write: START + address only
        i2c.transaction(0x50, &mut [Operation::Write(&[])])
            .expect("empty write failed");

        // Lone empty read: a dummy byte must be read and NACKed
        i2c.transaction(0x50, &mut [Operation::Read(&mut [])])
            .expect("empty read failed");

        // Trailing empty read must not steal the NACK from the last byte
        // actually read
        let mut buf = [0xFFu8; 2];
        let mut empty: [u8; 0] = [];
        let mut ops = [Operation::Read(&mut buf), Operation::Read(&mut empty)];
        i2c.transaction(0x50, &mut ops)
            .expect("read + empty failed");
        assert_eq!(buf, [0, 0]);
    }

    #[test]
    fn test_i2c_write_read() {
        let mut i2c = I2cBB::new(DummyPin, DummyPin, DummyDelay, 100_000);
        let mut buf = [0u8; 2];
        // write followed by read: repeated start in between
        i2c.write_read(0x50, &[0x01], &mut buf)
            .expect("write_read failed");
        // DummyPin always reads low
        assert_eq!(buf, [0, 0]);
    }
}
