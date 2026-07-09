//! Integration test: drive a (mocked) TI DRV8301 gate driver with the
//! bit-banged SPI bus through the `drv8301-dd` driver crate.
//!
//! The DRV8301 SPI interface is MODE_1 (CPOL = 0, CPHA = 1), 16-bit frames,
//! MSB first. A register read is two 16-bit frames: the command frame and a
//! repeated command frame that clocks out the response (N+1 timing).

use bitbang_hal_ng::spi::{SPI, SpiConfig};
use drv8301_dd::Drv8301;
use embedded_hal::spi::MODE_1;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::digital::{
    Mock as PinMock, State as PinState, Transaction as PinTransaction,
};

/// 16 MSB-first input samples for one frame received on MISO
fn miso_frame(word: u16) -> Vec<PinTransaction> {
    (0..16)
        .rev()
        .map(|bit| {
            PinTransaction::get(if (word >> bit) & 1 == 1 {
                PinState::High
            } else {
                PinState::Low
            })
        })
        .collect()
}

/// 16 MSB-first output transitions for one frame sent on MOSI
fn mosi_frame(word: u16) -> Vec<PinTransaction> {
    (0..16)
        .rev()
        .map(|bit| {
            PinTransaction::set(if (word >> bit) & 1 == 1 {
                PinState::High
            } else {
                PinState::Low
            })
        })
        .collect()
}

/// One MODE_1 clock pulse (high, low) per bit
fn sck_frame() -> Vec<PinTransaction> {
    (0..16)
        .flat_map(|_| {
            [
                PinTransaction::set(PinState::High),
                PinTransaction::set(PinState::Low),
            ]
        })
        .collect()
}

#[test]
fn drv8301_get_device_id_over_bitbang_spi() {
    // Read command for Status Register 2 (address 0x01):
    // bit 15 = 1 (read), bits 14:11 = address -> 0x8800
    let read_cmd: u16 = 0x8800;
    // Response frame (second transfer): frame fault = 0, address echo = 0x1,
    // D10:0 = GVDD_OV clear, DEVICE_ID = 0x1 -> 0x0801
    let response: u16 = 0x0801;

    // The driver issues two 16-bit transfers, each in its own CS frame
    let mut mosi_seq = mosi_frame(read_cmd);
    mosi_seq.extend(mosi_frame(read_cmd));
    let mut miso_seq = miso_frame(0x0000); // first response is discarded
    miso_seq.extend(miso_frame(response));

    // MODE_1: clock idles low (set once at construction), then pulses per bit
    let mut sck_seq = vec![PinTransaction::set(PinState::Low)];
    sck_seq.extend(sck_frame());
    sck_seq.extend(sck_frame());

    // ExclusiveDevice: CS deasserted at creation, then framed per transfer
    let cs_seq = [
        PinTransaction::set(PinState::High),
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
        PinTransaction::set(PinState::Low),
        PinTransaction::set(PinState::High),
    ];

    // Clones share the expectation state, so we can verify completion
    // after the pins have been moved into the driver stack
    let mut miso = PinMock::new(&miso_seq);
    let mut mosi = PinMock::new(&mosi_seq);
    let mut sck = PinMock::new(&sck_seq);
    let mut cs = PinMock::new(&cs_seq);

    let config = SpiConfig::new(MODE_1).with_frequency_hz(1_000_000);
    let bus = SPI::new(
        miso.clone(),
        mosi.clone(),
        sck.clone(),
        NoopDelay::new(),
        config,
    )
    .expect("failed to create SPI bus");
    let spi_device = ExclusiveDevice::new(bus, cs.clone(), NoopDelay::new()).unwrap();

    let mut drv = Drv8301::new(spi_device);
    let device_id = drv.get_device_id().expect("device id read failed");
    assert_eq!(device_id, 0x1);

    // Verify all expected pin transactions happened
    miso.done();
    mosi.done();
    sck.done();
    cs.done();
}
