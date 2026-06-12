# `bitbang-hal-ng`

[![crates.io](https://img.shields.io/crates/v/bitbang-hal-ng.svg)](https://crates.io/crates/bitbang-hal-ng)
[![Docs](https://docs.rs/bitbang-hal-ng/badge.svg)](https://docs.rs/bitbang-hal-ng)

A [bit banging] implementation of the [`embedded-hal`] **1.0** traits.
This is a maintained fork of [`bitbang-hal`](https://github.com/sajattack/bitbang-hal),
ported to `embedded-hal` 1.0 / `embedded-io`.

[bit banging]: https://en.wikipedia.org/wiki/Bit_banging
[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal

## Provided implementations

| Module   | Trait(s)                                             | Notes                                       |
|----------|------------------------------------------------------|---------------------------------------------|
| `spi`    | `embedded_hal::spi::SpiBus`                          | All four SPI modes, MSB/LSB-first bit order |
| `i2c`    | `embedded_hal::i2c::I2c`                             | No clock stretching support                 |
| `serial` | `embedded_io::Read`/`Write` (feature `embedded-io`)  | 8-N-1 UART                                  |

All implementations are timed with an `embedded_hal::delay::DelayNs` provider;
the target frequency / baud rate is set at construction time. Actual bus
speeds will be lower than the target due to GPIO and delay overhead — bit
banging is best suited for low-to-medium speed buses.

## Usage

SPI (e.g. for a TI DRV8301 gate driver, which speaks SPI mode 1):

```rust,ignore
use embedded_hal::spi::MODE_1;
use embedded_hal_bus::spi::ExclusiveDevice;
use bitbang_hal_ng::spi::{SPI, SpiConfig};

let config = SpiConfig::new(MODE_1).with_frequency_hz(500_000);
let bus = SPI::new(miso, mosi, sck, delay, config);
// SpiDevice with managed CS pin:
let spi_device = ExclusiveDevice::new(bus, cs, cs_delay).unwrap();
let mut drv = drv8301_dd::Drv8301::new(spi_device);
```

I2C (pins must be configured as open-drain outputs):

```rust,ignore
use bitbang_hal_ng::i2c::I2cBB;

let i2c = I2cBB::new(scl, sda, delay, 100_000); // 100 kHz
```

Serial:

```rust,ignore
use bitbang_hal_ng::serial::Serial;
use embedded_io::Write;

let mut serial = Serial::new(tx, rx, delay, 9600);
serial.write(b"hello")?;
```

## Support

For questions, issues, feature requests, and other changes, please file an
issue in the github project.

## License

Licensed under MIT license ([LICENSE](LICENSE) or http://opensource.org/licenses/MIT)

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you shall be licensed as above, without any
additional terms or conditions.
