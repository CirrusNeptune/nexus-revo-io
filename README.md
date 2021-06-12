![Maintenance](https://img.shields.io/badge/maintenance-experimental-blue.svg)
[![crates.io](https://img.shields.io/crates/v/nexus-revo-io.svg)](https://crates.io/crates/nexus-revo-io)
[![docs.rs](https://docs.rs/nexus-revo-io/badge.svg)](https://docs.rs/nexus-revo-io/)
[![CI](https://github.com/CirrusNeptune/nexus-revo-io/workflows/CI/badge.svg)](https://github.com/CirrusNeptune/nexus-revo-io/actions)

# nexus-revo-io

Library for the RF protocol used by [Nexus Revo] (ca. 2018, rechargeable remote) product line.

Compatible with the following devices:
* Revo Slim
* Revo Stealth
* Revo Extreme
* Revo Intense

Designed for use with [libftd2xx-rs] and [libftd2xx-cc1101] to interface with a [CC1101] RF
transceiver via FTDI SPI interface. However, any properly configured [`std::io`] interface
that behaves as a 433.94 MHz, 2.2254 kBaud, OOK RF modem can be used instead.

## Usage
Simply add this crate as a dependency in your `Cargo.toml`.

```toml
[dependencies]
nexus-revo-io = "~0.1.0"
```

Also follow the README guidance in [libftd2xx-rs] to set up the connection with the FTDI device.

[Sender] and [Receiver] examples are good starting points.

[Nexus Revo]: https://nexusrange.com
[libftd2xx-rs]: https://github.com/newAM/libftd2xx-rs
[libftd2xx-cc1101]: https://github.com/CirrusNeptune/libftd2xx-cc1101
[CC1101]: https://www.ti.com/product/CC1101
[Sender]: https://github.com/CirrusNeptune/nexus-revo-io/blob/main/examples/sender.rs
[Receiver]: https://github.com/CirrusNeptune/nexus-revo-io/blob/main/examples/receiver.rs
