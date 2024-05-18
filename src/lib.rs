//! Library for the RF protocol used by [Nexus Revo] (ca. 2018, rechargeable remote) product line.
//!
//! Compatible with the following devices:
//! * Revo Slim
//! * Revo Stealth
//! * Revo Extreme
//! * Revo Intense
//!
//! Designed for use with [libftd2xx-rs] and [libftd2xx-cc1101] to interface with a [CC1101] RF
//! transceiver via FTDI SPI interface. However, any properly configured [`std::io`] interface
//! that functions as a 433.94 MHz, 2.2254 kBaud, OOK RF modem can be used instead.
//!
//! # Usage
//! Simply add this crate as a dependency in your `Cargo.toml`.
//!
//! ```toml
//! [dependencies]
//! nexus-revo-io = "~0.1.0"
//! ```
//!
//! Also follow the README guidance in [libftd2xx-rs] to set up the connection with the FTDI device.
//!
//! [Sender] and [Receiver] examples are good starting points.
//!
//! [Nexus Revo]: https://nexusrange.com
//! [libftd2xx-rs]: https://github.com/newAM/libftd2xx-rs
//! [libftd2xx-cc1101]: https://github.com/CirrusNeptune/libftd2xx-cc1101
//! [CC1101]: https://www.ti.com/product/CC1101
//! [Sender]: https://github.com/CirrusNeptune/nexus-revo-io/blob/main/examples/sender.rs
//! [Receiver]: https://github.com/CirrusNeptune/nexus-revo-io/blob/main/examples/receiver.rs

//#![deny(missing_docs, unsafe_code)]

use bitstream_io::{BigEndian, BitRead, BitReader, BitWrite, BitWriter};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::io;
use std::io::{Error, ErrorKind, Write};
use libftd2xx::FtdiCommon;

/// Remote commands supported by Revo hardware.
#[derive(Debug, Copy, Clone, PartialEq, FromPrimitive)]
pub enum NexusCmd {

    Tamper = 7,
    Open = 10,
    Close = 14,
}

#[derive(PartialEq)]
enum Symbol {
    SyncZeros,
    SyncOnes,
    Zero,
    One,
}

pub trait CsvLogger {
    fn log_bit(&mut self, bit: bool, rssi_dbm: Option<i32>);
}

pub struct WriteCsvLogger<W: Write> {
    csv_write: W,
    raw_counter: u32,
    parsed_bit: bool,
    parsed_msg: bool,
}

impl<W: Write> WriteCsvLogger<W> {
    pub fn new(csv_write: W) -> Self {
        let mut ret = Self {
            csv_write,
            raw_counter: 0,
            parsed_bit: false,
            parsed_msg: false,
        };
        ret.csv_write.write_fmt(format_args!("x,y,type\n")).unwrap();
        ret
    }
}

impl<W: Write> CsvLogger for WriteCsvLogger<W> {
    fn log_bit(&mut self, bit: bool, rssi_dbm: Option<i32>) {
        self.raw_counter += 1;
        self.csv_write.write_fmt(format_args!("{},{},raw\n", self.raw_counter, bit as u32)).unwrap();
        self.csv_write.write_fmt(format_args!("{},{},parsed\n", self.raw_counter, self.parsed_bit as u32 * 2)).unwrap();
        self.csv_write.write_fmt(format_args!("{},{},msg\n", self.raw_counter, self.parsed_msg as u32 * 3)).unwrap();
        if let Some(r) = rssi_dbm {
            if (r % 2) != 0 {
                self.csv_write.write_fmt(format_args!("{},{}.5,rssi\n", self.raw_counter, r / 2)).unwrap();
            } else {
                self.csv_write.write_fmt(format_args!("{},{}.0,rssi\n", self.raw_counter, r / 2)).unwrap();
            }
        }
    }
}

/// Bit-level symbol reader for detecting and decoding messages in Revo RF protocol.
///
/// Designed to block and continuously scan for bit patterns containing coded symbols. Incoming
/// bytes do not need to be synchronized to sync words.
pub struct SymReader<'f, 'c, Ft: FtdiCommon, const BUF_CAP: usize, Logger: CsvLogger> {
    reader: BitReader<libftd2xx_cc1101::io::FifoReader<'f, 'c, Ft, BUF_CAP>, BigEndian>,
    window: u8,
    logger: Option<Logger>,
}

impl<'f, 'c, Ft: FtdiCommon, const BUF_CAP: usize, Logger: CsvLogger> SymReader<'f, 'c, Ft, BUF_CAP, Logger> {
    /// Constructs new SymReader wrapped around [`io::Read`] trait.
    pub fn new_with_logger(reader: libftd2xx_cc1101::io::FifoReader<'f, 'c, Ft, BUF_CAP>, logger: Option<Logger>) -> Self {
        Self {
            reader: BitReader::endian(reader, BigEndian),
            window: 0,
            logger,
        }
    }

    pub fn new(reader: libftd2xx_cc1101::io::FifoReader<'f, 'c, Ft, BUF_CAP>) -> Self {
        Self::new_with_logger(reader, None)
    }

    fn read_bit(&mut self) -> io::Result<bool> {
        let bit = self.reader.read_bit()?;
        if let Some(ref mut logger) = self.logger {
            let rssi_dbm = if let Some(r) = self.reader.reader() {
                let rssi_dec = r.rssi()?;
                Some(if rssi_dec >= 128 {
                    rssi_dec as i32 - 256
                } else {
                    rssi_dec as i32
                })
            } else {
                None
            };
            logger.log_bit(bit, rssi_dbm);
        }
        self.window <<= 1;
        self.window |= bit as u8;
        Ok(bit)
    }

    fn read_until_0(&mut self) -> io::Result<()> {
        while self.read_bit()? {}
        Ok(())
    }

    fn read_until_1(&mut self) -> io::Result<()> {
        while !(self.read_bit()?) {}
        Ok(())
    }

    fn read_symbol(&mut self) -> io::Result<Symbol> {
        loop {
            self.read_until_1()?;
            let mut one_count = 1_u32;
            while self.read_bit()? {
                one_count += 1;
            }
            return if one_count >= 4 {
                Ok(Symbol::One)
            } else {
                Ok(Symbol::Zero)
            }
        }
    }

    fn sync(&mut self) -> io::Result<()> {
        while self.read_symbol()? != Symbol::SyncZeros {}
        Ok(())
    }

    fn read_bit_symbol(&mut self) -> io::Result<bool> {
        match self.read_symbol()? {
            Symbol::Zero => {
                //self.bit_counter += 1;
                //self.parsed_bit = false;
                //println!("bit {} 0", self.bit_counter);
                Ok(false)
            }
            Symbol::One => {
                //self.bit_counter += 1;
                //self.parsed_bit = true;
                //println!("bit {} 1", self.bit_counter);
                Ok(true)
            }
            _ => Err(Error::new(ErrorKind::InvalidInput, "no data symbols")),
        }
    }

    fn read_byte(&mut self) -> io::Result<u8> {
        let mut value = 0;
        for _ in 0..8 {
            value <<= 1;
            value |= self.read_bit_symbol()? as u8;
        }
        Ok(value)
    }

    fn read_addr(&mut self) -> io::Result<u16> {
        let byte0 = self.read_byte()?;
        let byte1 = self.read_byte()?;
        Ok(u16::from_be_bytes([byte0, byte1]))
    }

    fn read_cmd(&mut self) -> io::Result<NexusCmd> {
        let byte = self.read_byte()?;
        println!("cmd: {:2X}", byte);
        FromPrimitive::from_u8(byte)
            .ok_or_else(|| io::Error::new(ErrorKind::InvalidInput, "unknown command"))
    }

    /// Blocks until a complete, valid Revo message is detected and decoded.
    pub fn read_msg(&mut self) -> io::Result<u8> {
        let mut code_search = 0_u32;
        loop {
            let bit = self.read_bit_symbol()?;
            code_search <<= 1;
            code_search |= bit as u32;
            if code_search & 0x1FFFFFF == 0x13CB862 {
                return Ok(1);
            } else if code_search & 0xFFFF == 0x59CF {
                //self.parsed_msg = true;
                let cmd_byte = self.read_byte()?;
                //self.parsed_msg = false;
                return Ok(cmd_byte);
            }
        }
    }
}
