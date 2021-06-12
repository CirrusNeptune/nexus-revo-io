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
//! that behaves as a 433.94 MHz, 2.2254 kBaud, OOK RF modem can be used instead.
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

#![deny(missing_docs, unsafe_code, warnings)]

use bitstream_io::{BitRead, BitWrite, BitReader, BitWriter, BigEndian};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::io;
use std::io::{Error, ErrorKind};

/// Remote commands supported by Revo hardware.
#[derive(Debug, Copy, Clone, PartialEq, FromPrimitive)]
pub enum NexusCmd {
    /// Toggles vibration and rotation on and off while remembering the mode of each.
    Pause = 1,
    /// Powers off Revo device. Must be turned back on at the device to re-establish communication.
    PowerOff = 2,
    /// Cycles through vibration modes.
    VibrateMode = 3,
    /// Turns off vibration and resets vibration mode. Idempotency can be faked by transmitting this
    /// and a certain number of [`VibrateMode`][`NexusCmd::VibrateMode`] commands.
    VibrateOff = 4,
    /// Cycles through rotation modes.
    RotateMode = 5,
    /// Turns off rotation and resets rotation mode. Idempotency can be faked by transmitting this
    /// and a certain number of [`RotateMode`][`NexusCmd::RotateMode`] commands.
    RotateOff = 6,
}

#[derive(PartialEq)]
enum Symbol {
    SyncZeros,
    SyncOnes,
    Zero,
    One,
}

/// Bit-level symbol reader for detecting and decoding messages in Revo RF protocol.
///
/// Designed to block and continuously scan for bit patterns containing coded symbols. Incoming
/// bytes do not need to be synchronized to sync words.
pub struct SymReader<'a, R: io::Read> {
    reader: BitReader<&'a mut R, BigEndian>,
    window: u8,
}

impl<'a, R: io::Read> SymReader<'a, R> {
    /// Constructs new SymReader wrapped around [`io::Read`] trait.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use nexus_revo_io::SymReader;
    /// # use libftd2xx::{Ft232h, Ftdi};
    /// # use libftd2xx_cc1101::CC1101;
    /// # use std::convert::TryInto;
    /// # use std::io::Read;
    /// # let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// # let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// # let mut cc1101 = CC1101::new(&mut ftdi);
    /// # let mut cc1101_reader = cc1101.reader::<32>();
    /// let mut sym_reader = SymReader::new(&mut cc1101_reader);
    /// ```
    pub fn new(reader: &'a mut R) -> Self {
        Self {
            reader: BitReader::endian(reader, BigEndian),
            window: 0,
        }
    }

    fn read_bit(&mut self) -> io::Result<bool> {
        let bit = self.reader.read_bit()?;
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
            self.read_until_0()?;
            if (self.window >> 1) & 0xf == 0xf {
                return Ok(Symbol::SyncOnes);
            }
            self.read_until_1()?;
            match (self.window >> 1) & 0xf {
                0x0 => return Ok(Symbol::SyncZeros),
                0x8 => return Ok(Symbol::Zero),
                0xe => return Ok(Symbol::One),
                _ => {}
            }
        }
    }

    fn sync(&mut self) -> io::Result<()> {
        while self.read_symbol()? != Symbol::SyncZeros {}
        Ok(())
    }

    fn read_bit_symbol(&mut self) -> io::Result<bool> {
        match self.read_symbol()? {
            Symbol::Zero => Ok(false),
            Symbol::One => Ok(true),
            _ => Err(Error::new(ErrorKind::InvalidInput, "no data symbols")),
        }
    }

    fn read_byte(&mut self) -> io::Result<u8> {
        let mut value = 0;
        for i in (0..4).rev() {
            let sym0 = self.read_bit_symbol()? as u8;
            let sym1 = self.read_bit_symbol()? as u8;
            value |= (sym0 << (i * 2 + 1)) | (sym1 << (i * 2));
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
        if byte & 0b1000 == 0 || byte & 0x7 != (byte >> 4) & 0x7 {
            return Err(io::Error::new(ErrorKind::InvalidInput, "malformed command"));
        }
        FromPrimitive::from_u8(byte & 0x7)
            .ok_or_else(|| io::Error::new(ErrorKind::InvalidInput, "unknown command"))
    }

    /// Blocks until a complete, valid Revo message is detected and decoded.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use nexus_revo_io::SymReader;
    /// # use libftd2xx::{Ft232h, Ftdi};
    /// # use libftd2xx_cc1101::CC1101;
    /// # use std::convert::TryInto;
    /// # use std::io::Read;
    /// # let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// # let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// # let mut cc1101 = CC1101::new(&mut ftdi);
    /// # let mut cc1101_reader = cc1101.reader::<32>();
    /// # let mut sym_reader = SymReader::new(&mut cc1101_reader);
    /// loop {
    ///     let msg = sym_reader.read_msg().expect("bad read_msg");
    ///     println!("{:x?} {:?}", msg.0, msg.1);
    /// }
    /// ```
    pub fn read_msg(&mut self) -> io::Result<(u16, NexusCmd)> {
        loop {
            match self.sync() {
                Ok(_) => {},
                Err(e) => if e.kind() == ErrorKind::InvalidInput { continue } else { return Err(e); },
            }
            let addr = match self.read_addr() {
                Ok(addr) => addr,
                Err(e) => if e.kind() == ErrorKind::InvalidInput { continue } else { return Err(e); },
            };
            let cmd = match self.read_cmd() {
                Ok(cmd) => cmd,
                Err(e) => if e.kind() == ErrorKind::InvalidInput { continue } else { return Err(e); },
            };
            return Ok((addr, cmd));
        }
    }
}

/// Bit-level symbol writer for generating messages in Revo RF protocol.
pub struct SymWriter<'a, W: io::Write> {
    writer: BitWriter<&'a mut W, BigEndian>,
}

impl<'a, W: io::Write> SymWriter<'a, W> {
    /// Constructs new SymWriter wrapped around [`io::Write`] trait.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use nexus_revo_io::{SymWriter, NexusCmd};
    /// # use libftd2xx::{Ft232h, Ftdi};
    /// # use libftd2xx_cc1101::CC1101;
    /// # use std::convert::TryInto;
    /// # use std::io::Write;
    /// # let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// # let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// # let mut cc1101 = CC1101::new(&mut ftdi);
    /// # let mut cc1101_writer = cc1101.writer::<32>();
    /// let mut sym_writer = SymWriter::new(&mut cc1101_writer);
    /// ```
    pub fn new(writer: &'a mut W) -> Self {
        Self {
            writer: BitWriter::endian(writer, BigEndian),
        }
    }

    fn preamble(&mut self) -> io::Result<()> {
        self.writer.write_bytes(&[0xe8, 0xe8, 0xe8, 0xe8])
    }

    fn sync(&mut self) -> io::Result<()> {
        self.writer.write(7, 0x40)
    }

    fn terminate(&mut self) -> io::Result<()> {
        self.writer.write(7, 0x7f)?;
        self.writer.write_bytes(&[0_u8; 43])?;
        self.writer.byte_align()
    }

    fn write_byte(&mut self, b: u8) -> io::Result<()> {
        self.writer.write_bytes(&[
            if (b >> 7) & 1 != 0 { 0xe0 } else { 0x80 } | if (b >> 6) & 1 != 0 { 0xe } else { 0x8 },
            if (b >> 5) & 1 != 0 { 0xe0 } else { 0x80 } | if (b >> 4) & 1 != 0 { 0xe } else { 0x8 },
            if (b >> 3) & 1 != 0 { 0xe0 } else { 0x80 } | if (b >> 2) & 1 != 0 { 0xe } else { 0x8 },
            if (b >> 1) & 1 != 0 { 0xe0 } else { 0x80 } | if b & 1 != 0 { 0xe } else { 0x8 },
        ])
    }

    fn write_addr(&mut self, addr: u16) -> io::Result<()> {
        for b in addr.to_be_bytes() {
            self.write_byte(b)?;
        }
        Ok(())
    }

    fn write_cmd(&mut self, cmd: NexusCmd) -> io::Result<()> {
        self.write_byte(((cmd as u8) << 4) | 0b1000 | (cmd as u8))
    }

    /// Sends a Revo message with requisite 150ms delay at end.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use nexus_revo_io::{SymWriter, NexusCmd};
    /// # use libftd2xx::{Ft232h, Ftdi};
    /// # use libftd2xx_cc1101::CC1101;
    /// # use std::convert::TryInto;
    /// # use std::io::Write;
    /// # let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// # let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// # let mut cc1101 = CC1101::new(&mut ftdi);
    /// # let mut cc1101_writer = cc1101.writer::<32>();
    /// # let mut sym_writer = SymWriter::new(&mut cc1101_writer);
    /// sym_writer.write_msg(0x6969, NexusCmd::VibrateMode).expect("bad write_msg");
    /// ```
    pub fn write_msg(&mut self, addr: u16, cmd: NexusCmd) -> io::Result<()> {
        self.preamble()?;
        self.sync()?;
        for _ in 0..3 {
            self.write_addr(addr)?;
            self.write_cmd(cmd)?;
            self.sync()?;
        }
        self.terminate()?;
        self.writer.flush()
    }
}
