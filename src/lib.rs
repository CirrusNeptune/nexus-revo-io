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

use std::fs::File;
use bitstream_io::{BigEndian, BitRead, BitReader, BitWrite, BitWriter};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::io;
use std::io::{Error, ErrorKind, Read, Write};
use std::ops::DerefMut;
use libftd2xx::Ft232h;

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

/// Bit-level symbol reader for detecting and decoding messages in Revo RF protocol.
///
/// Designed to block and continuously scan for bit patterns containing coded symbols. Incoming
/// bytes do not need to be synchronized to sync words.
pub struct SymReader<R: io::Read> {
    reader: BitReader<R, BigEndian>,
    window: u8,
    bit_counter: u32,
    raw_counter: u32,
    raw_csv: File,
    parsed_bit: bool,
    parsed_msg: bool,
}

impl<R: io::Read> SymReader<R> {
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
    pub fn new(reader: R) -> Self {
        let mut ret = Self {
            reader: BitReader::endian(reader, BigEndian),
            window: 0,
            bit_counter: 0,
            raw_counter: 0,
            raw_csv: File::create("/Users/cirrus/Desktop/raw.csv").unwrap(),
            parsed_bit: false,
            parsed_msg: false,
        };
        ret.raw_csv.write_fmt(format_args!("x,y,type\n")).unwrap();
        ret
    }

    fn read_bit(&mut self) -> io::Result<bool> {
        let bit = self.reader.read_bit()?;
        let mut rssi_dbm: Option<i32> = None;
        if let Some(r) = self.reader.reader() {
            unsafe {
                let reader: &mut &mut libftd2xx_cc1101::io::FifoReader<Ft232h, 32> = std::mem::transmute(r);
                let rssi_dec = reader.rssi().unwrap();
                let rssi_offset = 0;
                rssi_dbm = Some(if rssi_dec >= 128 {
                    (rssi_dec as i32 - 256)
                } else {
                    (rssi_dec as i32)
                });
            }
        }
        self.raw_counter += 1;
        /*
        self.raw_csv.write_fmt(format_args!("{},{},raw\n", self.raw_counter, bit as u32)).unwrap();
        self.raw_csv.write_fmt(format_args!("{},{},parsed\n", self.raw_counter, self.parsed_bit as u32 * 2)).unwrap();
        self.raw_csv.write_fmt(format_args!("{},{},msg\n", self.raw_counter, self.parsed_msg as u32 * 3)).unwrap();
        if let Some(r) = rssi_dbm {
            if (r % 2) != 0 {
                self.raw_csv.write_fmt(format_args!("{},{}.5,rssi\n", self.raw_counter, r / 2)).unwrap();
            } else {
                self.raw_csv.write_fmt(format_args!("{},{}.0,rssi\n", self.raw_counter, r / 2)).unwrap();
            }
        }
         */
        //println!("{} {}", self.raw_counter, bit as u32);
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
            return if one_count >= 5 {
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
                self.bit_counter += 1;
                self.parsed_bit = false;
                //println!("bit {} 0", self.bit_counter);
                Ok(false)
            }
            Symbol::One => {
                self.bit_counter += 1;
                self.parsed_bit = true;
                //println!("bit {} 1", self.bit_counter);
                Ok(true)
            }
            _ => Err(Error::new(ErrorKind::InvalidInput, "no data symbols")),
        }
    }

    fn read_byte(&mut self) -> io::Result<u8> {
        let mut value = 0;
        for i in (0..4).rev() {
            // TODO: make this a shift-only loop
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
        println!("cmd: {:2X}", byte);
        FromPrimitive::from_u8(byte)
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
    pub fn read_msg(&mut self) -> io::Result<u8> {
        let mut code_search = 0_u16;
        loop {
            let bit = self.read_bit_symbol()?;
            code_search <<= 1;
            code_search |= bit as u16;
            //println!("{:4X}", code_search);
            if code_search == 0x59CF {
                self.parsed_msg = true;
                let cmd_byte = self.read_byte()?;
                self.parsed_msg = false;
                return Ok(cmd_byte);
            }
        }
    }
}

/// Result of one [`SymReaderFsm::poll`] iteration.
pub enum SymReaderFsmPoll {
    /// Complete decoded message
    Msg((u16, NexusCmd)),
    /// I/O error
    Err(io::Error),
    /// One bit processed successfully
    Pending,
}

#[repr(u8)]
enum SymReaderFsmState {
    Sync,
    Addr,
    Cmd,
}

/// Finite State Machine version of [`SymReader`].
///
/// Processes one bit with each poll call. Useful for applications that do not want to block.
pub struct SymReaderFsm<R: io::Read> {
    reader: BitReader<R, BigEndian>,
    window: u8,
    counter: u8,
    decode_value: u16,
    addr: u16,
    state: SymReaderFsmState,
    read_until: bool,
}

impl<R: io::Read> SymReaderFsm<R> {
    /// Constructs new SymReaderFsm wrapped around [`io::Read`] trait.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use nexus_revo_io::SymReaderFsm;
    /// # use libftd2xx::{Ft232h, Ftdi};
    /// # use libftd2xx_cc1101::CC1101;
    /// # use std::convert::TryInto;
    /// # use std::io::Read;
    /// # let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// # let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// # let mut cc1101 = CC1101::new(&mut ftdi);
    /// # let mut cc1101_reader = cc1101.reader::<32>();
    /// let mut sym_reader = SymReaderFsm::new(&mut cc1101_reader);
    /// ```
    pub fn new(reader: R) -> Self {
        Self {
            reader: BitReader::endian(reader, BigEndian),
            window: 0,
            counter: 0,
            decode_value: 0,
            addr: 0,
            state: SymReaderFsmState::Sync,
            read_until: false,
        }
    }

    fn read_bit(&mut self) -> io::Result<bool> {
        let bit = self.reader.read_bit()?;
        self.window <<= 1;
        self.window |= bit as u8;
        Ok(bit)
    }

    fn check_sync_symbol(&self) -> bool {
        (self.window >> 1) & 0xf == 0x0
    }

    fn check_bit_symbol(&self) -> Option<bool> {
        match (self.window >> 1) & 0xf {
            0x8 => Some(false),
            0xe => Some(true),
            _ => None,
        }
    }

    fn handle_bit_symbol(&mut self) -> Option<bool> {
        self.counter -= 1;
        if let Some(bit) = self.check_bit_symbol() {
            self.decode_value |= (bit as u16) << self.counter;
            Some(self.counter == 0)
        } else {
            None
        }
    }

    fn decode_cmd(&self) -> Option<NexusCmd> {
        let b = self.decode_value as u8;
        if b & 0b1000 == 0 || b & 0x7 != (b >> 4) & 0x7 {
            None
        } else {
            FromPrimitive::from_u8(b & 0x7)
        }
    }

    fn start_sync(&mut self) {
        self.state = SymReaderFsmState::Sync;
    }

    fn start_addr(&mut self) {
        self.state = SymReaderFsmState::Addr;
        self.counter = 16;
        self.decode_value = 0;
    }

    fn start_cmd(&mut self) {
        self.state = SymReaderFsmState::Cmd;
        self.counter = 8;
        self.decode_value = 0;
    }

    fn poll_sync(&mut self) {
        if self.check_sync_symbol() {
            self.start_addr();
        } else {
            self.start_sync();
        }
    }

    fn poll_addr(&mut self) {
        if let Some(done) = self.handle_bit_symbol() {
            if done {
                self.addr = self.decode_value;
                self.start_cmd();
            }
        } else {
            self.poll_sync();
        }
    }

    fn poll_cmd(&mut self) -> SymReaderFsmPoll {
        if let Some(done) = self.handle_bit_symbol() {
            if done {
                self.start_sync();
                if let Some(cmd) = self.decode_cmd() {
                    return SymReaderFsmPoll::Msg((self.addr, cmd));
                }
            }
        } else {
            self.poll_sync();
        }

        SymReaderFsmPoll::Pending
    }

    /// Processes one bit of input, returning a valid message, an error, or pending. May be called
    /// repeatedly to continuously process incoming messages.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use nexus_revo_io::{SymReaderFsm, SymReaderFsmPoll};
    /// # use libftd2xx::{Ft232h, Ftdi};
    /// # use libftd2xx_cc1101::CC1101;
    /// # use std::convert::TryInto;
    /// # use std::io::Read;
    /// # let ft = Ftdi::new().expect("unable to Ftdi::new");
    /// # let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");
    /// # let mut cc1101 = CC1101::new(&mut ftdi);
    /// # let mut cc1101_reader = cc1101.reader::<32>();
    /// # let mut sym_reader = SymReaderFsm::new(&mut cc1101_reader);
    /// loop {
    ///     match sym_reader.poll() {
    ///         SymReaderFsmPoll::Msg(msg) => println!("{:x?} {:?}", msg.0, msg.1),
    ///         SymReaderFsmPoll::Err(e) => panic!("poll failure: {:?}", e),
    ///         SymReaderFsmPoll::Pending => {}
    ///     }
    /// }
    /// ```
    pub fn poll(&mut self) -> SymReaderFsmPoll {
        let bit = match self.read_bit() {
            Ok(bit) => bit,
            Err(e) => return SymReaderFsmPoll::Err(e),
        };

        if bit != self.read_until {
            return SymReaderFsmPoll::Pending;
        }

        self.read_until = !self.read_until;

        if !self.read_until {
            match self.state {
                SymReaderFsmState::Sync => self.poll_sync(),
                SymReaderFsmState::Addr => self.poll_addr(),
                SymReaderFsmState::Cmd => return self.poll_cmd(),
            }
        }

        SymReaderFsmPoll::Pending
    }
}

/// Bit-level symbol writer for generating messages in Revo RF protocol.
pub struct SymWriter<W: io::Write> {
    writer: BitWriter<W, BigEndian>,
}

impl<W: io::Write> SymWriter<W> {
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
    pub fn new(writer: W) -> Self {
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
        for b in addr.to_be_bytes().iter() {
            self.write_byte(*b)?;
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
