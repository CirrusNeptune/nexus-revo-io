use libftd2xx::{Ft232h, FtStatus, Ftdi, FtdiMpsse, MpsseSettings, TimeoutError};
use libftd2xx_cc1101::regs::{ModFormat, SyncMode};
use libftd2xx_cc1101::CC1101;
use nexus_revo_io::{NexusCmd, SymWriter};
use pretty_env_logger;
use std::convert::TryInto;
use std::thread;
use std::time::Duration;

fn initialize_mpsse<Ft: FtdiMpsse>(ftdi: &mut Ft) -> Result<(), TimeoutError> {
    const SETTINGS: MpsseSettings = MpsseSettings {
        reset: true,
        in_transfer_size: 4096,
        read_timeout: Duration::from_secs(1),
        write_timeout: Duration::from_secs(1),
        latency_timer: Duration::from_millis(16),
        mask: 0xb,
        clock_frequency: Some(1_000_000),
    };

    // Attempt initialize_mpsse up to 3 times
    let mut err = Err(TimeoutError::FtStatus(FtStatus::OTHER_ERROR));
    for _ in 0..3 {
        err = ftdi.initialize_mpsse(&SETTINGS);
        if err.is_ok() {
            return Ok(());
        }
    }
    err
}

pub fn main() {
    pretty_env_logger::init();

    let ft = Ftdi::new().expect("unable to Ftdi::new");
    let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");

    initialize_mpsse(&mut ftdi).expect("unable to initialize mpsse");

    let mut cc1101 = CC1101::new(&mut ftdi);
    cc1101
        .initialize(|regs, pa_table| {
            regs.set_freq(1093805); // 433.943634 MHz
            regs.set_mdmcfg3(regs.mdmcfg3().with_drate_m(0x67)); // 2.2254 kBaud
            regs.set_mdmcfg2(
                regs.mdmcfg2()
                    .with_mod_format(ModFormat::AskOok)
                    .with_sync_mode(SyncMode::None),
            );
            *pa_table = [0, 0x60, 0, 0, 0, 0, 0, 0]; // 0dBm OOK
        })
        .expect("unable to initialize cc1101");

    let mut cc1101_writer = cc1101.writer::<32>();
    let mut sym_writer = SymWriter::new(&mut cc1101_writer);

    const ADDR: u16 = 0xe45d;

    loop {
        for _ in 0..6 {
            sym_writer
                .write_msg(ADDR, NexusCmd::VibrateMode)
                .expect("unable to VibrateMode");
        }

        let ten_millis = Duration::from_millis(5000);
        thread::sleep(ten_millis);

        sym_writer
            .write_msg(ADDR, NexusCmd::VibrateOff)
            .expect("unable to VibrateOff");
    }
}
