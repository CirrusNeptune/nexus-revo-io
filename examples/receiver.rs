use libftd2xx::{Ft232h, FtStatus, Ftdi, FtdiMpsse, MpsseSettings, TimeoutError};
use libftd2xx_cc1101::regs::{FilterLength, MagnTarget, ModFormat, SyncMode};
use libftd2xx_cc1101::CC1101;
use nexus_revo_io::SymReader;
use pretty_env_logger;
use std::convert::TryInto;
use std::time::Duration;
use tokio::net::TcpListener;
use tokio::sync::watch;
use tokio::io::{AsyncWriteExt, AsyncReadExt};

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

#[tokio::main]
pub async fn main() {
    pretty_env_logger::init();

    let ft = Ftdi::new().expect("unable to Ftdi::new");
    let mut ftdi: Ft232h = ft.try_into().expect("not a Ft232h");

    initialize_mpsse(&mut ftdi).expect("unable to initialize mpsse");

    let mut cc1101 = CC1101::new(&mut ftdi);
    cc1101
        .initialize(|regs, pa_table| {
            regs.set_freq(1093674); // 433.892 MHz
            regs.set_mdmcfg4(regs.mdmcfg4()
                .with_drate_e(0x7)
                .with_chanbw_e(3)
                .with_chanbw_m(3)); // 0x7,0x83
            regs.set_mdmcfg3(regs.mdmcfg3().with_drate_m(0x83)); // 2370 Baud 0x7e, 2400 0x83
            regs.set_mdmcfg2(
                regs.mdmcfg2()
                    .with_mod_format(ModFormat::AskOok)
                    .with_sync_mode(SyncMode::None),
            );
            regs.set_agcctrl2(regs.agcctrl2().with_magn_target(MagnTarget::Db33));
            regs.set_agcctrl1(regs.agcctrl1().with_agc_lna_priority(false));
            regs.set_agcctrl0(regs.agcctrl0().with_filter_length(FilterLength::Samples24));
            regs.set_fsctrl1(regs.fsctrl1().with_freq_if(0x6));
            *pa_table = [0, 0x60, 0, 0, 0, 0, 0, 0]; // 0dBm OOK
        })
        .expect("unable to initialize cc1101");

    let mut cc1101_reader = cc1101.reader::<32>();
    let mut sym_reader = SymReader::new(&mut cc1101_reader);

    let (tx, rx) = watch::channel::<u8>(0);

    tokio::spawn(async move {
        let addr = "127.0.0.1:8888";
        let listener = TcpListener::bind(addr).await.unwrap();
        println!("Listening on: {}", addr);
        loop {
            let (mut socket, socket_addr) = listener.accept().await.unwrap();
            let mut cloned_rx = rx.clone();
            tokio::spawn(async move {
                println!("Accepted on: {}", socket_addr);
                loop {
                    let msg = *cloned_rx.borrow_and_update();
                    if socket
                        .write_all(&[msg])
                        .await
                        .is_err() {
                        break;
                    }
                    let mut buf = [0_u8; 1];
                    tokio::select! {
                        rx_res = cloned_rx.changed() => {
                            if rx_res.is_err() {
                                break;
                            }
                        }
                        read_res = socket.read(&mut buf) => {
                            match read_res {
                                Err(_) | Ok(0) => {
                                    break
                                }
                                _ => {}
                            }
                        }
                    }
                }
                println!("Closed on: {}", socket_addr);
            });
        }
    });

    loop {
        let msg = sym_reader.read_msg().expect("bad read_msg");
        tx.send_replace(msg);
        match msg {
            0xe => println!("close"),
            0xa => println!("open"),
            _ => {}
        }
    }
}
