#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::QUADSPI;
use embassy_stm32::qspi::{Qspi, TransferConfig};
use embassy_stm32::qspi::enums::{MemorySize, AddressSize, FIFOThresholdLevel, ChipSelectHighTime, QspiWidth, DummyCycles};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// pub struct TransferConfig {
//     /// Instruction width (IMODE)
//     pub iwidth: QspiWidth,
//     /// Address width (ADMODE)
//     pub awidth: QspiWidth,
//     /// Data width (DMODE)
//     pub dwidth: QspiWidth,
//     /// Instruction Id
//     pub instruction: u8,
//     /// Flash memory address
//     pub address: Option<u32>,
//     /// Number of dummy cycles (DCYC)
//     pub dummy: DummyCycles,
// }
const W25QXX_EXIT_QSPI_MODE: u8 = 0xFF;

fn w25qxx_exit_qspi_mode(qspi: &mut Qspi<QUADSPI, Blocking>) {
    let transaction = TransferConfig {
        iwidth: QspiWidth::QUAD,
        awidth: QspiWidth::NONE,
        dwidth: QspiWidth::NONE,
        instruction: W25QXX_EXIT_QSPI_MODE,
        address: None,
        dummy: DummyCycles::_0,
    };
    qspi.blocking_command(transaction);
    // qspi.write_enable().unwrap();
    // qspi.command(0xF5, &[], &mut []).unwrap();
}



#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led = Output::new(p.PC15, Level::High, Speed::Low);
    let mut led1 = Output::new(p.PI8, Level::High, Speed::Low);
    // let ext_flash = embassy_stm32::qspi::Flash::new(qspi);
    // let dma = embassy_stm32::dma::NoDma;
    let config = embassy_stm32::qspi::Config::{
    /// Flash memory size representend as 2^[0-32], as reasonable minimum 1KiB(9) was chosen.
    /// If you need other value the whose predefined use `Other` variant.
        memory_size: MemorySize::_8MiB,
    /// Address size (8/16/24/32-bit)
        address_size: AddressSize::_24bit,
    /// Scalar factor for generating CLK [0-255]
        prescaler: 1,
    /// Number of bytes to trigger FIFO threshold flag.
        fifo_threshold: FIFOThresholdLevel::_4Bytes,
    /// Minimum number of cycles that chip select must be high between issued commands
        cs_high_time: ChipSelectHighTime::_5Cycle,
    };
    let qspi = Qspi::new_blocking_bank1(p.QUADSPI, p.PF8, p.PF9, p.PF7, p.PF6, p.PF10, p.PG6, config);


    loop {
        info!("high");
        led.set_high();
        led1.set_high();
        Timer::after_millis(500).await;

        info!("low");
        led.set_low();
        led1.set_low();
        Timer::after_millis(500).await;
    }
}

