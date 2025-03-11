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
const W25QXX_ENTER_QSPI_MODE: u8 = 0x38;
const W25QXX_SET_READ_PARAMETERS: u8 = 0xC0; //QSPI mode only

const W25X_READ_STATUS_REG1: u8 = 0x05;
const W25X_READ_STATUS_REG2: u8 = 0x35;
const W25X_READ_STATUS_REG3: u8 = 0x15;

const W25X_WRITE_STATUS_REG1: u8 = 0x01; // Reg1 Reg2
const W25X_WRITE_STATUS_REG2: u8 = 0x01; // 0x31
const W25X_WRITE_STATUS_REG3: u8 = 0x01; // 0x11

const W25X_WRITE_ENABLE: u8 = 0x06;
const W25X_WRITE_DISABLE: u8 = 0x04;

// 添加W25QXX复位相关命令常量
const W25X_ENABLE_RESET: u8 = 0x66;
const W25X_RESET_DEVICE: u8 = 0x99;

// W25QXX读写相关命令
const W25X_FAST_READ_QUAD_IO: u8 = 0xEB; // 快速四线IO读取命令

// QE位掩码（用于检查四线模式使能）
const QE_MASK: u8 = 0x02; // 状态寄存器2中的QE位

// busy位掩码
const BUSY_MASK: u8 = 0x01;
/// 读取W25QXX的状态寄存器
/// 
/// - `srx`: 指定要读取的状态寄存器编号（1, 2, 3）
/// - `is_qpi_mode`: 当前是否处于QPI模式
/// 
fn w25qxx_read_sr(qspi: &mut Qspi<QUADSPI, Blocking>, srx: u8, is_qpi_mode: bool) -> u8 {
    // 选择要读取的状态寄存器指令
    let instruction = match srx {
        1 => W25X_READ_STATUS_REG1,
        2 => W25X_READ_STATUS_REG2,
        3 => W25X_READ_STATUS_REG3,
        _ => W25X_READ_STATUS_REG1, // 默认读取状态寄存器1
    };
    // 根据当前模式配置通信方式
    let (iwidth, dwidth) = if is_qpi_mode {
        (QspiWidth::QUAD, QspiWidth::QUAD)
    } else {
        (QspiWidth::SING, QspiWidth::SING)
    };

    let transaction = TransferConfig {
        iwidth,
        awidth: QspiWidth::NONE,
        dwidth,
        instruction,
        address: None,
        dummy: DummyCycles::_0,
    };

    // 创建接收缓冲区
    let mut data = [0u8; 1];
    
    // 执行读取操作
    qspi.blocking_read(&mut data, transaction);
    
    // 返回读取到的状态值
    data[0]
}

/// 写W25QXX的状态寄存器
/// 
/// - `srx`: 指定要写入的状态寄存器编号（1, 2, 3）
/// - `value`: 要写入的值
/// - `is_qpi_mode`: 当前是否处于QPI模式
/// 
/// 注意：写状态寄存器操作会触发内部写操作，需要等待写操作完成

fn w25qxx_write_sr(qspi: &mut Qspi<QUADSPI, Blocking>, srx: u8, value: u8, is_qpi_mode: bool) {
    // 选择要写入的状态寄存器指令
    let instruction = match srx {
        1 => W25X_WRITE_STATUS_REG1,
        2 => W25X_WRITE_STATUS_REG2,
        3 => W25X_WRITE_STATUS_REG3,
        _ => W25X_WRITE_STATUS_REG1, // 默认写状态寄存器1
    };
    // 根据当前模式配置通信方式
    let (iwidth, dwidth) = if is_qpi_mode {
        (QspiWidth::QUAD, QspiWidth::QUAD)
    } else {
        (QspiWidth::SING, QspiWidth::SING)
    };

        // 先发送写使能指令
    w25qxx_write_enable(qspi, is_qpi_mode);

    let transaction = TransferConfig {
        iwidth,
        awidth: QspiWidth::NONE,
        dwidth,
        instruction,
        address: None,
        dummy: DummyCycles::_0,
    };

    // 创建发送缓冲区
    let data = [value];
    
    // 执行写入操作
    qspi.blocking_write(&data, transaction);
}
/// 写使能
fn w25qxx_write_enable(qspi: &mut Qspi<QUADSPI, Blocking>, is_qpi_mode: bool) {
    let iwidth = if is_qpi_mode {
        QspiWidth::QUAD
    } else {
        QspiWidth::SING
    };
    
    let transaction = TransferConfig {
        iwidth,
        awidth: QspiWidth::NONE,
        dwidth: QspiWidth::NONE,
        instruction: W25X_WRITE_ENABLE,
        address: None,
        dummy: DummyCycles::_0,
    };
    
    qspi.blocking_command(transaction);
}

/// 写禁止
fn w25qxx_write_disable(qspi: &mut Qspi<QUADSPI, Blocking>, is_qpi_mode: bool) {
    let iwidth = if is_qpi_mode {
        QspiWidth::QUAD
    } else {
        QspiWidth::SING
    };
    
    let transaction = TransferConfig {
        iwidth,
        awidth: QspiWidth::NONE,
        dwidth: QspiWidth::NONE,
        instruction: W25X_WRITE_DISABLE,
        address: None,
        dummy: DummyCycles::_0,
    };
    
    qspi.blocking_command(transaction);
}

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
}

fn w25qxx_enter_qspi_mode(qspi: &mut Qspi<QUADSPI, Blocking>) {
    // 1. 检查并设置QE位（需要实现读写状态寄存器的函数）
    let status = w25qxx_read_sr(qspi, 2, false);
    if (status & QE_MASK) == 0 {
        // QE位未设置，需要设置
        w25qxx_write_sr(qspi, 2, status | QE_MASK, false);
    }

    // 2. 进入QSPI模式
    let transaction = TransferConfig {
        iwidth: QspiWidth::SING,
        awidth: QspiWidth::NONE,
        dwidth: QspiWidth::NONE,
        instruction: W25QXX_ENTER_QSPI_MODE,
        address: None,
        dummy: DummyCycles::_0,
    };
    qspi.blocking_command(transaction);
    // 3. 设置读参数
    let transaction = TransferConfig {
        iwidth: QspiWidth::QUAD,
        awidth: QspiWidth::NONE,
        dwidth: QspiWidth::QUAD,
        instruction: W25QXX_SET_READ_PARAMETERS,
        address: None,
        dummy: DummyCycles::_0,
    };

    // 先发送写使能指令（在QPI模式下）
    w25qxx_write_enable(qspi, true);
    let data: [u8; 1] = [0x03 << 4];
    qspi.blocking_write(&data, transaction);



}

fn is_w25qxx_write_busy(qspi: &mut Qspi<QUADSPI, Blocking>, is_qpi_mode: bool) -> bool {
    let status = w25qxx_read_sr(qspi, 1, is_qpi_mode);
    (status & 0x01) != 0
}

/// 此函数会使Flash回到默认SPI模式
fn w25qxx_reset(qspi: &mut Qspi<QUADSPI, Blocking>, is_qpi_mode: bool) {
    // 配置当前模式
    let iwidth = if is_qpi_mode {
        QspiWidth::QUAD
    } else {
        QspiWidth::SING
    };

    // 1. 发送使能复位命令
    let transaction = TransferConfig {
        iwidth,
        awidth: QspiWidth::NONE,
        dwidth: QspiWidth::NONE,
        instruction: W25X_ENABLE_RESET,
        address: None,
        dummy: DummyCycles::_0,
    };

    // 等待设备不忙
    while is_w25qxx_write_busy(qspi, is_qpi_mode) {}
    
    // 发送使能复位命令
    qspi.blocking_command(transaction);

    // 2. 发送复位命令
    let transaction = TransferConfig {
        iwidth,
        awidth: QspiWidth::NONE,
        dwidth: QspiWidth::NONE,
        instruction: W25X_RESET_DEVICE,
        address: None,
        dummy: DummyCycles::_0,
    };
    
    // 发送复位命令
    qspi.blocking_command(transaction);
    
    // 注意：复位后，设备将回到普通SPI模式，如果需要跟踪模式，应该更新状态
}

/// 启用W25Q Flash的内存映射模式
///
/// 这将允许CPU直接从外部Flash读取指令和数据，无需额外的传输命令
fn w25q_memory_mapped_enable(qspi: &mut Qspi<QUADSPI, Blocking>, is_qpi_mode: bool) {
    // 确保Flash处于QPI模式
    if !is_qpi_mode {
        w25qxx_enter_qspi_mode(qspi);
    }
    
    // 等待设备不忙
    while is_w25qxx_write_busy(qspi, true) {}
    
    // 在embassy-stm32库中，我们需要使用memory_mapped方法来配置并启用内存映射模式
    let config = TransferConfig {
        instruction: W25X_FAST_READ_QUAD_IO,
        iwidth: QspiWidth::QUAD,
        awidth: QspiWidth::QUAD,
        dwidth: QspiWidth::QUAD,
        address: Some(24), // 24位地址
        dummy: DummyCycles::_8,
        // SIOO模式表示是否每条指令都发送指令码
        // 在embassy-stm32中，可能没有直接对应选项，使用最接近的设置
    };
    
    // 启用内存映射模式
    qspi.enable_memory_map(&config);
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led = Output::new(p.PC15, Level::High, Speed::Low);
    let mut led1 = Output::new(p.PI8, Level::High, Speed::Low);
    // let ext_flash = embassy_stm32::qspi::Flash::new(qspi);
    // let dma = embassy_stm32::dma::NoDma;
    let config = embassy_stm32::qspi::Config{
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
    let mut qspi = Qspi::new_blocking_bank1(p.QUADSPI, p.PF8, p.PF9, p.PF7, p.PF6, p.PF10, p.PG6, config);

    let mut is_qpi_mode = false; // 跟踪当前模式
    w25qxx_exit_qspi_mode(&mut qspi);
    is_qpi_mode = false;
    w25qxx_reset(&mut qspi, is_qpi_mode);
    is_qpi_mode = false;
    w25qxx_enter_qspi_mode(&mut qspi);
    is_qpi_mode = true;
    w25q_memory_mapped_enable(&mut qspi, is_qpi_mode);
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

