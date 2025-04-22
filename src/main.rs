
#![no_std]
#![no_main]

use defmt::info;
// use cortex_m::delay;
// use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
// use defmt::*;
// use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
// use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::QUADSPI;
use embassy_stm32::qspi::{Qspi, TransferConfig};
use embassy_stm32::qspi::enums::{MemorySize, AddressSize, FIFOThresholdLevel, ChipSelectHighTime, QspiWidth, DummyCycles};
// use embassy_time::Delay;
// use embassy_time::Timer;
// use embassy_time::*;

use core::cell::RefCell;
use cortex_m_rt::{entry, exception};
#[cfg(feature = "defmt")]
use defmt_rtt as _;

use embassy_boot_stm32::*;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash, ErrorType, NorFlashError, NorFlashErrorKind};
// use embassy_stm32::flash::{Flash, BANK1_REGION};
use embassy_sync::blocking_mutex::Mutex;
// use core::arch::asm;

// QSPI Flash在内存中的基地址
const QSPI_FLASH_BASE: u32 = 0x9000_0000;


// 命令常量保留在外部
const W25QXX_EXIT_QSPI_MODE: u8 = 0xFF;
const W25QXX_ENTER_QSPI_MODE: u8 = 0x38;
const W25QXX_SET_READ_PARAMETERS: u8 = 0xC0; //QSPI mode only

const W25X_READ_STATUS_REG1: u8 = 0x05;
const W25X_READ_STATUS_REG2: u8 = 0x35;
const W25X_READ_STATUS_REG3: u8 = 0x15;

const W25X_WRITE_STATUS_REG1: u8 = 0x01; // Reg1 Reg2
const W25X_WRITE_STATUS_REG2: u8 = 0x01; // 0x31
const W25X_WRITE_STATUS_REG3: u8 = 0x01; // 0x11

const W25X_PAGE_PROGRAM: u8 = 0x02;  // 单线页编程命令
const W25X_QUAD_PAGE_PROGRAM: u8 = 0x32;  // 四线页编程命令
const W25X_SECTOR_ERASE: u8 = 0x20;  // 4KB扇区擦除
const W25X_BLOCK_ERASE_32K: u8 = 0x52;  // 32KB块擦除
const W25X_BLOCK_ERASE_64K: u8 = 0xD8;  // 64KB块擦除
const W25X_READ_DATA: u8 = 0x03;  // 读数据命令

const W25X_WRITE_ENABLE: u8 = 0x06;
const W25X_WRITE_DISABLE: u8 = 0x04;

const W25X_ENABLE_RESET: u8 = 0x66;
const W25X_RESET_DEVICE: u8 = 0x99;

const W25X_FAST_READ_QUAD_IO: u8 = 0xEB; // 快速四线IO读取命令

const QE_MASK: u8 = 0x02; // 状态寄存器2中的QE位
const BUSY_MASK: u8 = 0x01;



#[derive(Debug)]
pub enum QspiError {
    NotAligned,
    OutOfBounds,
    WriteError,
    EraseError,
    ReadError,
    NotReady,
}

// 为错误类型实现NorFlashError trait
impl NorFlashError for QspiError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            QspiError::NotAligned => NorFlashErrorKind::NotAligned,
            QspiError::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            QspiError::WriteError | QspiError::EraseError | 
            QspiError::ReadError | QspiError::NotReady => NorFlashErrorKind::Other,
        }
    }
}

// 修改ErrorType实现
impl<'d> ErrorType for QspiFlashDriver<'d> {
    type Error = QspiError;  // 使用自定义错误类型
}





pub struct QspiFlashDriver<'d> {
    qspi: Qspi<'d, QUADSPI, Blocking>,
    base_address: u32,  // QSPI Flash映射的起始地址
    is_qpi_mode: bool,  // 当前是否处于QPI模式
}





impl<'d> QspiFlashDriver<'d> {
    pub fn new(qspi: Qspi<'d, QUADSPI, Blocking>, base_address: u32, is_qpi_mode: bool) -> Self {
        Self {
            qspi,
            base_address,
            is_qpi_mode,
        }
    }
    
    /// 写使能
    fn write_enable(&mut self) {
        let iwidth = if self.is_qpi_mode { QspiWidth::QUAD } else { QspiWidth::SING };
        let transaction = TransferConfig {
            iwidth,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::NONE,
            instruction: W25X_WRITE_ENABLE,
            address: None,
            dummy: DummyCycles::_0,
        };
        self.qspi.blocking_command(transaction);
    }
    
    /// 写禁止
    pub fn write_disable(&mut self) {
        let iwidth = if self.is_qpi_mode { QspiWidth::QUAD } else { QspiWidth::SING };
        
        let transaction = TransferConfig {
            iwidth,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::NONE,
            instruction: W25X_WRITE_DISABLE,
            address: None,
            dummy: DummyCycles::_0,
        };
        
        self.qspi.blocking_command(transaction);
    }
    
    /// 判断Flash是否处于写忙状态
    pub fn is_write_busy(&mut self) -> bool {
        let status = self.read_status_register(1);
        (status & BUSY_MASK) != 0
    }
    
    /// 等待写操作完成
    fn wait_busy(&mut self) {
        while self.is_write_busy() {}
    }
    
    /// 读取W25QXX的状态寄存器
    /// 
    /// - `reg_num`: 指定要读取的状态寄存器编号（1, 2, 3）
    pub fn read_status_register(&mut self, reg_num: u8) -> u8 {
        // 选择要读取的状态寄存器指令
        let instruction = match reg_num {
            1 => W25X_READ_STATUS_REG1,
            2 => W25X_READ_STATUS_REG2,
            3 => W25X_READ_STATUS_REG3,
            _ => W25X_READ_STATUS_REG1, // 默认读取状态寄存器1
        };
        
        // 根据当前模式配置通信方式
        let (iwidth, dwidth) = if self.is_qpi_mode {
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
        self.qspi.blocking_read(&mut data, transaction);
        
        // 返回读取到的状态值
        data[0]
    }
    
    /// 写W25QXX的状态寄存器
    /// 
    /// - `reg_num`: 指定要写入的状态寄存器编号（1, 2, 3）
    /// - `value`: 要写入的值
    pub fn write_status_register(&mut self, reg_num: u8, value: u8) {
        // 选择要写入的状态寄存器指令
        let instruction = match reg_num {
            1 => W25X_WRITE_STATUS_REG1,
            2 => W25X_WRITE_STATUS_REG2,
            3 => W25X_WRITE_STATUS_REG3,
            _ => W25X_WRITE_STATUS_REG1, // 默认写状态寄存器1
        };
        
        // 根据当前模式配置通信方式
        let (iwidth, dwidth) = if self.is_qpi_mode {
            (QspiWidth::QUAD, QspiWidth::QUAD)
        } else {
            (QspiWidth::SING, QspiWidth::SING)
        };

        // 先发送写使能指令
        self.write_enable();

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
        self.qspi.blocking_write(&data, transaction);
        
        // 等待写入完成
        self.wait_busy();
    }
    
    /// 退出QPI模式
    pub fn exit_qpi_mode(&mut self) {
        let transaction = TransferConfig {
            iwidth: QspiWidth::QUAD,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::NONE,
            instruction: W25QXX_EXIT_QSPI_MODE,
            address: None,
            dummy: DummyCycles::_0,
        };
        self.qspi.blocking_command(transaction);
        self.is_qpi_mode = false;
    }
    
    /// 进入QPI模式
    pub fn enter_qpi_mode(&mut self) {
        // 1. 检查并设置QE位
        let status = self.read_status_register(2);
        info!("status: {:?}", status);
        if (status & QE_MASK) == 0 {
            // QE位未设置，需要设置
            self.write_status_register(2, status | QE_MASK);
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
        self.qspi.blocking_command(transaction);
        
        // 更新模式状态
        self.is_qpi_mode = true;
        
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
        self.write_enable();
        let data: [u8; 1] = [0x03 << 4];
        self.qspi.blocking_write(&data, transaction);
    }
    
    /// 复位W25Q Flash芯片
    /// 此函数会使Flash回到默认SPI模式
    pub fn reset(&mut self) {
        // 配置当前模式
        let iwidth = if self.is_qpi_mode {
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
        self.wait_busy();
        
        // 发送使能复位命令
        self.qspi.blocking_command(transaction);

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
        self.qspi.blocking_command(transaction);
        
        // 复位后，设备将回到普通SPI模式
        self.is_qpi_mode = false;
    }
    
    /// 启用W25Q Flash的内存映射模式
    /// 这将允许CPU直接从外部Flash读取指令和数据
    pub fn enable_memory_mapped(&mut self) {
        // 确保Flash处于QPI模式
        if !self.is_qpi_mode {
            self.enter_qpi_mode();
        }
        
        // 等待设备不忙
        self.wait_busy();
        
        // 配置并启用内存映射模式
        let config = TransferConfig {
            instruction: W25X_FAST_READ_QUAD_IO,
            iwidth: QspiWidth::QUAD,
            awidth: QspiWidth::QUAD,
            dwidth: QspiWidth::QUAD,
            address: Some(24), // 24位地址
            dummy: DummyCycles::_8,
        };
        
        // 启用内存映射模式
        self.qspi.enable_memory_map(&config);
    }
}



impl<'d> ReadNorFlash for QspiFlashDriver<'d> {
    const READ_SIZE: usize = 1;  // 最小读取单位（字节）
    
    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        // 检查参数是否有效
        if (offset as usize + bytes.len()) > (self.capacity() + self.base_address as usize) {
            return Err(QspiError::OutOfBounds);
        }
        // 由于已启用内存映射，可以直接从映射的内存地址读取
        // let src_ptr = (self.base_address + offset) as *const u8;
        let src_ptr = offset as *const u8;
        for (i, byte) in bytes.iter_mut().enumerate() {
            unsafe {
                *byte = core::ptr::read_volatile(src_ptr.add(i));
            }
        }
        Ok(())
    }
    
    fn capacity(&self) -> usize {
        8 * 1024 * 1024  // 假设8MB容量，根据实际调整
    }
}

impl<'d> NorFlash for QspiFlashDriver<'d> {
    const WRITE_SIZE: usize = 1;  // 最小写入单位（字节）
    const ERASE_SIZE: usize = 4096;  // 最小擦除单位（4KB扇区）
    
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        // 检查参数有效性
        if from > to || (to as usize) > self.capacity() {
            return Err(QspiError::OutOfBounds);
        }
    
        // 计算需要擦除的区域
        let start_sector = from / Self::ERASE_SIZE as u32;
        let end_sector = to / Self::ERASE_SIZE as u32;
        
        // 逐扇区擦除
        for sector in start_sector..=end_sector {
            let sector_addr = sector * Self::ERASE_SIZE as u32;
            
            // 写使能
            self.write_enable();
            
            // 发送扇区擦除命令
            let iwidth = if self.is_qpi_mode { QspiWidth::QUAD } else { QspiWidth::SING };
            let transaction = TransferConfig {
                iwidth,
                awidth: if self.is_qpi_mode { QspiWidth::QUAD } else { QspiWidth::SING },
                dwidth: QspiWidth::NONE,
                instruction: W25X_SECTOR_ERASE,
                address: Some(sector_addr),
                dummy: DummyCycles::_0,
            };
            
            self.qspi.blocking_command(transaction);
            
            // 等待擦除完成
            self.wait_busy();
        }
        
        Ok(())
    }
    
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        // 检查参数...
        if (offset as usize + bytes.len()) > self.capacity() {
            return Err(QspiError::OutOfBounds);
        }

        // W25Q系列Flash通常使用页编程，每页256字节
        const PAGE_SIZE: usize = 256;
        
        let mut remaining = bytes.len();
        let mut src_offset = 0;
        let mut dst_offset = offset;
        
        while remaining > 0 {
            // 计算当前页内可写入的字节数
            let current_page = (dst_offset as usize) / PAGE_SIZE;
            let bytes_in_page = ((current_page + 1) * PAGE_SIZE) - (dst_offset as usize);
            let to_write = core::cmp::min(remaining, bytes_in_page);
            
            // 写使能
            self.write_enable();
            
            // 发送页编程命令
            let iwidth = if self.is_qpi_mode { QspiWidth::QUAD } else { QspiWidth::SING };
            let cmd = if self.is_qpi_mode { W25X_QUAD_PAGE_PROGRAM } else { W25X_PAGE_PROGRAM };
            let transaction = TransferConfig {
                iwidth,
                awidth: if self.is_qpi_mode { QspiWidth::QUAD } else { QspiWidth::SING },
                dwidth: if self.is_qpi_mode { QspiWidth::QUAD } else { QspiWidth::SING },
                instruction: cmd,
                address: Some(dst_offset),
                dummy: DummyCycles::_0,
            };
            
            // 写入数据
            self.qspi.blocking_write(&bytes[src_offset..(src_offset + to_write)], transaction);
            
            // 等待写入完成
            self.wait_busy();
            
            // 更新计数器
            remaining -= to_write;
            src_offset += to_write;
            dst_offset += to_write as u32;
        }
        
        Ok(())
    }
}






#[entry]
fn main() -> ! {
// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(25_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV5,
            mul: PllMul::MUL192,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV2),
            divr: Some(PllDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_P; // 480 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 240 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 120 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 120 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 120 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 120 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale0;

        // 禁用时间驱动(TIM2)
        // config.rcc.tim2 = Some(embassy_stm32::rcc::Tim::Disabled);
    }


    let p = embassy_stm32::init(config);

    defmt::info!("ART-PI bootloader!");

    let config = embassy_stm32::qspi::Config{
        memory_size: MemorySize::_8MiB,
        address_size: AddressSize::_24bit,
        prescaler: 1,
        fifo_threshold: FIFOThresholdLevel::_4Bytes,
        cs_high_time: ChipSelectHighTime::_5Cycle,
    };
    let mut qspi = Qspi::new_blocking_bank1(p.QUADSPI, p.PF8, p.PF9, p.PF7, p.PF6, p.PF10, p.PG6, config);



    // 创建QSPI Flash驱动实例
    let mut qspi_driver = QspiFlashDriver::new(qspi, 0x9000_0000, false);

    // 重置Flash并初始化
    qspi_driver.exit_qpi_mode();
    qspi_driver.reset();
    qspi_driver.enter_qpi_mode();
    qspi_driver.enable_memory_mapped();

    // 共享驱动
    let qspi_flash = Mutex::new(RefCell::new(qspi_driver));

    info!("qspi_flash");

    let config = BootLoaderConfig::from_linkerfile_blocking(&qspi_flash, &qspi_flash, &qspi_flash);
    info!("config");

    let active_offset = config.active.offset();
    info!("active_offset: {:08x}", active_offset);


    // 在跳转前验证QSPI Flash内容
    let target_addr = active_offset;
    info!("目标跳转地址: 0x{:08x}", target_addr);

// 验证向量表的前几个关键值
unsafe {
    // 读取栈指针(第一个向量表项)
    let stack_ptr = core::ptr::read_volatile(target_addr as *const u32);
    // 读取复位处理程序地址(第二个向量表项)
    let reset_handler = core::ptr::read_volatile((target_addr + 4) as *const u32);
    
    info!("栈指针: 0x{:08x}", stack_ptr);
    info!("复位处理程序: 0x{:08x}", reset_handler);
    
    // 简单验证固件有效性
    if stack_ptr < 0x2000_0000 || stack_ptr > 0x3000_0000 || 
       reset_handler < target_addr || reset_handler > (target_addr + 0x100000) {
        defmt::error!("无效的固件向量表!");
        // 启动失败处理
        loop {
            cortex_m::asm::nop();
        }
    }
    // 1. 完全禁用缓存 (不仅仅是失效)
    let mut p = cortex_m::Peripherals::steal();
    p.SCB.invalidate_icache();
    cortex_m::asm::dsb();
    p.SCB.disable_icache();
    p.SCB.disable_dcache(&mut p.CPUID);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // 2. 禁用SysTick
    let mut stk = cortex_m::Peripherals::steal().SYST;
    stk.set_reload(0);
    stk.clear_current();
    stk.disable_counter();
    
    // 其他系统状态清理
    // 可能需要禁用其他中断
    cortex_m::interrupt::disable();





    }  
    // 跳转前添加小延迟以确保配置生效
    for _ in 0..1000 {
        cortex_m::asm::nop();
    }

    let bl = BootLoader::prepare::<_, _, _, 2048>(config);

    // 跳转到固件
    unsafe { bl.load(target_addr) }

    loop {

    }

}


// #[unsafe(no_mangle)]
// #[cfg_attr(target_os = "none", unsafe(link_section = ".HardFault.user"))]
// unsafe extern "C" fn HardFault() {
//     cortex_m::peripheral::SCB::sys_reset();
// }

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
    let irqn = core::ptr::read_volatile(SCB_ICSR) as u8 as i16 - 16;

    panic!("DefaultHandler #{:?}", irqn);
}

#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    // 打印出关键寄存器的值以帮助调试
    defmt::error!("硬fault异常发生!");
    defmt::error!("异常发生地址: 0x{:08x}", ef.pc());
    defmt::error!("lr=0x{:08x}, r0=0x{:08x}, r1=0x{:08x}", ef.lr(), ef.r0(), ef.r1());
    defmt::error!("r2=0x{:08x}, r3=0x{:08x}, r12=0x{:08x}", ef.r2(), ef.r3(), ef.r12());
    defmt::error!("xPSR=0x{:08x}", ef.xpsr());
    
    // 检查是否有更多硬故障信息(仅Cortex-M4/M7有此功能)
    let scb = cortex_m::peripheral::SCB::ptr();
    let cfsr = unsafe { (*scb).cfsr.read() };
    if cfsr != 0 {
        defmt::error!("CFSR=0x{:08x}", cfsr);
        if cfsr & 0x8000 != 0 { 
            defmt::error!("- 内存管理错误(MMFSR): 0x{:02x}", cfsr as u8); 
        }
        if cfsr & 0x80_0000 != 0 { 
            defmt::error!("- 总线错误(BFSR): 0x{:02x}", (cfsr >> 8) as u8); 
        }
        if cfsr & 0x800_0000 != 0 { 
            defmt::error!("- 用法错误(UFSR): 0x{:04x}", (cfsr >> 16) as u16); 
        }
    }
    // loop {
    //     cortex_m::asm::nop();
    // }
    // 系统复位，避免卡死
    cortex_m::peripheral::SCB::sys_reset();
}


#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::error!("发生panic: {}", defmt::Display2Format(_info));
    
    // 输出栈回溯的一些信息
    let lr: u32;
    unsafe { core::arch::asm!("mov {}, lr", out(reg) lr) };
    defmt::error!("LR寄存器值: 0x{:08x}", lr);
    cortex_m::asm::udf();
}
