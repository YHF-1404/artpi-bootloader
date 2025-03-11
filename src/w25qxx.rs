use embedded_hal::digital::v2::OutputPin;
use std::io::{Read, Write};
use std::time::Duration;
use std::thread;

pub struct W25qxx<Spidev, CS> {
    spi: Spidev,
    cs: CS,
}
const W25QXX_MANID_VALUE: u8  = 0xEF;

/** Device ID */
const W25QXX_DEVID_VALUE_128: u8 = 0x17; /* 128Mbit */

const W25QXX_PAGE_SIZE: usize = 256;

/* Constants */
const W25QXX_SECTOR_SIZE: usize = 4 * 1024; /* 4K */
const W25QXX_BLOCK32K_SIZE: usize = 32 * 1024; /* 32K */
const W25QXX_BLOCK64K_SIZE: usize = 64 * 1024; /* 64K */

/// Easily readable representation of the command bytes used by the flash chip.
#[repr(u8)]
enum Command {
    Jedec = 0x90,
    PageProgram = 0x02,
    ReadData = 0x03,
    FastRead = 0x0B,
    ReadStatusRegister1 = 0x05,
    ReadStatusRegister2 = 0x35,
    WriteEnable = 0x06,
    SectorErase = 0x20,
    Block32Erase = 0x52,
    Block64Erase = 0xD8,
    ChipErase = 0xC7,
    EnableReset = 0x66,
    Reset = 0x99,
}

enum StatusRegister {
    Busy = 0x01,
    WriteEnable = 0x02,
}

#[derive(Debug)]
pub enum Error<E> {
    SPIError(E),
}

impl<Spidev, CS> W25qxx<Spidev, CS>
where
    Spidev: Write + Read,
    CS: OutputPin,
{
    pub fn new(spi: Spidev, cs: CS) -> Result<W25qxx<Spidev, CS> , Error<()>> {
        let mut flash = W25qxx { spi, cs };
        
        let _ = flash.cs.set_high();

        Ok(flash)
    }

    pub fn init(&mut self) -> Result<(), Error<()>> {
        self.read_jedec_register()?;

        println!("W25QXX - Identification OK");

        self.reset()?;

        println!("W25QXX - Reset OK");
        println!("W25QXX - Initialized OK");

        Ok(())
    }

    pub fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), Error<()>> {
        self.fast_read(address, buffer)
    }
    
    pub fn write(&mut self, address: u32, buffer: &[u8]) -> Result<(), Error<()>> {
        /* Write size 1 Page */
        let page_size: usize = W25QXX_PAGE_SIZE; /* 256 Bytes */
        let mut size = buffer.len();
        let mut offset: usize = 0;
        let mut addr:u32 = address;

        while size > 0 {
            /* 1.- Compute number of bytes we can write before reaching end of page */
            let mut write_size: usize = page_size - (addr as usize % page_size);

            /* 2.- If number of bytes to reach end of page is greater than size, reduce to size */
            if size < write_size {
                write_size = size;
            }
            /* 3.- Execute write command */
            self.busy_wait();

            /* 4.- Execute write command */
            let _= self.page_program(addr, &buffer[offset..(offset + write_size)]);

            /* 5.- Update the offset and the remaining size */
            offset += write_size;
            size -= write_size;
            addr += write_size as u32;
        }

        Ok(())
    }
    
    pub fn erase(&mut self, address: u32, len: usize) -> Result<(), Error<()>>  {
        let u_end:u32 = address + len as u32;
        let mut size:usize = len;
        let mut addr:u32 = address;

        /* Check alignment to 512 */
        if ((addr % W25QXX_SECTOR_SIZE as u32) != 0) || ((len % W25QXX_SECTOR_SIZE) != 0) {
            return Err(Error::SPIError(()));
        }
    
        /* Loop until everything is erased  */
        while addr < u_end {
            let bytes_erase = size;

            /* Erase 64K (64K Block) */
            if ((addr % W25QXX_BLOCK64K_SIZE as u32) == 0) && (bytes_erase >= W25QXX_BLOCK64K_SIZE) {
                self.busy_wait();
                self.erase_cmd(addr, Command::Block64Erase as u8)?;
                size -= W25QXX_BLOCK64K_SIZE;
                addr += W25QXX_BLOCK64K_SIZE as u32;
            }
            /* Erase 32K (32K Block) */
            else if ((addr % W25QXX_BLOCK32K_SIZE as u32) == 0) && (bytes_erase >= W25QXX_BLOCK32K_SIZE) {
                self.busy_wait();
                self.erase_cmd(addr, Command::Block32Erase as u8)?;
                size -= W25QXX_BLOCK32K_SIZE;
                addr += W25QXX_BLOCK32K_SIZE as u32;
            }
            /* Erase 4K (Sector) */
            else if ((addr % W25QXX_SECTOR_SIZE as u32) == 0) && (bytes_erase >= W25QXX_SECTOR_SIZE) {
                self.busy_wait();
                self.erase_cmd(addr, Command::SectorErase as u8)?;
                size -= W25QXX_SECTOR_SIZE;
                addr += W25QXX_SECTOR_SIZE as u32;
            } else {
                /* Error, not aligned erase (we should never reach this point) */
                return Err(Error::SPIError(()));
            }
        }
    
        Ok(())
    }

    #[allow(dead_code)]
    pub fn chip_erase(&mut self) -> Result<(), Error<()>> {
        /* Check the BUSY bit and the SUS bit in Status Register
        * before issuing the Reset command sequence */
        /* Note: Not checking suspend, as it will not be used within this driver */
        self.busy_wait();

        /* Before Erase, write enable latch */
        self.write_enable()?;

        let mut tx_cmd: [u8; 1] = [Command::ChipErase as u8];

        self.spi_transmit_and_receive(&mut tx_cmd, &mut [], 0)
    }

    fn read_jedec_register(&mut self) -> Result<(), Error<()>> {
        let mut tx_cmd: [u8; 4] = [0; 4];
        let mut rx_buffer: [u8; 2] = [0; 2];

        tx_cmd[0] = Command::Jedec as u8;

        let _ = self.spi_transmit_and_receive(&mut tx_cmd, &mut rx_buffer, 0);

        if rx_buffer[0] != W25QXX_MANID_VALUE || rx_buffer[1] != W25QXX_DEVID_VALUE_128 {
            return Err(Error::SPIError(()));
        }

        println!("W25QXX - Manufacture ID: 0x{:02X}", rx_buffer[0]);
        println!("W25QXX - Device ID: 0x{:02X}", rx_buffer[1]);

        Ok(())
    }

    fn reset(&mut self) -> Result<(), Error<()>> {
        self.busy_wait();
        self.spi.write(&[Command::EnableReset as u8]).unwrap();
        self.spi.write(&[Command::Reset as u8]).unwrap();
        Ok(())
    }

    fn read_status_register(&mut self, reg_num: u8) -> Result<u8, Error<()>> {
        let mut tx_cmd: [u8; 1] = [0; 1];
        let mut rx_buffer: [u8; 1] = [0; 1];
        if reg_num == 1 {
            tx_cmd[0] = Command::ReadStatusRegister1 as u8;
        } else if reg_num == 2 {
            tx_cmd[0] = Command::ReadStatusRegister2 as u8;
        } else {
            return Err(Error::SPIError(()));
        }

        self.spi_transmit_and_receive(&mut tx_cmd, &mut rx_buffer, 0)?;

        Ok(rx_buffer[0])
    }

    fn is_busy(&mut self) -> Result<bool, Error<()>> {
        Ok((self.read_status_register(1).unwrap() & StatusRegister::Busy as u8) != 0)
    }

    fn busy_wait(&mut self)  {
        while self.is_busy().expect("Error read status register") {
            thread::sleep(Duration::from_millis(1));
        }
    }

    fn is_write_enable(&mut self) -> bool {
        // Leer el registro de estado
        let status: u8 = self.read_status_register(1).unwrap();

        // Comprobar si el bit de Write Enable está establecido
        (status & StatusRegister::WriteEnable as u8) != 0
    }

    fn write_enable(&mut self) -> Result<(), Error<()>> {
        let mut tx_cmd: [u8; 1] = [Command::WriteEnable as u8];

        self.spi_transmit_and_receive(&mut tx_cmd, &mut [], 0)?;

        if !self.is_write_enable() {
            return Err(Error::SPIError(()));
        }

        Ok(())
    }

    fn spi_transmit(&mut self, cmd: u8, address: u32, tx_buffer: &[u8]) -> Result<(), Error<()>> {
        let mut tx_cmd: [u8; 4] = [0; 4];

        tx_cmd[0] = cmd;
        tx_cmd[1] = ((address >> 16) & 0xFF) as u8;
        tx_cmd[2] = ((address >> 8) & 0xFF) as u8;
        tx_cmd[3] = ((address) & 0xFF) as u8;

        /* Chip select low */
        let _ = self.cs.set_low();

        /* Send Command */
        let write_result = self.spi.write(&mut tx_cmd);

        /* Send Bytes */
        match write_result {
            Ok(size) => {
                // Comprobar si se escribió algún byte
                if size > 0 && !tx_buffer.is_empty() {
                    // Solo escribir el buffer si tiene tamaño
                    self.spi.write(tx_buffer).unwrap();
                }
            }
            Err(_e) => {
                return Err(Error::SPIError(()));
            }
        }

        /* Chip select high */
        let _ = self.cs.set_high();

        Ok(())
    }

    fn spi_transmit_and_receive(&mut self, tx_buffer: &mut [u8], rx_buffer: &mut [u8], dummy_bytes: usize) -> Result<(), Error<()>> {
        /* Chip select low */
        let _ = self.cs.set_low();

        /* Send Bytes */
        if !tx_buffer.is_empty() {
            self.spi.write(tx_buffer).unwrap();
        }

        /* Send Bytes */
        if dummy_bytes > 0 {
            let dummy_buffer: [u8; 1] = [0x00; 1];
            self.spi.write(&dummy_buffer).unwrap();
        }

        // Receive bytes
        if !rx_buffer.is_empty() {
            self.spi.read(rx_buffer).unwrap();
        }

        /* Chip select high */
        let _ = self.cs.set_high();

        Ok(())
    }

    fn page_program(&mut self, address: u32, tx_buffer: &[u8]) -> Result<(), Error<()>> {
        /* Argument check */
        if tx_buffer.is_empty() || tx_buffer.len() == 0 || tx_buffer.len() > W25QXX_PAGE_SIZE {
            return Err(Error::SPIError(()));
        }

        /* Before program enable write enable latch */
        self.write_enable()?;

        self.spi_transmit(Command::PageProgram as u8, address, tx_buffer)
    }

    #[allow(dead_code)]
    fn slow_read(&mut self, address: u32, rx_buffer: &mut [u8]) -> Result<(), Error<()>> {
        let mut tx_cmd: [u8; 4] = [0; 4];

        if rx_buffer.is_empty() || rx_buffer.len() == 0 {
            return Err(Error::SPIError(()));
        }

        tx_cmd[0] = Command::ReadData as u8;
        tx_cmd[1] = ((address >> 16) & 0xFF) as u8;
        tx_cmd[2] = ((address >> 8) & 0xFF) as u8;
        tx_cmd[3] = ((address) & 0xFF) as u8;

        self.spi_transmit_and_receive(&mut tx_cmd, rx_buffer, 0)
    }

    fn fast_read(&mut self, address: u32, rx_buffer: &mut [u8]) -> Result<(), Error<()>> {
        /* Argument check */
        let mut tx_cmd: [u8; 4] = [0; 4];

        if rx_buffer.is_empty() || rx_buffer.len() == 0 {
            return Err(Error::SPIError(()));
        }

        tx_cmd[0] = Command::FastRead as u8;
        tx_cmd[1] = ((address >> 16) & 0xFF) as u8;
        tx_cmd[2] = ((address >> 8) & 0xFF) as u8;
        tx_cmd[3] = ((address) & 0xFF) as u8;

        self.spi_transmit_and_receive(&mut tx_cmd, rx_buffer, 1)
    }

    fn erase_cmd(&mut self, address: u32, cmd: u8) -> Result<(), Error<()>>  {
        /* Before Erase enable write enable latch */
        self.write_enable()?;

        self.spi_transmit(cmd, address, &[])
    }
}