MEMORY
{
  /* 内部 Flash，仅用于 Bootloader 代码 */
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K

  /* 外部 QSPI Flash，用于 Active 固件、DFU 升级和Bootloader状态 */
  QSPI_FLASH : ORIGIN = 0x90000000, LENGTH = 8M

  /* RAM */
  DTCM  : ORIGIN = 0x20000000, LENGTH = 128K  /* DTCM RAM */
  RAM   : ORIGIN = 0x24000000, LENGTH = 512K  /* AXI SRAM */
  ITCM  : ORIGIN = 0x00000000, LENGTH = 64K   /* ITCM RAM */
}

/* 内部 Flash 分区：
   - Bootloader 代码：128K (使用全部空间)
*/
__bootloader_code_start = ORIGIN(FLASH);
__bootloader_code_end   = __bootloader_code_start + 128K;

/* 外部 QSPI Flash 分区：
   - Active 固件：4M - 4K
   - DFU 固件：4M
   - Bootloader 状态：4K (放在最后位置)
*/
__bootloader_active_start = ORIGIN(QSPI_FLASH);
__bootloader_active_end   = __bootloader_active_start + (4M - 4K);

__bootloader_dfu_start    = __bootloader_active_end;
__bootloader_dfu_end      = __bootloader_dfu_start + 4M;

__bootloader_state_start  = __bootloader_dfu_end;
__bootloader_state_end    = __bootloader_state_start + 4K;