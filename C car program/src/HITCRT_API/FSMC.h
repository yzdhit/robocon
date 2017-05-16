#ifndef FSMC_H
#define FSMC_H
#define BANK1_SRAM3_ADDR    ((UINT32)0x68000000)
//#define BANK1_SRAM4_ADDR    (UINT32*)0x6c000000
//#define FSMC_SRAM_Write(WriteAddr,Data)       *(vu16*)(BANK1_SRAM3_ADDR + (WriteAddr << 10)) = Data 
//#define FSMC_SRAM_READ(ReadAddr,Data)         (*(vu16*)(BANK1_SRAM3_ADDR + (ReadAddr << 10)))

extern void FSMC_Configuration(void);
//extern u16 FPGA_Read(u16 WriteAddr);
//extern void FPGA_Write(u16 pBuffer, u16 WriteAddr);
#endif
