#ifndef SD_H
#define SD_H
#include "diskio.h"
#include "tff.h"
#define NULL 0
extern FATFS fs;
extern FIL fil;
extern FRESULT res;		//文件系统返回信息
extern u32 len;
extern void TurnToSD(void);
extern void WriteWord(u16 Word);//向SD卡写一个16位数
extern void TestSD(void);

#endif
