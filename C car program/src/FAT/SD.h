#ifndef SD_H
#define SD_H
#include "diskio.h"
#include "tff.h"
#define NULL 0
extern FATFS fs;
extern FIL fil;
extern FRESULT res;		//�ļ�ϵͳ������Ϣ
extern u32 len;
extern void TurnToSD(void);
extern void WriteWord(u16 Word);//��SD��дһ��16λ��
extern void TestSD(void);

#endif
