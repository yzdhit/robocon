#ifndef __HITCRT_ARM7_H

#define __HITCRT_ARM7_H
#include "HITCRT_Types.h"
//#include "Select.h"

/*******************************************************************
����˵����ARM״̬LED
��ע����
********************************************************************/ 			
#define TURN_OFF_ARM_LED()		//Ϩ��ARM_LED
#define TURN_ON_ARM_LED()		//����ARM_LED
/*ARM I2C�ӿ�*/
#if EN_I2C
extern UCHAR8 RcvByteFromI2C(UCHAR8 ucSla, UCHAR8 *pucDat);
extern UCHAR8 SendByteToI2C(UCHAR8 ucSla, UCHAR8 pucDat);
extern UCHAR8 ReadNByteFromI2C (UCHAR8 ucSla, UINT32 ucSubaType, UINT32 uiSuba, UCHAR8 *pucDataBuf, UINT32 uiNum);
extern UCHAR8 WriteNByteToI2C(UCHAR8 ucSla, UCHAR8 ucSubaType, UINT32 uiSuba, UCHAR8 *pucDataBuf, UINT32 uiNum);
extern void InitI2C(UINT32 uiBaud);
extern void __irq IrqI2C(void);
#endif

/*ARM  SPI���нӿ�*/
#if	EN_SPI
extern void __irq SPI0_IRQ(void);
extern void InitSPI0Master(void);
extern void InitSPI0Slave(void);
extern void SPI0Send2Byte(SSHORT16 ssData);
#endif

/*ARM  ��ʱ��*/
extern void InitTimer0(USHORT16 usFrq, UINT32 uiAddrIrq);
extern void InitTimer1(void);

/*ARM   UART�ӿ�*/
extern SINT32 InitUART0(UINT32 uiBaud,UINT32 uiAddrIrq,UCHAR8 ucTrigger);	
extern SINT32 InitUART1(UINT32 uiBaud,UINT32 uiAddrIrq,UCHAR8 ucTrigger);
extern void SendByteByUART0(UCHAR8 ucData);          //UART0����һ�ֽ�����
extern void SendByteByUART1(UCHAR8 ucData);          //UART1����һ�ֽ�����
extern void SendStrByUART0 (UCHAR8  *pucStr);
extern void SendStrByUART1 (UCHAR8  *pucStr);
extern void SendStructByUART0(UCHAR8 *pstTemp, UCHAR8 ucDataLength);
extern void SendStructByUART1(UCHAR8 *pstTemp, UCHAR8 ucDataLength);
extern void InitArmIO(void);

#endif