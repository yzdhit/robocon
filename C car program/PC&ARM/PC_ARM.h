
//下位机程序编写规范
#ifndef ROBOCON_PC_ARM
#define ROBOCON_PC_ARM
#include "HITCRT_RobotTypes.h"
#include "PC_Comm_Structs.h"
#include "HITCRT_Types.h"
#include "stdlib.h"


#ifndef NULL
#define NULL 0
#endif

#define HEADERBYTE 0x55 // 与上位机通信的协议规定的通信数据首字节固定为0x55
#define TAILBYTE   0xAA  // 与上位机通信协议规定的通信数据最后一个字节固定为0xAA

#define 	COMM_STATUS_IDLE	(0) //  通信状态：IDLE
#define   COMM_STATUS_HEADER_FOUND  (1) // 通信状态： 已经找到通信协议头0x55
#define  	COMM_STATUS_ID_FOUND   (2) // 通信状态： 已经找到对象ID
#define  	COMM_BUF_LENGTH   (512) //串口通信缓冲区大小，开大小为4*12 = 48 的缓冲区


#define   DECLARE_COMM_OBJS(ucID , pData)    (Define_Comm_Objs(ucID , &pData , sizeof(pData)))
//Define_Comm_Objs(uiRecData_ID , &uiRecData , sizeof(uiRecData));

#define  SEND_STRUCT_BY_UART0(ucID , pVar)    (SendStructToUART0(ucID, &pVar, sizeof(pVar)))
//SendStructToUART0(stMyTest_ID, &stMyTest, sizeof(stMyTest));
//#define  SEND_STRUCT_BY_UART1(ucID , pVar)    (SendStructToUART1(ucID, &pVar, sizeof(pVar)))

#define SEND_VARIABLE_BY_UART0(Var)  (SendVariableByUARTR0((UCHAR8*) &Var, sizeof(Var)))

#define SendByteByUART0(Data)  {USART_SendData(USART1, Data);\
									while(USART_GetFlagStatus(USART1 ,USART_FLAG_TC ) != SET);\
									USART_ClearFlag(USART1 ,USART_FLAG_TC);}
#define SendByteByUART1(Data)  {USART_SendData(USART2, Data);\
									while(USART_GetFlagStatus(USART2 ,USART_FLAG_TC ) != SET);\
									USART_ClearFlag(USART2 ,USART_FLAG_TC);}
typedef struct
{
	UCHAR8 ucID;  // 该变量的ID
	void * pVarAddress;  // 结构体变量首地址
	UINT32 uiLength;// 对结构体变量求sizeof的大小
	//ST_COMM_OBJ * pNextVar // 指向下一个变量的地址
	//ST_COMM_OBJ * pNextVar
	void * pNextVar;
}ST_COMM_OBJ;  // 通信对象信息结构体


typedef struct
{
	ST_COMM_OBJ * pCommObjHeader ; // 指向通信对象列表的首地址
	UINT32 uiObjCount ; // 通信对象技术
}ST_OBJ_CENTER; // 通信对象管理中心

ST_OBJ_CENTER stObjCenter = {NULL , 0};  //定义数据管理中心实例
UCHAR8 ucStatus = 0; // 初始空闲
UCHAR8 ucCommRecCount = 0; // 根据串口通信协议解析后存放于CommBuf缓冲区内的字节数目
ST_COMM_OBJ* pstObj = NULL; // 当前正在接收的对象的指针;

UCHAR8 CommBuf[COMM_BUF_LENGTH];// 开缓冲区


UINT32 SendHeadByUART0(UCHAR8 ucID)
{
	/****************************************************************************************************
		宏函数名称：SendHeadByUART0(UCHAR8 ucID)
		函数功能：根据ID和通信协议将协议头发送到上位机
		入口参数：ucID:变量的ID
	****************************************************************************************************/
	SendByteByUART0(HEADERBYTE);
	SendByteByUART0(ucID);
	return 0;
}

UINT32 SendTailByUART0(void)
{
	/****************************************************************************************************
		宏函数名称：SendHeadByUART0(UCHAR8 ucID)
		函数功能：根据ID和通信协议将协议头发送到上位机
		入口参数：
	****************************************************************************************************/
	SendByteByUART0(0x00);
	SendByteByUART0(TAILBYTE);
	return 0;
	
}



UINT32 SendHeadByUART1(UCHAR8 ucID)
{
	/****************************************************************************************************
		宏函数名称：SendHeadByUART0(UCHAR8 ucID)
		函数功能：根据ID和通信协议将协议头发送到上位机
		入口参数：ucID:变量的ID
	****************************************************************************************************/
	SendByteByUART1(HEADERBYTE);
	SendByteByUART1(ucID);
	return 0;
}

UINT32 SendTailByUART1(void)
{
	/****************************************************************************************************
		宏函数名称：SendHeadByUART0(UCHAR8 ucID)
		函数功能：根据ID和通信协议将协议头发送到上位机
		入口参数：
	****************************************************************************************************/
	SendByteByUART1(0x00);
	SendByteByUART1(TAILBYTE);
	return 0;
	
}

UINT32 SendVariableByUARTR0(UCHAR8 * pVar , UINT32 uiLength)
{
	/****************************************************************************************************
		宏函数名称：SendVariableByUARTR0(UCHAR8 * pVar , UINT32 uiLength)
		函数功能：将单个变量到上位机
		入口参数：pVar:变量首地址
	****************************************************************************************************/
	int i = 0;
	for(i = 0 ; i < uiLength ; i++)
		{
			SendByteByUART0(*(pVar + i));			
		}
	return 0;
	
}

UINT32 SendVariableByUARTR1(UCHAR8 * pVar , UINT32 uiLength)
{
	/****************************************************************************************************
		宏函数名称：SendVariableByUARTR1(UCHAR8 * pVar , UINT32 uiLength)
		函数功能：将单个变量到上位机
		入口参数：pVar:变量首地址
	****************************************************************************************************/
	int i = 0;
	for( i = 0 ; i < uiLength ; i++)
		{
			SendByteByUART1(*(pVar + i));			
		}
	return 0;
	
}

UINT32 SendStructToUART0(UCHAR8 ucID , void * pVarAdd ,  UINT32 uiLength)
	{
		/****************************************************************************************************
		宏函数名称：SendStruct(UCHAR8 ucID , void * pVarAdd ,  UINT32 uiLength)
		函数功能：根据输入的ID、结构体首地址、结构体长度将对于结构体发送到上位机
		入口参数：ucID，该通信对象的ID 
				  pVarAdd, 通信对象的首地址
				  uiLength, 对通信对象求sizeof的大小
		****************************************************************************************************/
		UINT32 i = 0;
		SendByteByUART0(HEADERBYTE); //发送协议头（共两个字节）
		SendByteByUART0(ucID);
		for ( i = 0 ; i < uiLength ; i++)
		{
			SendByteByUART0(* (( (unsigned char *)pVarAdd ) + i) ); // 依次发送各个字节
		}
		SendByteByUART0(0x00); //发送协议尾（共两个字节）
		SendByteByUART0(TAILBYTE);
		return 0;
	}
	
	UINT32 SendStruct_2(ST_COMM_OBJ * pObj)
	{
		/****************************************************************************************************
		宏函数名称：SendStruct_2(ST_COMM_OBJ * pObj)
		函数功能：根据输入的通信对象信息发送结构体到上位机
		入口参数：ST_COMM_OBJ ：与要发送对象相关的通信信息结构体
		****************************************************************************************************/
		UINT32 i = 0;
		UCHAR8 * pTmp =(UCHAR8 *)(pObj->pVarAddress) ;
		SendByteByUART0(HEADERBYTE); //发送协议头（共两个字节）
		SendByteByUART0(pObj->ucID);		
		for ( i = 0 ; i < pObj->uiLength ; i++)
		{
			SendByteByUART0(  *( pTmp + i) ); // 依次发送各个字节
		}
		SendByteByUART0(0x00); //发送协议尾（共两个字节）
		SendByteByUART0(TAILBYTE);
		return 0;
	}
	


void CreateCommObj(UCHAR8 ucID , void * pAdd , UINT32 uiLength) // 定义与PC通信的数据对象
{
		/****************************************************************************************************
		函数名称：CreateCommObj(UCHAR8 ucID , void * pAdd , UINT32 uiLength)
		函数功能：创建结构体对象，将新建的结构体对象添加到结构体管理中心实例
		入口参数：ucID : 欲添加对象的ID
				  pAdd: 欲添加对象结构体的首地址
				  uiLength:欲添加对象结构体求sizeof()的大小
		****************************************************************************************************/
		
	//添加一个新对象
	ST_COMM_OBJ * pObj = stObjCenter.pCommObjHeader;
	if(stObjCenter.pCommObjHeader == NULL)
		{
			pObj= (ST_COMM_OBJ*)malloc(sizeof(ST_COMM_OBJ));
			stObjCenter.pCommObjHeader = pObj;
			pObj->ucID = ucID;
			pObj->pVarAddress = pAdd;
			pObj->uiLength  = uiLength;
			pObj->pNextVar = NULL;
			return;
		}
	
	while(pObj->pNextVar != NULL)
	{
		pObj =(ST_COMM_OBJ*) pObj->pNextVar;
	}
	pObj->pNextVar = (ST_COMM_OBJ*)malloc(sizeof(ST_COMM_OBJ));
	pObj = (ST_COMM_OBJ*)pObj->pNextVar;
	pObj->ucID = ucID;
	pObj->pVarAddress = pAdd;
	pObj->uiLength = uiLength;
	pObj->pNextVar = NULL;
	stObjCenter.uiObjCount ++;
}

void Define_Comm_Objs (UCHAR8 ucID , void* pVar , UINT32 uiLength) //定义通信对象
{
	/****************************************************************************************************
		函数名称：Define_Comm_Objs(UCHAR8 ucID , void * pAdd , UINT32 uiLength)
		函数功能：提供给使用者的函数。创建结构体对象，将新建的结构体对象添加到结构体管理中心实例
		入口参数：ucID : 欲添加对象的ID
				  pAdd: 欲添加对象结构体的首地址
				  uiLength:欲添加对象结构体求sizeof()的大小
	****************************************************************************************************/
	
	CreateCommObj(ucID , pVar , uiLength);
	return;
}


//中断函数
void  CommuInterrupt_Irq(void)
{
	/****************************************************************************************************
		函数名称：Interrupt_Irq(void)
		函数功能：用于处理与上位机PC之间进行通信的串口中断函数
		入口参数：
	****************************************************************************************************/
	
	UINT32 i = 0;
	UCHAR8 ucTmp = 0;

	ST_COMM_OBJ * pObj = NULL;
	
 
	
	for ( i = 0 ; i < 1 ; i++)
	{
		ucTmp = USART_ReceiveData(USART1) ;  //从串口接收一个字节
		
		switch (ucStatus)
		{
			//case 　COMM_STATUS_IDLE: //空闲状态
			case COMM_STATUS_IDLE:
				if(ucTmp == HEADERBYTE)  // 接收到起始字节
				{
					ucStatus = COMM_STATUS_HEADER_FOUND;
					CommBuf[0] = ucTmp;
					ucCommRecCount = 1;					
				}
				break;
			//case  COMM_STATUS_HEADER_FOUND: //已经接收到协议头	
			case COMM_STATUS_HEADER_FOUND:
					//查找该ID是否定义过
					pObj = stObjCenter.pCommObjHeader;
						while(pObj != NULL)
						{
							if(pObj->ucID == ucTmp) //是否为该对象 
							{
								CommBuf[1] = ucTmp;					
								ucCommRecCount = 2;
								ucStatus = COMM_STATUS_ID_FOUND;
								pstObj = pObj;
								break;
							}
							pObj = pObj->pNextVar; //移到下一个
						}

						if(pObj == NULL)
							{
								// 没有找到匹配的ID，说明之前的Header是无效的
								//ucStatus =COMM_STATUS_IDLE;
								ucStatus = COMM_STATUS_IDLE;
								ucCommRecCount = 0;
							}
						
					break;
			//case   COMM_STATUS_ID_FOUND:  // 已经找到ID，需要填满足够的数后进行解析				
			case COMM_STATUS_ID_FOUND:
				CommBuf[ucCommRecCount] = ucTmp;
				ucCommRecCount ++;
				if(ucCommRecCount == pstObj->uiLength + 4)
				{
					// 数据接收完全
					//校验数据正确性
					if(CommBuf[pstObj->uiLength + 4 - 2] == 0x00 && CommBuf[pstObj->uiLength + 4 - 1] == TAILBYTE)
					{
						// 数据正确
						for(i = 0 ; i < pstObj->uiLength  ; i++)
						{
							*(( (UCHAR8 *)(pstObj->pVarAddress) ) + i ) = CommBuf[i+2];

						}
						ucStatus = COMM_STATUS_IDLE;
						ucCommRecCount = 0;
						pstObj = NULL;
						break;
					}
					else
					{
						// 数据校验不正确，去掉Header后重新解析数据
						// 待补充
						ucStatus = COMM_STATUS_IDLE;
						ucCommRecCount = 0;
						pstObj = NULL;
					}
					break;
				}
				if(ucCommRecCount >= COMM_BUF_LENGTH)  // 缓冲区已满
				{
					// 出现故障，或结构体定义有问题，直接忽略该次对象的接收
									
					ucCommRecCount = 0;
					ucStatus =COMM_STATUS_IDLE;
					pstObj = NULL;
					break;
				}
				break;
		}

	 
	}
	

}


void InitArm_PC_UART0(void)
{
	/****************************************************************************************************
		函数名称：InitArm_PC(void)
		函数功能：为上位机和PC进行通信进行初始化， 使用UART0
		入口参数：uiBaund:串口的波特率
	****************************************************************************************************/

	stObjCenter.pCommObjHeader = NULL;	  //初始化对象管理中心
	stObjCenter.uiObjCount = 0;
	//InitUART0(uiBaund, (UINT32)Interrupt_Irq, 1);	  //设置串口0，与电脑通信
	return;
}

void InitArm_PC_UART1(void)
{
	/****************************************************************************************************
		函数名称：InitArm_PC(void)
		函数功能：为上位机和PC进行通信进行初始化， 使用UART0
		入口参数：uiBaund:串口的波特率
	****************************************************************************************************/

	stObjCenter.pCommObjHeader = NULL;	  //初始化对象管理中心
	stObjCenter.uiObjCount = 0;
	//InitUART1(uiBaund, (UINT32)Interrupt_Irq, 1);	  //设置串口0，与电脑通信
}




#endif
