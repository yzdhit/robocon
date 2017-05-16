
//��λ�������д�淶
#ifndef ROBOCON_PC_ARM
#define ROBOCON_PC_ARM
#include "HITCRT_RobotTypes.h"
#include "PC_Comm_Structs.h"
#include "HITCRT_Types.h"
#include "stdlib.h"


#ifndef NULL
#define NULL 0
#endif

#define HEADERBYTE 0x55 // ����λ��ͨ�ŵ�Э��涨��ͨ���������ֽڹ̶�Ϊ0x55
#define TAILBYTE   0xAA  // ����λ��ͨ��Э��涨��ͨ���������һ���ֽڹ̶�Ϊ0xAA

#define 	COMM_STATUS_IDLE	(0) //  ͨ��״̬��IDLE
#define   COMM_STATUS_HEADER_FOUND  (1) // ͨ��״̬�� �Ѿ��ҵ�ͨ��Э��ͷ0x55
#define  	COMM_STATUS_ID_FOUND   (2) // ͨ��״̬�� �Ѿ��ҵ�����ID
#define  	COMM_BUF_LENGTH   (512) //����ͨ�Ż�������С������СΪ4*12 = 48 �Ļ�����


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
	UCHAR8 ucID;  // �ñ�����ID
	void * pVarAddress;  // �ṹ������׵�ַ
	UINT32 uiLength;// �Խṹ�������sizeof�Ĵ�С
	//ST_COMM_OBJ * pNextVar // ָ����һ�������ĵ�ַ
	//ST_COMM_OBJ * pNextVar
	void * pNextVar;
}ST_COMM_OBJ;  // ͨ�Ŷ�����Ϣ�ṹ��


typedef struct
{
	ST_COMM_OBJ * pCommObjHeader ; // ָ��ͨ�Ŷ����б���׵�ַ
	UINT32 uiObjCount ; // ͨ�Ŷ�����
}ST_OBJ_CENTER; // ͨ�Ŷ����������

ST_OBJ_CENTER stObjCenter = {NULL , 0};  //�������ݹ�������ʵ��
UCHAR8 ucStatus = 0; // ��ʼ����
UCHAR8 ucCommRecCount = 0; // ���ݴ���ͨ��Э�����������CommBuf�������ڵ��ֽ���Ŀ
ST_COMM_OBJ* pstObj = NULL; // ��ǰ���ڽ��յĶ����ָ��;

UCHAR8 CommBuf[COMM_BUF_LENGTH];// ��������


UINT32 SendHeadByUART0(UCHAR8 ucID)
{
	/****************************************************************************************************
		�꺯�����ƣ�SendHeadByUART0(UCHAR8 ucID)
		�������ܣ�����ID��ͨ��Э�齫Э��ͷ���͵���λ��
		��ڲ�����ucID:������ID
	****************************************************************************************************/
	SendByteByUART0(HEADERBYTE);
	SendByteByUART0(ucID);
	return 0;
}

UINT32 SendTailByUART0(void)
{
	/****************************************************************************************************
		�꺯�����ƣ�SendHeadByUART0(UCHAR8 ucID)
		�������ܣ�����ID��ͨ��Э�齫Э��ͷ���͵���λ��
		��ڲ�����
	****************************************************************************************************/
	SendByteByUART0(0x00);
	SendByteByUART0(TAILBYTE);
	return 0;
	
}



UINT32 SendHeadByUART1(UCHAR8 ucID)
{
	/****************************************************************************************************
		�꺯�����ƣ�SendHeadByUART0(UCHAR8 ucID)
		�������ܣ�����ID��ͨ��Э�齫Э��ͷ���͵���λ��
		��ڲ�����ucID:������ID
	****************************************************************************************************/
	SendByteByUART1(HEADERBYTE);
	SendByteByUART1(ucID);
	return 0;
}

UINT32 SendTailByUART1(void)
{
	/****************************************************************************************************
		�꺯�����ƣ�SendHeadByUART0(UCHAR8 ucID)
		�������ܣ�����ID��ͨ��Э�齫Э��ͷ���͵���λ��
		��ڲ�����
	****************************************************************************************************/
	SendByteByUART1(0x00);
	SendByteByUART1(TAILBYTE);
	return 0;
	
}

UINT32 SendVariableByUARTR0(UCHAR8 * pVar , UINT32 uiLength)
{
	/****************************************************************************************************
		�꺯�����ƣ�SendVariableByUARTR0(UCHAR8 * pVar , UINT32 uiLength)
		�������ܣ���������������λ��
		��ڲ�����pVar:�����׵�ַ
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
		�꺯�����ƣ�SendVariableByUARTR1(UCHAR8 * pVar , UINT32 uiLength)
		�������ܣ���������������λ��
		��ڲ�����pVar:�����׵�ַ
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
		�꺯�����ƣ�SendStruct(UCHAR8 ucID , void * pVarAdd ,  UINT32 uiLength)
		�������ܣ����������ID���ṹ���׵�ַ���ṹ�峤�Ƚ����ڽṹ�巢�͵���λ��
		��ڲ�����ucID����ͨ�Ŷ����ID 
				  pVarAdd, ͨ�Ŷ�����׵�ַ
				  uiLength, ��ͨ�Ŷ�����sizeof�Ĵ�С
		****************************************************************************************************/
		UINT32 i = 0;
		SendByteByUART0(HEADERBYTE); //����Э��ͷ���������ֽڣ�
		SendByteByUART0(ucID);
		for ( i = 0 ; i < uiLength ; i++)
		{
			SendByteByUART0(* (( (unsigned char *)pVarAdd ) + i) ); // ���η��͸����ֽ�
		}
		SendByteByUART0(0x00); //����Э��β���������ֽڣ�
		SendByteByUART0(TAILBYTE);
		return 0;
	}
	
	UINT32 SendStruct_2(ST_COMM_OBJ * pObj)
	{
		/****************************************************************************************************
		�꺯�����ƣ�SendStruct_2(ST_COMM_OBJ * pObj)
		�������ܣ����������ͨ�Ŷ�����Ϣ���ͽṹ�嵽��λ��
		��ڲ�����ST_COMM_OBJ ����Ҫ���Ͷ�����ص�ͨ����Ϣ�ṹ��
		****************************************************************************************************/
		UINT32 i = 0;
		UCHAR8 * pTmp =(UCHAR8 *)(pObj->pVarAddress) ;
		SendByteByUART0(HEADERBYTE); //����Э��ͷ���������ֽڣ�
		SendByteByUART0(pObj->ucID);		
		for ( i = 0 ; i < pObj->uiLength ; i++)
		{
			SendByteByUART0(  *( pTmp + i) ); // ���η��͸����ֽ�
		}
		SendByteByUART0(0x00); //����Э��β���������ֽڣ�
		SendByteByUART0(TAILBYTE);
		return 0;
	}
	


void CreateCommObj(UCHAR8 ucID , void * pAdd , UINT32 uiLength) // ������PCͨ�ŵ����ݶ���
{
		/****************************************************************************************************
		�������ƣ�CreateCommObj(UCHAR8 ucID , void * pAdd , UINT32 uiLength)
		�������ܣ������ṹ����󣬽��½��Ľṹ�������ӵ��ṹ���������ʵ��
		��ڲ�����ucID : ����Ӷ����ID
				  pAdd: ����Ӷ���ṹ����׵�ַ
				  uiLength:����Ӷ���ṹ����sizeof()�Ĵ�С
		****************************************************************************************************/
		
	//���һ���¶���
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

void Define_Comm_Objs (UCHAR8 ucID , void* pVar , UINT32 uiLength) //����ͨ�Ŷ���
{
	/****************************************************************************************************
		�������ƣ�Define_Comm_Objs(UCHAR8 ucID , void * pAdd , UINT32 uiLength)
		�������ܣ��ṩ��ʹ���ߵĺ����������ṹ����󣬽��½��Ľṹ�������ӵ��ṹ���������ʵ��
		��ڲ�����ucID : ����Ӷ����ID
				  pAdd: ����Ӷ���ṹ����׵�ַ
				  uiLength:����Ӷ���ṹ����sizeof()�Ĵ�С
	****************************************************************************************************/
	
	CreateCommObj(ucID , pVar , uiLength);
	return;
}


//�жϺ���
void  CommuInterrupt_Irq(void)
{
	/****************************************************************************************************
		�������ƣ�Interrupt_Irq(void)
		�������ܣ����ڴ�������λ��PC֮�����ͨ�ŵĴ����жϺ���
		��ڲ�����
	****************************************************************************************************/
	
	UINT32 i = 0;
	UCHAR8 ucTmp = 0;

	ST_COMM_OBJ * pObj = NULL;
	
 
	
	for ( i = 0 ; i < 1 ; i++)
	{
		ucTmp = USART_ReceiveData(USART1) ;  //�Ӵ��ڽ���һ���ֽ�
		
		switch (ucStatus)
		{
			//case ��COMM_STATUS_IDLE: //����״̬
			case COMM_STATUS_IDLE:
				if(ucTmp == HEADERBYTE)  // ���յ���ʼ�ֽ�
				{
					ucStatus = COMM_STATUS_HEADER_FOUND;
					CommBuf[0] = ucTmp;
					ucCommRecCount = 1;					
				}
				break;
			//case  COMM_STATUS_HEADER_FOUND: //�Ѿ����յ�Э��ͷ	
			case COMM_STATUS_HEADER_FOUND:
					//���Ҹ�ID�Ƿ����
					pObj = stObjCenter.pCommObjHeader;
						while(pObj != NULL)
						{
							if(pObj->ucID == ucTmp) //�Ƿ�Ϊ�ö��� 
							{
								CommBuf[1] = ucTmp;					
								ucCommRecCount = 2;
								ucStatus = COMM_STATUS_ID_FOUND;
								pstObj = pObj;
								break;
							}
							pObj = pObj->pNextVar; //�Ƶ���һ��
						}

						if(pObj == NULL)
							{
								// û���ҵ�ƥ���ID��˵��֮ǰ��Header����Ч��
								//ucStatus =COMM_STATUS_IDLE;
								ucStatus = COMM_STATUS_IDLE;
								ucCommRecCount = 0;
							}
						
					break;
			//case   COMM_STATUS_ID_FOUND:  // �Ѿ��ҵ�ID����Ҫ�����㹻��������н���				
			case COMM_STATUS_ID_FOUND:
				CommBuf[ucCommRecCount] = ucTmp;
				ucCommRecCount ++;
				if(ucCommRecCount == pstObj->uiLength + 4)
				{
					// ���ݽ�����ȫ
					//У��������ȷ��
					if(CommBuf[pstObj->uiLength + 4 - 2] == 0x00 && CommBuf[pstObj->uiLength + 4 - 1] == TAILBYTE)
					{
						// ������ȷ
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
						// ����У�鲻��ȷ��ȥ��Header�����½�������
						// ������
						ucStatus = COMM_STATUS_IDLE;
						ucCommRecCount = 0;
						pstObj = NULL;
					}
					break;
				}
				if(ucCommRecCount >= COMM_BUF_LENGTH)  // ����������
				{
					// ���ֹ��ϣ���ṹ�嶨�������⣬ֱ�Ӻ��Ըôζ���Ľ���
									
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
		�������ƣ�InitArm_PC(void)
		�������ܣ�Ϊ��λ����PC����ͨ�Ž��г�ʼ���� ʹ��UART0
		��ڲ�����uiBaund:���ڵĲ�����
	****************************************************************************************************/

	stObjCenter.pCommObjHeader = NULL;	  //��ʼ�������������
	stObjCenter.uiObjCount = 0;
	//InitUART0(uiBaund, (UINT32)Interrupt_Irq, 1);	  //���ô���0�������ͨ��
	return;
}

void InitArm_PC_UART1(void)
{
	/****************************************************************************************************
		�������ƣ�InitArm_PC(void)
		�������ܣ�Ϊ��λ����PC����ͨ�Ž��г�ʼ���� ʹ��UART0
		��ڲ�����uiBaund:���ڵĲ�����
	****************************************************************************************************/

	stObjCenter.pCommObjHeader = NULL;	  //��ʼ�������������
	stObjCenter.uiObjCount = 0;
	//InitUART1(uiBaund, (UINT32)Interrupt_Irq, 1);	  //���ô���0�������ͨ��
}




#endif
