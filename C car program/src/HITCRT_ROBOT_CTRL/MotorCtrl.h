#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H
#include "HITCRT_Types.h"
#include "HITCRT_RobotTypes.h"



#define AMQ 12//ִ�е��������

/****************************************************************************************************
��������:CopyFloatSlowly(FP32 * pfpDest, FP32 *pfpSrc, FP32 fpCopyStep)
��������: ������������ֹ�ٶȵȱ�����ͻ��
�������: pfpDest Ŀ�����ݵ�ָ��
			   pfpSrcԴ���ݵ�ָ��
			   ÿ�ο����Ĳ���
�������:��
��ע: ��Ҫ�������ò�����ȫcopy��ɡ�����õ������ݲ���
****************************************************************************************************/
extern void CopyFloatSlowly(FP32 * pfpDest, FP32 *pfpSrc, const FP32 fpCopyStep);
extern void CopyIntSlowly(SINT32 * psiDest, SINT32 * psiSrc,  const UINT32 puiCopyStep);
extern void CopyIntSlowlyEx(SSHORT16* psiDest, SINT32 siSrc,  const UINT32 puiCopyStep);
extern void CopyIntSlowlyEx1(SINT32* psiDest, SINT32 siSrc,  const UINT32 puiCopyStep);


/****************************************************************************************************
��������:CalVelt()
��������: �������̹����������Ӧ��ʱ������ٶ�
�������: pstVeltCoder�������ٶȵ����̽ṹ��
			   cuCoderchan��ʹ�õ�����ͨ��
			   fpCoffC2V��ʵ���ٶ������̹����ٶȵ�ת��ϵ��
�������: �����ٶ�
��ע: �ú����Ǹ���T���������̷����ٶ�
****************************************************************************************************/
extern FP32 CalVelt(ST_MOTOR_CTRL * pstRobotM);

/****************************************************************************************************
��������:VeltLoopCtrl()
��������: �Ե�������ٶȱջ�����
�������:  pstRobotM��Ҫ���ƵĻ������ϵĵ��
��ע: 
****************************************************************************************************/
extern void VeltLoopCtrl (ST_MOTOR_CTRL * pstRobotM);

/****************************************************************************************************
��������:PotLoopCtrl()
��������: �Ե������λ�ñջ����ƣ�λ�÷���������Ϊ����
�������:  pstRobotM��Ҫ���ƵĻ������ϵĵ��
��ע: 
****************************************************************************************************/
extern void PotLoopCtrl (ST_MOTOR_CTRL *pstRobotM);

/****************************************************************************************************
��������:OpenLoopCtrl()
��������: �����������
�������:  pstRobotM��Ҫ���Ƶĵ��
��ע: 
****************************************************************************************************/
extern void OpenLoopCtrl(ST_MOTOR_CTRL *pstRobotM);
/****************************************************************************************************
��������:AutoPotLoopCtrl(ST_MOTOR_CTRL *pstRobotM) 
��������: ���λ���Զ����ƣ������ٶȹ滮��λ�ñջ��������������ƣ�·���л��ȹ���
�������:  pstRobotM��Ҫ���ƵĻ����˵ĵ��
��ע: 
****************************************************************************************************/
extern void AutoPotLoopCtrl(ST_MOTOR_CTRL *pstRobotM);

extern void CalPosPID(ST_PID * pstPid);

extern void PotDoubleLoopCtrl(ST_MOTOR_CTRL * pstMotor);

/****************************************************************************************************
��������: DiffAutoPotLoopCtrl(ST_MOTOR_CTRL * pstMotor)
��������: ���λ���Զ����ƣ��ٶȹ滮��λ�ñջ�������������·���л�
�������:  pstMotor��Ҫ���ƵĻ����˵ĵ��
��ע: 
****************************************************************************************************/

extern void DiffAutoPotLoopCtrl(ST_MOTOR_CTRL * pstMotor);


/****************************************************************************************************
��������:DiffAutoVeltDis(ST_MOTOR_CTRL *pstMotor)
��������: ��Ӧ���ٵ��̵ĵ���ٶȷ��䣬��Ҫ�ǰ��սǶȷ��䣬�����ǰ����������
�������:  pstMotor��ִ�е����ָ��
��ע: ע���Ǵ�0-  AMQ-1
****************************************************************************************************/
extern void DiffAutoVeltDis(ST_MOTOR_CTRL *pstMotor);

extern void SetPathEndPosEx(ST_MOTOR_CTRL *pMotor,ST_ACTION_PATH * pPath,FP32 siEndPos);
extern void SetPathEndPosOnly(ST_MOTOR_CTRL *pMotor, ST_ACTION_PATH * pPath,SINT32 siEndPos);
extern void SetPathEndPosWithVEx2(ST_MOTOR_CTRL *pMotor,ST_ACTION_PATH * pPath,FP32 siEndPos);


#endif

