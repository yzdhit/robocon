
#include "HITCRT_Types.h"
#include "HITCRT_RobotTypes.h"
#include "HITCRT_API.h"
#include "MotorConfig.h"



#define SET_NAV_PID_PARA(NAME)		\
{									\
	NAME##_ROT_PID_PARA,			\
	NAME##_TRVS_PID_PARA,			\
	NAME##_VTC_PID_PARA				\
}

#define SET_ONMI_ROBOT_PARA(WA, WB, WC)										\
{																			\
	SET_MOTOR_CTRL_PARA(WA, WA##_MOTOR_CHANNEL, WA##_CODER_CHANNEL),		\
	SET_MOTOR_CTRL_PARA(WB, WB##_MOTOR_CHANNEL, WB##_CODER_CHANNEL),		\
	SET_MOTOR_CTRL_PARA(WC, WC##_MOTOR_CHANNEL, WC##_CODER_CHANNEL),		\
	SET_NAV_PID_PARA(NAV_LINE),												\
	SET_NAV_PID_PARA(NAV_CIR),												\
	ROBOT_POT_INIT,															\
	ROBOT_POT_INIT,															\
	ROBOT_POT_SEN,															\
	ROBOT_VELT_DES_INIT,													\
	ROBOT_VELT_LIMIT,														\
	(ST_PATH*)NULL,															\
	(ST_PATH*)NULL,															\
	ROBOT_STOP,																\
	NAV_OFF,																\
	PATH_RUN,																\
	BASE_BREAK,																\
	ALL_BREAK,                                                                                                                        \
	BLUE_FIELD,                                                                                                              \
	MID_SPEED,                                                                                                           \
	REPOSIT_INIT	,															\
	SELFCHECK_INIT															\
}

//BASE_BREAK																\		

#define INSTANCE_ROBOT(stR,WA,WB,WC,WA_SEL,WB_SEL,WC_SEL)	\
CONFIG_MOTOR(WA,WA##_MOTOR_CHANNEL,WA_SEL)					\
CONFIG_MOTOR(WB,WB##_MOTOR_CHANNEL,WB_SEL)					\
CONFIG_MOTOR(WC,WC##_MOTOR_CHANNEL,WC_SEL)					
//ST_OMNI_MOBILE_ROBOT stR = 	SET_ONMI_ROBOT_PARA(WA, WB, WC)		



	



















