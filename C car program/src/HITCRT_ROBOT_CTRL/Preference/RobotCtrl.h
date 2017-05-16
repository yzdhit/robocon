#ifndef ROBOT_CTRL_H
#define ROBOT_CTRL_H

//M与C通信的命令
#define MC_UP_CMD             0xc1
#define MC_DOWN_CMD       0xc2
#define MC_OUT_CMD          0xc3
#define MC_IN_CMD             0xc4
#define MC_CATCH_CMD      0xc5
#define MC_RELEASE_CMD   0xc6
#define MC_ROTFIT_CMD     0xc7
#define MC_ROTOPST_CMD  0xc8
#define MC_AUTO_CMD        0xc9
#define MC_ROT_VALVE_CMD        0xca

#define MC_INIT_CMD                 0xcf
#define MC_PICKUP_CMD            0xce

//A与C通信命令
#define AC_AC_LEAVE_START_CMD    0xa1
#define AC_C_L2_CMD    0xa2
#define AC_A_BASKET_CMD    0xa3
#define AC_C_LIFTUP_CMD    0xa4
#define AC_AC_COMSUCCESS_CMD    0xa5

//气阀相关
#define ROT_VALVE                     0             
#define ROT_SWITCH_VALVE      1
#define TOP_CATCH_VALVE        2
#define MID_CATCH_VALVE        4
#define MID_LIFTUP_VALVE       6
#define FRONT_LEG_VALVE        3
#define BACK_LEG_VALVE          5

//开关相关
#define FRONT_LEG_SWITCH               7
#define FRONT_LEG_GOL2_SWITCH    0
#define BACK_LEG_SWITCH_FRONT     8
#define BACK_LEG_SWITCH_BACK       3
#define LEFT_LIGHT_SWITCH              2
#define RIGHT_LIGHT_SWITCH            1
#define LEFT_POS_SWITCH                  5
#define RIGHT_POS_SWITCH                6
#define TOP_ROTAIRPOS_SWITCH        15
#define TOP_ROTAIRREVERSEPOS_SWITCH        14


//巡线相关
#define LINE_CHAN_0                       0


//模式相关
#define C_MODE_INIT_0                                 0
#define C_MODE_TEST_SENSOR_1                  1
#define C_MODE_TEST_CODER_2                    2
#define C_MODE_TEST_LINE_3                       3
#define C_MODE_CAL_GYRO_4                        4
#define C_MODE_MANUAL_5                           5
#define C_MODE_LOW_SPEED_6                     6
#define C_MODE_MID_SPEED_7                      7
#define C_MODE_TOP_SPEED_8                      8        


extern FP32 g_fpModifyAngle;
extern FP32 g_fpGyroAngle;
extern SSHORT16 g_ssMemsGryo;
extern UINT32 g_uiVisionUpdateTime;
extern UCHAR8  g_ucVisionUsedCnt;


#endif

