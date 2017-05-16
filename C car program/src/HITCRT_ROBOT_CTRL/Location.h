#include "HITCRT_Types.h"
#include "HITCRT_RobotTypes.h"
//要用的宏

#define OMNI_CODER_A  9
#define OMNI_CODER_B  4

//A B 随动轮的夹角，弧度
#define ALPHA_A_B        1.5707963
#define SIN_ALPHA_A_B   1

// A 随动轮与 X  轴的夹角
#define ALPHA_A       0.78539815

//辅助角度+ PI
//#define PI_ALPHA_A    3.92699075
#define PI_ALPHA_A    ((180+46)*3.1415926/180)

//#define PI_ALPHA_A_ALPHA_A_B  5.49778705
#define PI_ALPHA_A_ALPHA_A_B  ((180+90+46)*3.1415926/180)

#define BETA_1      (1 * 3.1415926 / 180)
#define BETA_A_B  (90.3 * 3.1415926 / 180)


//#define CTR_TO_CTR   156.2f
#define CTR_TO_CTR   167.5f
//#define COFF_CODERA_TO_LENGTH  0.163079815f
//#define COFF_CODERB_TO_LENGTH  0.163079815f

#define COFF_CODERA_TO_LENGTH  0.1606858f

#define COFF_CODERB_TO_LENGTH  0.1606858f



/****************垂直布置相关参数******************/
#define COFF_OMNI_A  1.0f
#define COFF_OMNI_B  1.0f
#define COFF_OMNI_Q  1.00050050264403f
#define COFF_OMNI_NEWQ  -1.00546215f



#define DELTA_ANGLE_VERTICAL 2.360557773f 

/****************120°布置相关参数******************/
#define COFF_2_3        0.666666667f
#define COFF_ROBOT_3R   718.8//1029//3*343

#define CODER_TO_DEG     159.1549431f    // 1000/(2*PI)，随动轮码盘线数转化为角度系数，1000为码盘线数  
#define OMNI_WHEEL_R     25.4f           // 随动轮半径
#define COFF_PULSE_TO_LENGTHA  0.156417112f     //简化计算 OMNI_WHEEL_R/CODER_TO DEG 合成一个量
#define COFF_PULSE_TO_LENGTHB  0.156417112f     //简化计算 OMNI_WHEEL_R/CODER_TO DEG 合成一个量

#define DELTA_ANGLE_TRIANGULAR  0.001471427f//0.010495357f

#define COFF_2CENTER_DIS 69.284//390.7045//196.163
#define COFF_2CENTER_ANGLE 0.167254//0.33788


#define USE_MEMS_GRY0 0
/*******************************************************************
函数名称：DoubleVerticalOMNINav()
函数功能：利用双全向轮随动轮求出机器人全场相对坐标_模型2
输入：      
输出：    无
备注：   
********************************************************************/
extern void DoubleVerticalOMNILocate(ST_DOUBLE_OMNI_LOCATION *stDoubleOMNI , ST_OMNI_MOBILE_ROBOT *stRobot);
/*******************************************************************
函数名称：DoubleTriangularOMNILocate()
函数功能：利用120度布置的双随动轮求出机器人全局坐标
输入：      
输出：    无
备注：    每次计算时，三轮速度和角速度与上一时刻相比（理解成)不变，再
          利用当前航向角计算出绝度速度，此速度也（理解成）不变，（实际
          上，此速度用上一次的航向角计算，肯定不同，但此算法忽略其影响）。
          最后，对所有的速度表达式两端积分，便可得到以下公式。
***************************** ***************************************/
extern void DoubleTriangularOMNILocate(ST_DOUBLE_OMNI_LOCATION *pstDoubleOMNI , ST_OMNI_MOBILE_ROBOT *pstRobot);


extern void DoubleVerticalOMNILocateEx1(ST_DOUBLE_OMNI_LOCATION * pstDoubleOMNI, ST_OMNI_MOBILE_ROBOT * pstRobot);

extern void DoubleVerticalOMNILocatenew(ST_DOUBLE_OMNI_LOCATION *stDoubleOMNI , ST_OMNI_MOBILE_ROBOT *stRobot);




