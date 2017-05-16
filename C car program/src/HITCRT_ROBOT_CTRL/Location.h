#include "HITCRT_Types.h"
#include "HITCRT_RobotTypes.h"
//Ҫ�õĺ�

#define OMNI_CODER_A  9
#define OMNI_CODER_B  4

//A B �涯�ֵļнǣ�����
#define ALPHA_A_B        1.5707963
#define SIN_ALPHA_A_B   1

// A �涯���� X  ��ļн�
#define ALPHA_A       0.78539815

//�����Ƕ�+ PI
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



/****************��ֱ������ز���******************/
#define COFF_OMNI_A  1.0f
#define COFF_OMNI_B  1.0f
#define COFF_OMNI_Q  1.00050050264403f
#define COFF_OMNI_NEWQ  -1.00546215f



#define DELTA_ANGLE_VERTICAL 2.360557773f 

/****************120�㲼����ز���******************/
#define COFF_2_3        0.666666667f
#define COFF_ROBOT_3R   718.8//1029//3*343

#define CODER_TO_DEG     159.1549431f    // 1000/(2*PI)���涯����������ת��Ϊ�Ƕ�ϵ����1000Ϊ��������  
#define OMNI_WHEEL_R     25.4f           // �涯�ְ뾶
#define COFF_PULSE_TO_LENGTHA  0.156417112f     //�򻯼��� OMNI_WHEEL_R/CODER_TO DEG �ϳ�һ����
#define COFF_PULSE_TO_LENGTHB  0.156417112f     //�򻯼��� OMNI_WHEEL_R/CODER_TO DEG �ϳ�һ����

#define DELTA_ANGLE_TRIANGULAR  0.001471427f//0.010495357f

#define COFF_2CENTER_DIS 69.284//390.7045//196.163
#define COFF_2CENTER_ANGLE 0.167254//0.33788


#define USE_MEMS_GRY0 0
/*******************************************************************
�������ƣ�DoubleVerticalOMNINav()
�������ܣ�����˫ȫ�����涯�����������ȫ���������_ģ��2
���룺      
�����    ��
��ע��   
********************************************************************/
extern void DoubleVerticalOMNILocate(ST_DOUBLE_OMNI_LOCATION *stDoubleOMNI , ST_OMNI_MOBILE_ROBOT *stRobot);
/*******************************************************************
�������ƣ�DoubleTriangularOMNILocate()
�������ܣ�����120�Ȳ��õ�˫�涯�����������ȫ������
���룺      
�����    ��
��ע��    ÿ�μ���ʱ�������ٶȺͽ��ٶ�����һʱ����ȣ�����)���䣬��
          ���õ�ǰ����Ǽ���������ٶȣ����ٶ�Ҳ�����ɣ����䣬��ʵ��
          �ϣ����ٶ�����һ�εĺ���Ǽ��㣬�϶���ͬ�������㷨������Ӱ�죩��
          ��󣬶����е��ٶȱ��ʽ���˻��֣���ɵõ����¹�ʽ��
***************************** ***************************************/
extern void DoubleTriangularOMNILocate(ST_DOUBLE_OMNI_LOCATION *pstDoubleOMNI , ST_OMNI_MOBILE_ROBOT *pstRobot);


extern void DoubleVerticalOMNILocateEx1(ST_DOUBLE_OMNI_LOCATION * pstDoubleOMNI, ST_OMNI_MOBILE_ROBOT * pstRobot);

extern void DoubleVerticalOMNILocatenew(ST_DOUBLE_OMNI_LOCATION *stDoubleOMNI , ST_OMNI_MOBILE_ROBOT *stRobot);




