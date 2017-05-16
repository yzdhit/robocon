#ifndef MY_CORDIC_H_
#define MY_CORDIC_H_

#define DSP32_Q(x) ((int) ((x)*(((unsigned) (1 << (20)))))) 
#define P2_20  ((float)((unsigned) (1 << (20))))
#define PI 3.1415926536f


int MySin(int zi);
int MyCos(int zi);
int MyArctan2(int xi, int yi);
float Cordic_Sin(float angle) ;
float Cordic_Cos(float angle) ;
float Cordic_Tan(float dbAngle);
float Cordic_Ctan(float dbAngle);
float Cordic_Atan(float dbTanVal) ;
float Cordic_Actan(float dbCtanVal);
float Cordic_Asin(float dbSinVal);
float Cordic_Acos(float dbCosVal);
#endif
