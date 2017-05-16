#include "cordic.h"
#include "math.h"
#include "HITCRT_TYPES.h"
#define  NUMOFDET 12  //不能超过25
#define ABS(x) ( (x) >= 0 ? (x) : ( - (x) ) )
 
const int atanLUT[] = {
	DSP32_Q(0.785398163397448),
	DSP32_Q(0.463647609000806),
	DSP32_Q(0.244978663126864),
	DSP32_Q(0.124354994546761),
	DSP32_Q(0.062418809995957),
	DSP32_Q(0.031239833430268),
	DSP32_Q(0.015623728620477),
	DSP32_Q(0.007812341060101),
	DSP32_Q(0.003906230131967),
	DSP32_Q(0.001953122516479),
	DSP32_Q(0.000976562189559),
	DSP32_Q(0.000488281211195),
	DSP32_Q(0.000244140620149),
	DSP32_Q(0.000122070311894),
	DSP32_Q(0.000061035156174),
	DSP32_Q(0.000030517578116),
	DSP32_Q(0.000015258789061),
	DSP32_Q(0.000007629394531),
	DSP32_Q(0.000003814697266),
	DSP32_Q(0.000001907348633),
	DSP32_Q(0.000000953674316),
	DSP32_Q(0.000000476837158),
	DSP32_Q(0.000000238418579),
	DSP32_Q(0.000000119209290),
	DSP32_Q(0.000000059604645),
};

int MySin(int zi) 
{ 
	int x, y, z, x_new , y_new; 
	int i, sign; 
	
	x = DSP32_Q(0.607252935008882); 
	y = 0; 
	z = zi; 

	for(i=0; i<NUMOFDET; i++) 
	{ 
		if(z >= 0) 
		{ 
			sign = 1; 
		} 
		else 
		{ 
			sign = -1; 
		} 

		x_new = x - ((sign*y)>>i); 
		y_new = y + ((sign*x)>>i); 
		x = x_new;
		y = y_new;
		z = z - sign*atanLUT[i]; 
	} 

	return (y); 
} 

int MyCos(int zi) 
{ 
	int x, y, z, x_new , y_new; 
	int i, sign; 

	x = DSP32_Q(0.607252935008882); 
	y = 0; 
	z = zi; 

	for(i=0; i<NUMOFDET; i++) 
	{ 
		if(z >= 0) 
		{ 
			sign = 1; 
		} 
		else 
		{ 
			sign = -1; 
		} 

		x_new = x - ((sign*y)>>i); 
		y_new = y + ((sign*x)>>i); 
		x = x_new;
		y = y_new;
		z = z - sign*atanLUT[i]; 
	} 

	return (x); 
} 

int MyArctan2(int xi, int yi) 
{ 
  int x, y, z, x_new; 
  int i, sign; 
   
  x = xi; 
  y = yi; 
  z = 0; 
     
  for(i=0; i<NUMOFDET; i++) 
  { 
    if(y >= 0) 
    { 
 //     sign = -1; 
   x_new +=  ((y)>>i); 
   y -=  ((x)>>i); 
    x = x_new; 
    z += atanLUT[i];  
    } 
    else 
    { 
 //     sign = 1; 
	  x_new -=  ((y)>>i); 
	  y += ((x)>>i); 
	  x = x_new; 
	  z -=  atanLUT[i]; 
    } 
 /*    
    x_new = x - ((sign*y)>>i); 
    y = y + ((sign*x)>>i); 
    x = x_new; 
    z = z - sign*atanLUT[i]; */
  } 
   
  return (z); 
}

float Cordic_Sin(float angle)  //angle ： 角度制
{	
	// 精度能够达到e-5度以上
	int pp =  ( (( (ABS( (int)angle ) ) /90)) );
	int ret = 0;

	float ang = ( ABS(angle) - pp * 90 );
	switch (pp%4)
	{
	case 0:
		ret = MySin(DSP32_Q(ang*( PI/180.0 )));
		break;
	case 1:
		ret = MyCos(DSP32_Q(ang*( PI/180.0 )));
		break;
	case 2:
		ret = -MySin(DSP32_Q(ang*( PI/180.0 )));
		break;
	case 3:
		ret = - MyCos(DSP32_Q(ang*( PI/180.0 )));
		break;
	}

	ret = ret>>20;
	if (angle < 0)
	{
		ret = -ret;
	}
	return (FP32)ret;
}

float Cordic_Cos(float angle)
{
	// 精度能够达到 e-5度以上
	int pp =  ((int)(floor(ABS(angle)/90.0))) ;
	float ret = 0;

	float ang = ( ABS(angle) - pp * 90 );
	switch (pp%4)
	{
	case 0:
		ret = (float)MyCos(DSP32_Q(ang/180.0 * PI));
		break;
	case 1:
		ret = -(float)MySin(DSP32_Q(ang/180.0 * PI));
		break;
	case 2:
		ret = -(float)MyCos(DSP32_Q(ang/180.0 * PI));
		break;
	case 3:
		ret = (float)MySin(DSP32_Q(ang/180.0 * PI));
		break;
	}

	ret = ret/ P2_20;
	return ret;
}

float Cordic_Atan(float tanVal)
{
	//精度能够达到e-5以上
	float ret = 0;
	if (ABS(tanVal) < 1)
	{
		ret =(FP32) ( ( MyArctan2( DSP32_Q(1),DSP32_Q( ABS(tanVal) ) ) ) >> 20) ;		
	}
	else
	{
		ret = (float) ( (MyArctan2(DSP32_Q(1/ABS(tanVal)) , DSP32_Q(1)) ) >> (20) ) ;
	}
	if (tanVal < 0)
	{
		return -ret;
	}
	return ret;
}

float Cordic_Tan(float dbAngle)
{
	
	return Cordic_Sin(dbAngle)/Cordic_Cos(dbAngle);
}

float Cordic_Ctan(float dbAngle)
{
	return Cordic_Cos(dbAngle)/Cordic_Sin(dbAngle);
}

float Cordic_Actan(float dbCtanVal)
{
	if (dbCtanVal >= 0)
	{
		return PI/2 - Cordic_Atan(dbCtanVal);
	}
	else
	{
		return  -PI/2 + Cordic_Atan(-dbCtanVal);
	}
	return 0;
}

float Cordic_Asin(float dbSinVal)
{
	//算法待定
	return asin(dbSinVal);
}
float Cordic_Acos(float dbCosVal)
{
	//算法待定
	return acos(dbCosVal);
}