#include "Quaternion.h"

#define _180_DIV_PI 57.295779515f // = 180 / PI

float BNO080_Roll;
float BNO080_Pitch;
float BNO080_Yaw;

void Quaternion_Update(float* q)
{
	float q1, q2, q3, q4;
	float norm;

	float length2=q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
    if(!isfinite(length2)||length2<0.25f||length2>2.25f){BNO080_Pitch=BNO080_Roll=BNO080_Yaw=NAN;return;}
    norm = 1.0f/sqrtf(length2);    // normalize quaternion
	
	q1 = q[0] * norm; //x
	q2 = q[1] * norm; //y
	q3 = q[2] * norm; //z
	q4 = q[3] * norm; //w

	BNO080_Pitch = atan2f(2.0f * (q2*q3 + q1*q4), q1*q1 + q2*q2 - q3*q3 - q4*q4);
	BNO080_Roll  = -asinf(fmaxf(-1.0f,fminf(1.0f,2.0f * (q2*q4 - q1*q3))));
	BNO080_Yaw   = atan2f(2.0f * (q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);

	BNO080_Pitch *= _180_DIV_PI;
	BNO080_Roll  *= _180_DIV_PI;
	BNO080_Yaw   *= _180_DIV_PI;
	
	if(BNO080_Yaw>=0)
		BNO080_Yaw = 360.f - BNO080_Yaw;
	else	
		BNO080_Yaw = -BNO080_Yaw;
	
	
	if(BNO080_Pitch>=0)
		BNO080_Pitch = 180.f - BNO080_Pitch;
	else
		BNO080_Pitch = -(BNO080_Pitch + 180.f);
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) { return x>0 ? 1.0f/sqrtf(x) : NAN; }
