#include <math.h>
#include "Kinematics.h"
#include "Point.h"
#include "Vector.h"
#include "Matrix.h"

#define PI 3.141592

using namespace Robot;

bool Kinematics::LegIK(double *out, double x, double y, double z, double a, double b, double c)
{
	Matrix Tad, Tda, Tcd, Tdc, Tac;
	Vector vec;
	double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;
	double hip_yaw, hip_pitch, hip_roll, knee, ankle_pitch, ankle_roll;
	
	double THIGH_LENGTH = Thigh_Length;
	double CALF_LENGTH = Calf_Length;
	double ANKLE_LENGTH = Ankle_Length;
	double LEG_LENGTH = Leg_Length;
	
	Tad.SetTransform(Point(x, y, z - LEG_LENGTH), Vector(a, b, c));
	
	vec.X = x + Tad.m[2] * ANKLE_LENGTH;
	vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
	vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;
	
	// Knee
	_Rac = vec.Length();
	_Acos = acos((_Rac*_Rac - THIGH_LENGTH*THIGH_LENGTH - CALF_LENGTH*CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
	
	if(isnan(_Acos) == 1)
		return false;
	
	knee = _Acos;
	
	// Ankle Roll
	Tda = Tad;
	if(Tda.Inverse() == false)
		return false;
	_k = sqrt(Tda.m[7]*Tda.m[7] + Tda.m[11]*Tda.m[11]);
	_l = sqrt(Tda.m[7]*Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH)*(Tda.m[11] - ANKLE_LENGTH));
	_m = (_k*_k - _l*_l - ANKLE_LENGTH*ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
	
	if(_m > 1.0)
		_m = 1.0;
	else if(_m < -1.0)
		_m = -1.0;
	
	_Acos = acos(_m);
	
	if(isnan(_Acos) == 1)
		return false;
		
	if(Tda.m[7] < 0.0)
		ankle_roll = -_Acos;
	else
		ankle_roll = _Acos;
	
	// Hip Yaw
	Tcd.SetTransform(Point(0.0, 0.0, -ANKLE_LENGTH), Vector(ankle_roll * 180 / PI, 0, 0));
	Tdc = Tcd;
	
	if(Tdc.Inverse() == false)
		return false;
	
	Tac.Multiply(&Tad, &Tdc);
	_Atan = atan2(-Tac.m[1], Tac.m[5]);
	
	if(isinf(_Atan) == 1)
		return false;
		
	hip_yaw = _Atan;
	
	// Hip Roll
	_Atan = atan2(Tac.m[9], -Tac.m[1] * sin(hip_yaw) + Tac.m[5] * cos(hip_yaw));
	
	if(isinf(_Atan) == 1)
		return false;
	
	hip_roll = _Atan;
	
	// Hip and Ankle Pitch
	_Atan = atan2(Tac.m[2] * cos(hip_yaw) + Tac.m[6] * sin(hip_yaw), Tac.m[0] * cos(hip_yaw) + Tac.m[4] * sin(hip_yaw));
	
	if(isinf(_Atan) == 1)
		return false;
		
	_theta = _Atan;
	_k = sin(knee) * CALF_LENGTH;
	_l = -THIGH_LENGTH - cos(knee) * CALF_LENGTH;
	_m = cos(hip_yaw) * vec.X + sin(hip_yaw) * vec.Y;
	_n = cos(hip_roll) * vec.Z + sin(hip_yaw) * sin(hip_roll) * vec.X - cos(hip_yaw) * sin(hip_roll) * vec.Y;
	_s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
	_c = (_n - _k * _s) / _l;
	_Atan = atan2(_s, _c);
	
	if(isinf(_Atan) == 1)
		return false;
		
	hip_pitch = _Atan;
	ankle_pitch = _theta - knee - hip_pitch;
	
	// Set out values
	*(out)     = hip_yaw;
	*(out + 1) = hip_roll;
	*(out + 2) = hip_pitch;
	*(out + 3) = knee;
	*(out + 4) = ankle_pitch;
	*(out + 5) = ankle_roll; 

	return true;
}


bool Kinematics::ArmIK(double *out, double x, double y, double z, double a, double b, double c)
{
}
		
bool Kinematics::LegFK()
{
}

bool Kinematics::ArmFK()
{
}
