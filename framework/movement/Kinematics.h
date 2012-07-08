#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "Point.h"
#include "Vector.h"
#include "Matrix.h"

namespace Robot
{
	class Kinematics
	{
		public:	
			static const double Thigh_Length = 67.0;
			static const double Calf_Length = 67.0;
			static const double Ankle_Length = 25.0;
			static const double Leg_Length = 159.0;
	
			static bool LegIK(double *out, double x, double y, double z, double a, double b, double c);
			static bool ArmIK(double *out, double x, double y, double z, double a, double b, double c);	
			static bool LegFK();
			static bool ArmFK();
	};
}

#endif
