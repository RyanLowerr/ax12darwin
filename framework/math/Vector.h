#ifndef _VECTOR_H
#define _VECTOR_H

#include "Point.h"

namespace Robot
{
	class Vector
	{
		public:
		
			double X;
			double Y;
			double Z;
		
			Vector();
			Vector(double x, double y, double z);
			Vector(const Point &p1, const Point &p2);
			Vector(const Vector &vector);
			~Vector();
		
			double Length();
			void Normalize();
			double DotProduct(const Vector &vector);
			Vector CrossProduct(const Vector &vector);
			double AngleBetween(Vector &vector);
			double AngleBetween(Vector &vector, Vector &axis);

			Vector & operator = (const Vector &vector);
			Vector & operator += (const Vector &vector);
			Vector & operator -= (const Vector &vector);
			Vector & operator += (const double value);
			Vector & operator -= (const double value);
			Vector & operator *= (const double value);
			Vector & operator /= (const double value);
			Vector operator + (const Vector &vector);
			Vector operator - (const Vector &vector);
			Vector operator + (const double value);
			Vector operator - (const double value);
			Vector operator * (const double value);
			Vector operator / (const double value);
			
		private:
		
	};
}

#endif
