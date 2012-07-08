#ifndef _MATRIX_H
#define _MATRIX_H

#include "Vector.h"
#include "Point.h"

namespace Robot
{
	class Matrix
	{
	public:
		enum
		{
			m00 = 0,
			m01,
			m02,
			m03,
			m10,
			m11,
			m12,
			m13,
			m20,
			m21,
			m22,
			m23,
			m30,
			m31,
			m32,
			m33,
			NUM_ELEMENTS
		};
		
		double m[NUM_ELEMENTS];
		
		Matrix();
		Matrix(const Matrix &matrix);
		~Matrix();
		
		void Identity();
		bool Inverse();
		Point Transform(Point p);
		Vector Transform(Vector v);
		void SetTransform(Point p, Vector v);
		void Multiply(Matrix *m1, Matrix *m2);
		
		Matrix & operator = (const Matrix &matrix);
	};
}

#endif
