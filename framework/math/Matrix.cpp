
#include <math.h>
#include "Matrix.h"

using namespace Robot;

Matrix::Matrix()
{
	Identity();
}

Matrix::Matrix(const Matrix &matrix)
{
	*this = matrix;
}

Matrix::~Matrix()
{
}

void Matrix::Identity()
{
	m[m00] = 1; m[m01] = 0; m[m02] = 0; m[m03] = 0;
	m[m10] = 0; m[m11] = 1; m[m12] = 0; m[m13] = 0;
	m[m20] = 0; m[m21] = 0; m[m22] = 1; m[m23] = 0;
	m[m30] = 0; m[m31] = 0; m[m32] = 0; m[m33] = 1;
}

bool Matrix::Inverse()
{
	Matrix src, dst, tmp;
	double det;

    /* transpose matrix */
    for (int i = 0; i < 4; i++)
    {
    	src.m[i]      = m[i*4];
    	src.m[i + 4]  = m[i*4 + 1];
    	src.m[i + 8]  = m[i*4 + 2];
    	src.m[i + 12] = m[i*4 + 3];
    }

    // calculate pairs for first 8 elements (cofactors)
    tmp.m[0] = src.m[10] * src.m[15];
    tmp.m[1] = src.m[11] * src.m[14];
    tmp.m[2] = src.m[9]  * src.m[15];
    tmp.m[3] = src.m[11] * src.m[13];
    tmp.m[4] = src.m[9]  * src.m[14];
    tmp.m[5] = src.m[10] * src.m[13];
    tmp.m[6] = src.m[8]  * src.m[15];
    tmp.m[7] = src.m[11] * src.m[12];
    tmp.m[8] = src.m[8]  * src.m[14];
    tmp.m[9] = src.m[10] * src.m[12];
    tmp.m[10] = src.m[8] * src.m[13];
    tmp.m[11] = src.m[9] * src.m[12];
    
    // calculate first 8 elements (cofactors)
    dst.m[0] = (tmp.m[0]*src.m[5] + tmp.m[3]*src.m[6] + tmp.m[4]*src.m[7])  - (tmp.m[1]*src.m[5] + tmp.m[2]*src.m[6] + tmp.m[5]*src.m[7]);
    dst.m[1] = (tmp.m[1]*src.m[4] + tmp.m[6]*src.m[6] + tmp.m[9]*src.m[7])  - (tmp.m[0]*src.m[4] + tmp.m[7]*src.m[6] + tmp.m[8]*src.m[7]);
    dst.m[2] = (tmp.m[2]*src.m[4] + tmp.m[7]*src.m[5] + tmp.m[10]*src.m[7]) - (tmp.m[3]*src.m[4] + tmp.m[6]*src.m[5] + tmp.m[11]*src.m[7]);
    dst.m[3] = (tmp.m[5]*src.m[4] + tmp.m[8]*src.m[5] + tmp.m[11]*src.m[6]) - (tmp.m[4]*src.m[4] + tmp.m[9]*src.m[5] + tmp.m[10]*src.m[6]);
    dst.m[4] = (tmp.m[1]*src.m[1] + tmp.m[2]*src.m[2] + tmp.m[5]*src.m[3])  - (tmp.m[0]*src.m[1] + tmp.m[3]*src.m[2] + tmp.m[4]*src.m[3]);
    dst.m[5] = (tmp.m[0]*src.m[0] + tmp.m[7]*src.m[2] + tmp.m[8]*src.m[3])  - (tmp.m[1]*src.m[0] + tmp.m[6]*src.m[2] + tmp.m[9]*src.m[3]);
    dst.m[6] = (tmp.m[3]*src.m[0] + tmp.m[6]*src.m[1] + tmp.m[11]*src.m[3]) - (tmp.m[2]*src.m[0] + tmp.m[7]*src.m[1] + tmp.m[10]*src.m[3]);
    dst.m[7] = (tmp.m[4]*src.m[0] + tmp.m[9]*src.m[1] + tmp.m[10]*src.m[2]) - (tmp.m[5]*src.m[0] + tmp.m[8]*src.m[1] + tmp.m[11]*src.m[2]);
    
    // calculate pairs for second 8 elements (cofactors)
    tmp.m[0] = src.m[2]*src.m[7];
    tmp.m[1] = src.m[3]*src.m[6];
    tmp.m[2] = src.m[1]*src.m[7];
    tmp.m[3] = src.m[3]*src.m[5];
    tmp.m[4] = src.m[1]*src.m[6];
    tmp.m[5] = src.m[2]*src.m[5];
    
    // Streaming SIMD Extensions - Inverse of 4x4 Matrix 8
    tmp.m[6]  = src.m[0]*src.m[7];
    tmp.m[7]  = src.m[3]*src.m[4];
    tmp.m[8]  = src.m[0]*src.m[6];
    tmp.m[9]  = src.m[2]*src.m[4];
    tmp.m[10] = src.m[0]*src.m[5];
    tmp.m[11] = src.m[1]*src.m[4];
    
    // calculate second 8 elements (cofactors)
    dst.m[8]  = (tmp.m[0]*src.m[13]  + tmp.m[3]*src.m[14]  + tmp.m[4]*src.m[15])  - (tmp.m[1]*src.m[13]  + tmp.m[2]*src.m[14]  + tmp.m[5]*src.m[15]);
    dst.m[9]  = (tmp.m[1]*src.m[12]  + tmp.m[6]*src.m[14]  + tmp.m[9]*src.m[15])  - (tmp.m[0]*src.m[12]  + tmp.m[7]*src.m[14]  + tmp.m[8]*src.m[15]);
    dst.m[10] = (tmp.m[2]*src.m[12]  + tmp.m[7]*src.m[13]  + tmp.m[10]*src.m[15]) - (tmp.m[3]*src.m[12]  + tmp.m[6]*src.m[13]  + tmp.m[11]*src.m[15]);
    dst.m[11] = (tmp.m[5]*src.m[12]  + tmp.m[8]*src.m[13]  + tmp.m[11]*src.m[14]) - (tmp.m[4]*src.m[12]  + tmp.m[9]*src.m[13]  + tmp.m[10]*src.m[14]);
    dst.m[12] = (tmp.m[2]*src.m[10]  + tmp.m[5]*src.m[11]  + tmp.m[1]*src.m[9])   - (tmp.m[4]*src.m[11]  + tmp.m[0]*src.m[9]   + tmp.m[3]*src.m[10]);
    dst.m[13] = (tmp.m[8]*src.m[11]  + tmp.m[0]*src.m[8]   + tmp.m[7]*src.m[10])  - (tmp.m[6]*src.m[10]  + tmp.m[9]*src.m[11]  + tmp.m[1]*src.m[8]);
    dst.m[14] = (tmp.m[6]*src.m[9]   + tmp.m[11]*src.m[11] + tmp.m[3]*src.m[8])   - (tmp.m[10]*src.m[11] + tmp.m[2]*src.m[8]   + tmp.m[7]*src.m[9]);
    dst.m[15] = (tmp.m[10]*src.m[10] + tmp.m[4]*src.m[8]   + tmp.m[9]*src.m[9])   - (tmp.m[8]*src.m[9]   + tmp.m[11]*src.m[10] + tmp.m[5]*src.m[8]);
    
    // calculate determinant
    det = src.m[0]*dst.m[0] + src.m[1]*dst.m[1] + src.m[2]*dst.m[2] + src.m[3]*dst.m[3];
    
    // calculate matrix inverse
    if (det == 0)
    {
    	det = 0;
    	return false;
    }
    else
    {
    	det = 1 / det;
    }

    for (int i = 0; i < NUM_ELEMENTS; i++)
    	m[i] = dst.m[i] * det;

    return true;
}

Point Matrix::Transform(Point point)
{
	Point result;
	result.X = m[m00] * point.X + m[m01] * point.Y + m[m02] * point.Z + m[m03];
	result.Y = m[m10] * point.X + m[m11] * point.Y + m[m12] * point.Z + m[m13];
	result.Z = m[m20] * point.X + m[m21] * point.Y + m[m22] * point.Z + m[m23];
    return result;
}

Vector Matrix::Transform(Vector vector)
{
	Vector result;
	result.X = m[m00] * vector.X + m[m01] * vector.Y + m[m02] * vector.Z + m[m03];
	result.Y = m[m10] * vector.X + m[m11] * vector.Y + m[m12] * vector.Z + m[m13];
	result.Z = m[m20] * vector.X + m[m21] * vector.Y + m[m22] * vector.Z + m[m23];
    return result;
}

void Matrix::SetTransform(Point point, Vector angle)
{
	double cx = cos(angle.X * 3.141592 / 180.0);
	double cy = cos(angle.Y * 3.141592 / 180.0);
	double cz = cos(angle.Z * 3.141592 / 180.0);
	double sx = sin(angle.X * 3.141592 / 180.0);
	double sy = sin(angle.Y * 3.141592 / 180.0);
	double sz = sin(angle.Z * 3.141592 / 180.0);
	
	Identity();
	m[0] = cz * cy;
  m[1] = cz * sy * sx - sz * cx;
  m[2] = cz * sy * cx + sz * sx;
	m[3] = point.X;
  m[4] = sz * cy;
  m[5] = sz * sy * sx + cz * cx;
  m[6] = sz * sy * cx - cz * sx;
  m[7] = point.Y;
  m[8] = -sy;
  m[9] = cy * sx;
  m[10] = cy * cx;
  m[11] = point.Z;
}

void Matrix::Multiply(Matrix *m1, Matrix *m2)
{
	// Turn this into some kind of loop! GOSH!
	m[0]  = m1->m[0]*m2->m[0] + m1->m[1]*m2->m[4] + m1->m[2]*m2->m[8] + m1->m[3]*m2->m[12];
	m[1]  = m1->m[0]*m2->m[1] + m1->m[1]*m2->m[5] + m1->m[2]*m2->m[9] + m1->m[3]*m2->m[13];
	m[2]  = m1->m[0]*m2->m[2] + m1->m[1]*m2->m[6] + m1->m[2]*m2->m[10] + m1->m[3]*m2->m[14];
	m[3]  = m1->m[0]*m2->m[3] + m1->m[1]*m2->m[7] + m1->m[2]*m2->m[11] + m1->m[3]*m2->m[15];
	
	m[4]  = m1->m[4]*m2->m[0] + m1->m[5]*m2->m[4] + m1->m[6]*m2->m[8] + m1->m[7]*m2->m[12];
	m[5]  = m1->m[4]*m2->m[1] + m1->m[5]*m2->m[5] + m1->m[6]*m2->m[9] + m1->m[7]*m2->m[13];
	m[6]  = m1->m[4]*m2->m[2] + m1->m[5]*m2->m[6] + m1->m[6]*m2->m[10] + m1->m[7]*m2->m[14];
	m[7]  = m1->m[4]*m2->m[3] + m1->m[5]*m2->m[7] + m1->m[6]*m2->m[11] + m1->m[7]*m2->m[15];
	
	m[8]  = m1->m[8]*m2->m[0] + m1->m[9]*m2->m[4] + m1->m[10]*m2->m[8] + m1->m[11]*m2->m[12];
	m[9]  = m1->m[8]*m2->m[1] + m1->m[9]*m2->m[5] + m1->m[10]*m2->m[9] + m1->m[11]*m2->m[13];
	m[10] = m1->m[8]*m2->m[2] + m1->m[9]*m2->m[6] + m1->m[10]*m2->m[10] + m1->m[11]*m2->m[14];
	m[11] = m1->m[8]*m2->m[3] + m1->m[9]*m2->m[7] + m1->m[10]*m2->m[11] + m1->m[11]*m2->m[15];
	
	m[12] = m1->m[12]*m2->m[0] + m1->m[13]*m2->m[4] + m1->m[14]*m2->m[8] + m1->m[15]*m2->m[12];
	m[13] = m1->m[12]*m2->m[1] + m1->m[13]*m2->m[5] + m1->m[14]*m2->m[9] + m1->m[15]*m2->m[13];
	m[14] = m1->m[12]*m2->m[2] + m1->m[13]*m2->m[6] + m1->m[14]*m2->m[10] + m1->m[15]*m2->m[14];
	m[15] = m1->m[12]*m2->m[3] + m1->m[13]*m2->m[7] + m1->m[14]*m2->m[11] + m1->m[15]*m2->m[15];
}

Matrix & Matrix::operator = (const Matrix &matrix)
{
	for(int i = 0; i < NUM_ELEMENTS; i++)
		m[i] = matrix.m[i];
	return *this;
}
