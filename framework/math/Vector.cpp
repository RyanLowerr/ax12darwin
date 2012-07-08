#include <math.h>
#include "Vector.h"

using namespace Robot;


Vector::Vector()
{
	X = 0;
	Y = 0;
	Z = 0;
}

Vector::Vector(double x, double y, double z)
{
	X = x;
	Y = y;
	Z = z;
}

Vector::Vector(const Point &pt1, const Point &pt2)
{
	X = pt2.X - pt1.X;
	Y = pt2.Y - pt1.Y;
	Z = pt2.Z - pt1.Z;
}

Vector::Vector(const Vector &vector)
{
	X = vector.X;
	Y = vector.Y;
	Z = vector.Z;
}

Vector::~Vector()
{
}

double Vector::Length()
{
	return sqrt(X * X + Y * Y + Z * Z);
}

void Vector::Normalize()
{
	double length = Length();
	
	X = X / length;
	Y = Y / length;
	Z = Z / length;
}

double Vector::DotProduct(const Vector &vector)
{
	return (X * vector.X + Y * vector.Y + Z * vector.Z);
}

Vector Vector::CrossProduct(const Vector &vector)
{
	Vector result;
	result.X = Y * vector.Z - Z * vector.Y;
	result.Y = Z * vector.X - X * vector.Z;
	result.Z = X * vector.Y - Y * vector.X;
	return result;
}

double Vector::AngleBetween(Vector &vector)
{
	return acos((X * vector.X + Y * vector.Y + Z * vector.Z) / (Length() * vector.Length())) * (180.0 / 3.141592);
}

double Vector::AngleBetween(Vector &vector, Vector &axis)
{
	double angle = AngleBetween(vector);
	Vector cross = CrossProduct(vector);
	if(cross.DotProduct(axis) < 0.0)
		angle *= -1.0;

	return angle;
}

Vector & Vector::operator = (const Vector &vector)
{
	X = vector.X;
	Y = vector.Y;
	Z = vector.Z;
	return *this;
}

Vector & Vector::operator += (const Vector &vector)
{
	X += vector.X;
	Y += vector.Y;
	Z += vector.Z;
	return *this;
}

Vector & Vector::operator -= (const Vector &vector)
{
	X -= vector.X;
	Y -= vector.Y;
	Z -= vector.Z;
	return *this;
}

Vector & Vector::operator += (const double value)
{
	X += value;
	Y += value;
	Z += value;
	return *this;
}

Vector & Vector::operator -= (const double value)
{
	X -= value;
	Y -= value;
	Z -= value;
	return *this;
}

Vector & Vector::operator *= (const double value)
{
	X *= value;
	Y *= value;
	Z *= value;
	return *this;
}

Vector & Vector::operator /= (const double value)
{
	X /= value;
	Y /= value;
	Z /= value;
	return *this;
}

Vector Vector::operator + (const Vector &vector)
{
	return Vector(X + vector.X, Y + vector.Y, Z + vector.Z);
}

Vector Vector::operator - (const Vector &vector)
{
	return Vector(X - vector.X, Y - vector.Y, Z - vector.Z);
}

Vector Vector::operator + (const double value)
{
	return Vector(X + value, Y + value, Z + value);
}

Vector Vector::operator - (const double value)
{
	return Vector(X - value, Y - value, Z - value);
}

Vector Vector::operator * (const double value)
{
	return Vector(X * value, Y * value, Z * value);
}

Vector Vector::operator / (const double value)
{
	return Vector(X / value, Y / value, Z / value);
}
