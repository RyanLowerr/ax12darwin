
#include <math.h>
#include "Point.h"

using namespace Robot;

Point::Point()
{
	X = 0;
	Y = 0;
	Z = 0;
}

Point::Point(double x, double y, double z)
{
	X = x;
	Y = y;
	Z = z;
}

Point::Point(const Point &P)
{
	X = P.X;
	Y = P.Y;
	Z = P.Z;
}

Point::~Point()
{
}

Point & Point::operator = (const Point &P)
{
	X = P.X;
	Y = P.Y;
	Z = P.Z;
	return *this;
}

Point & Point::operator += (const Point &P)
{
	X += P.X;
	Y += P.Y;
	Z += P.Z;
	return *this;
}

Point & Point::operator -= (const Point &P)
{
	X -= P.X;
	Y -= P.Y;
	Z -= P.Z;
	return *this;
}

Point & Point::operator += (const double v)
{
	X += v;
	Y += v;
	Z += v;
	return *this;
}

Point & Point::operator -= (const double v)
{
	X -= v;
	Y -= v;
	Z -= v;
	return *this;
}

Point & Point::operator *= (const double v)
{
	X *= v;
	Y *= v;
	Z *= v;
	return *this;
}

Point & Point::operator /= (const double v)
{
	X /= v;
	Y /= v;
	Z /= v;
	return *this;
}

Point Point::operator + (const Point &P)
{
	return Point(X + P.X, Y + P.Y, Z + P.Z);
}

Point Point::operator - (const Point &P)
{
	return Point(X - P.X, Y - P.Y, Z - P.Z);
}

Point Point::operator + (const double v)
{
	return Point(X + v, Y + v, Z + v);
}

Point Point::operator - (const double v)
{
	return Point(X - v, Y - v, Z - v);
}

Point Point::operator * (const double v)
{
	return Point(X * v, Y * v, Z * v);
}

Point Point::operator / (const double v)
{
	return Point(X / v, Y / v, Z / v);
}
