
#ifndef _POINT_H
#define _POINT_H

namespace Robot
{
	class Point
	{
		public:
		
			double X;
			double Y;
			double Z;
		
			Point();
			Point(double x, double y, double z);
			Point(const Point &P);
			~Point();
		
			Point & operator = (const Point &P);
			Point & operator += (const Point &P);
			Point & operator -= (const Point &P);
			Point & operator += (const double v);
			Point & operator -= (const double v);
			Point & operator *= (const double v);
			Point & operator /= (const double v);
			Point operator + (const Point &P);
			Point operator - (const Point &P);
			Point operator + (const double v);
			Point operator - (const double v);
			Point operator * (const double v);
			Point operator / (const double v);
			
		private:
	
	};
}

#endif
