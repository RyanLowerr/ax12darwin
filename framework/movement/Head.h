#ifndef _HEAD_H_
#define _HEAD_H_

#include "Motion.h"
#include "Point.h"

namespace Robot
{
	class Head : public Motion
	{
	
		public:
	
			Head();
			~Head();
		
			static Head* GetInstance();
			void Initialize();
			void Process();
		
			void MoveHome();
			void Move(double pan, double tilt);
			void MoveOffset(double pan, double tilt);
			void InitTracking();
			void Tracking(Point p);
			void Tracking();
		
			double PanMax();  
			double PanMin();
			double Pan();
		
			double TiltMax();
			double TiltMin();
			double Tilt();
	
		private:
	
			static Head* m_Instance;
		
			double m_Pan_Home;
			double m_Pan_Max;
			double m_Pan_Min;
			double m_Pan_Error;
			double m_Pan_Diff;
			double m_Pan_PGain;
			double m_Pan_IGain;
			double m_Pan_DGain;
			double m_Pan;
		
			double m_Tilt_Home;
			double m_Tilt_Max;
			double m_Tilt_Min;
			double m_Tilt_Error;
			double m_Tilt_Diff;
			double m_Tilt_PGain;
			double m_Tilt_IGain;
			double m_Tilt_DGain;
			double m_Tilt;
		
			void Limits();
	};
}

#endif
