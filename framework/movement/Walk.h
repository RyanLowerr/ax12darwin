#ifndef _WALK_H_
#define _WALK_H_

#include "Motion.h"

namespace Robot
{
	class Walk : public Motion
	{
		public:
		
			// Walking Controls
			enum
			{
				PERIOD_TIME        = 0,
				DOUBLE_LEG_RATIO   = 1,
				MOVE_GAIN          = 2,
				MOVE_AMPLITUDE_X   = 3,
				MOVE_AMPLITUDE_Y   = 4,
				MOVE_AMPLITUDE_Z   = 5,
				MOVE_AMPLITUDE_C   = 6,
				SHIFT_AMPLITUDE_X  = 7,
				SHIFT_AMPLITUDE_Y  = 8,
				SHIFT_AMPLITUDE_Z  = 9,
				SHIFT_AMPLITUDE_C  = 10,
				OFFSET_R_FOOT_X    = 11,
				OFFSET_R_FOOT_Y    = 12,
				OFFSET_R_FOOT_Z    = 13,
				OFFSET_R_FOOT_C    = 14,
				OFFSET_L_FOOT_X    = 15,
				OFFSET_L_FOOT_Y    = 16,
				OFFSET_L_FOOT_Z    = 17,
				OFFSET_L_FOOT_C    = 18,
				OFFSET_HIP_PITCH   = 19,
				OFFSET_HIP_ROLL    = 20,
				OFFSET_ANKLE_PITCH = 21,
				OFFSET_ANKLE_ROLL  = 22,
				NUMBER_WALK_CONTROLS
			};
			
			Walk();
			~Walk();
			
			static Walk* GetInstance();
			void Initialize();
			void Process();
			void SetBalanceEnable(bool enable);
			bool IsBalanceEnable();
			void Set_Control(int control, double value);
			double Get_Control(int control);
			
		private:
			
			static Walk* m_Instance;
			
			double m_Time;
			
			double m_Controls[NUMBER_WALK_CONTROLS];
			double m_QControls[NUMBER_WALK_CONTROLS];
			
			double m_Double_Leg_Ratio;
			double m_Single_Leg_Ratio;
			double m_Shift_Time_X;
			double m_Shift_Time_Y;
			double m_Shift_Time_Z;
			double m_Shift_Time_C;
			double m_Move_Time_X;
			double m_Move_Time_Y;
			double m_Move_Time_Z;
			double m_Move_Time_C;
			double m_Step_Time;
			double m_Step_Time_Start_R;
			double m_Step_Time_End_R;
			double m_Step_Time_Start_L;
			double m_Step_Time_End_L;
			
			bool m_Balance_Enabled;
			
			double SinWave(double time, double period, double period_shift, double magnitude, double magnitude_shift);
			void Update_Time_Parameters();
			void Update_Motion_Parameters();
	};
}

#endif
