#ifndef _MOTION_MANAGER_H_
#define _MOTION_MANAGER_H_

#include <pthread.h>
#include <signal.h>
#include <list>
#include "Motion.h"
#include "Joint.h"
#include "AX12.h"

namespace Robot
{
	class MotionManager
	{
		public:
			
			MotionManager();
			~MotionManager();
			
			bool Initialize(AX12* pax12);
			void Process();
			void AddMotion(Motion* pmotion);
			void RemoveMotion(Motion* pmotion);
			
			static MotionManager* GetInstance();
			
			bool Running();
			void Running(bool value);
			bool Debug();
			void Debug(bool value);
			int GyroPitch();
			int GyroRoll();
			int AccelPitch();
			int AccelRoll();
			int Button();
			int FallCount();
		
		private:
			
			bool m_Running;
			bool m_Debug;
			Joint m_Joints;
			int m_Gyro_Pitch;
			int m_Gyro_Roll;
			int m_Accel_Pitch;
			int m_Accel_Roll;
			int m_Button;
			int m_Fall;
			int m_Fall_Count;
			
			static MotionManager* m_Instance;
			
			std::list<Motion*> m_Motions;
			AX12* mp_AX12;
			timer_t m_TimerID;
			
			static void SignalProc(int arg);
		};
}

#endif
