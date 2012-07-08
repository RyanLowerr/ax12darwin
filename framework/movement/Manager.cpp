#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>

#include "Manager.h"
#include "Motion.h"
#include "Joint.h"
#include "AX12.h"

using namespace Robot;

MotionManager* MotionManager::m_Instance = new MotionManager();

MotionManager::MotionManager()
{
	m_Debug = true;
	m_Running = false;
}

MotionManager::~MotionManager()
{
}

void MotionManager::SignalProc(int arg)
{
	if(m_Instance->Running())
		m_Instance->Process();
}

bool MotionManager::Initialize(AX12* pax12)
{
	int value;
	mp_AX12 = pax12;
	
	if(mp_AX12->Connect() == false)
	{
		return false;
	}
	
	for(int id = 1; id < Joint::NUMBER_OF_JOINTS; id++)
	{
		if(m_Debug == true)
			printf("\nJoint ID: %d ...", id);
		
		if(mp_AX12->ReadWord(id, AX12::P_GOAL_POSITION_L, &value) == AX12::SUCCESS)
		{
			m_Joints.SetValue(id, value);
			m_Joints.SetEnable(id, true);
			
			if(m_Debug == true)
				printf(" Success! [Position: %d]", value);
		}
		else
		{
			m_Joints.SetEnable(id, false);
			
			if(m_Debug == true)
				printf(" Fail!");
		}
	}
	
	struct itimerspec time_spec;
	struct sigevent sig_spec;
	m_TimerID = 0;
	
	sig_spec.sigev_notify = SIGEV_SIGNAL;
	sig_spec.sigev_signo = SIGRTMIN;
	
	time_spec.it_value.tv_sec = 0;
	time_spec.it_value.tv_nsec = 1000000;
	time_spec.it_interval.tv_sec = 0;
	time_spec.it_interval.tv_nsec = 1000000;
	
	timer_create(CLOCK_REALTIME, &sig_spec, &m_TimerID);
	timer_settime(m_TimerID, 0, &time_spec, NULL);
	signal(SIGRTMIN, SignalProc);
	
	return true;
}

void MotionManager::Process()
{
	// IMU Processing
	
	// Motion Processing
	if(m_Motions.size() != 0)
	{
		for(std::list<Motion*>::iterator it = m_Motions.begin(); it != m_Motions.end(); it++)
		{
			(*it)->Process();
			
			for(int id = 0; id < 21; id++)
			{
				if((*it)->m_Joint.GetEnable(id) == true)
					m_Joints.SetValue(id, (*it)->m_Joint.GetValue(id));
			}
		}
	}
	
	// Sync write processed joint positions
	int param[100];
	int n = 0;
	int joint_count = 0;
	
	for(int id = 1; id < Joint::NUMBER_OF_JOINTS; id++)
	{
		if(m_Joints.GetEnable(id) == true)
		{
			param[n++] = id;
			param[n++] = AX12::GetLowByte(m_Joints.GetValue(id));
			param[n++] = AX12::GetHighByte(m_Joints.GetValue(id));
			joint_count++;
		}
	}
	
	if(joint_count > 0)
		mp_AX12->SyncWrite(AX12::P_GOAL_POSITION_L, 2, joint_count, param);
}

void MotionManager::AddMotion(Motion* pmotion)
{
	pmotion->Initialize();
	m_Motions.push_back(pmotion);
}

void MotionManager::RemoveMotion(Motion* pmotion)
{
	m_Motions.remove(pmotion);
}

MotionManager* MotionManager::GetInstance()
{
	return m_Instance;
}

bool MotionManager::Running()
{
	return m_Running; 
}

void MotionManager::Running(bool value)
{ 
	m_Running = value;
}

bool MotionManager::Debug()
{
	return m_Debug;
}

void MotionManager::Debug(bool value)
{
	m_Debug = value;
}

int MotionManager::GyroPitch()
{
	return m_Gyro_Pitch;
}

int MotionManager::GyroRoll()
{
	return m_Gyro_Roll;
}

int MotionManager::AccelPitch()
{
	return m_Accel_Pitch;
}

int MotionManager::AccelRoll()
{
	return m_Accel_Roll;
}

int MotionManager::Button()
{
	return m_Button;
}

int MotionManager::FallCount()
{
	return m_Fall_Count;
}
