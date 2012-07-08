#include <stdio.h>
#include <math.h>

#include "Head.h"
#include "Joint.h"
#include "AX12.h"
#include "Motion.h"
#include "Manager.h"
#include "Kinematics.h"

using namespace Robot;

Head* Head::m_Instance = new Head();

Head::Head()
{
	m_Pan_Home  = 0.0;
	m_Pan_Max   = 45.0;
	m_Pan_Min   = -45.0;
	m_Pan_PGain = 0.1;
	m_Pan_IGain = 0.0;
	m_Pan_DGain = 0.1;
	
	m_Tilt_Home  = 0.0;
	m_Tilt_Max   = 45.0;
	m_Tilt_Min   = -45.0;
	m_Tilt_PGain = 0.1;
	m_Tilt_IGain = 0.0;
	m_Tilt_DGain = 0.1;
}

Head::~Head()
{
}

Head* Head::GetInstance() 
{ 
	return m_Instance; 
}

void Head::Initialize()
{
	Limits();
	InitTracking();
	MoveHome();
}

void Head::Process()
{
	m_Joint.SetAngle(Joint::ID_HEAD_PAN, m_Pan);
	m_Joint.SetAngle(Joint::ID_HEAD_TILT, m_Tilt);
}

void Head::Limits()
{
	if(m_Pan > m_Pan_Max)
		m_Pan = m_Pan_Max;
	else if(m_Pan < m_Pan_Min)
		m_Pan = m_Pan_Min;
	
	if(m_Tilt > m_Tilt_Max)
		m_Tilt = m_Tilt_Max;
	else if(m_Tilt < m_Tilt_Min)
		m_Tilt = m_Tilt_Min;
}

void Head::Move(double pan, double tilt)
{
	m_Pan = pan;
	m_Tilt = tilt;
	Limits();
}

void Head::MoveHome()
{
	Move(m_Pan_Home, m_Tilt_Home);
}

void Head::MoveOffset(double pan, double tilt)
{
	Move(m_Pan + pan, m_Tilt + tilt);
}

void Head::InitTracking()
{
	m_Pan_Error = 0;
	m_Pan_Diff = 0;
	m_Tilt_Error = 0;
	m_Tilt_Diff = 0;
}

void Head::Tracking(Point p)
{
	m_Pan_Diff = p.X - m_Pan_Error;
	m_Pan_Error = p.X;
	
	m_Tilt_Diff = p.Y - m_Tilt_Error;
	m_Tilt_Error = p.Y;
	
	Tracking();
}

void Head::Tracking()
{
	double p;
	double i;
	double d;
	
	p = m_Pan_Error * m_Pan_PGain;
	i = 0;
	d = m_Pan_Diff * m_Pan_DGain;
	m_Pan += (p + i + d);
	
	p = m_Tilt_Error * m_Tilt_PGain;
	i = 0;
	d = m_Tilt_Diff * m_Tilt_DGain;
	m_Tilt += (p + i + d);
	
	Limits();
}

double Head::PanMax()
{
	return m_Pan_Max;
}
 
double Head::PanMin()
{
	return m_Pan_Min;
}

double Head::Pan()
{
	return m_Pan;
}

double Head::TiltMax()
{
	return m_Tilt_Max;
}

double Head::TiltMin()
{
	return m_Tilt_Min;
}

double Head::Tilt()
{
	return m_Tilt;
}
