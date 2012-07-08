
#include "AX12.h"
#include "Joint.h"

using namespace Robot;

Joint::Joint()
{
	for(int i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		m_Enable[i] = true;
		m_Value[i] = AX12::CENTER_VALUE;
		m_Angle[i] = 0.0;
	}
}

Joint::~Joint()
{
}

void Joint::SetEnable(int id, bool enable)
{
	m_Enable[id] = enable;
}

void Joint::SetEnableAll(bool enable)
{
	SetEnableLegs(enable);
	SetEnableHead(enable);
	SetEnableArms(enable);
}

void Joint::SetEnableLegs(bool enable)
{
	for(int id = ID_R_HIP_YAW; id <= ID_L_ANKLE_ROLL; id++)
		m_Enable[id] = enable;
}

void Joint::SetEnableHead(bool enable)
{
	m_Enable[ID_HEAD_PAN] = enable;
	m_Enable[ID_HEAD_TILT] = enable;
}

void Joint::SetEnableArms(bool enable)
{
	for(int id = ID_R_SHOULDER_PITCH; id <= ID_L_ELBOW; id++) 
		m_Enable[id] = enable;
}

bool Joint::GetEnable(int id)
{
	return m_Enable[id];
}

void Joint::SetValue(int id, int value)
{
	if(value < AX12::MIN_VALUE)
		value = AX12::MIN_VALUE;
	else if(value >= AX12::MAX_VALUE)
		value = AX12::MAX_VALUE;
	
	m_Value[id] = value;
	m_Angle[id] = AX12::Value2Angle(value);
}

int Joint::GetValue(int id)
{
	return m_Value[id];
}

void Joint::SetAngle(int id, double angle)
{
	if(angle < AX12::MIN_ANGLE)
		angle = AX12::MIN_ANGLE;
	else if(angle >= AX12::MAX_ANGLE)
		angle = AX12::MAX_ANGLE;
	
	m_Value[id] = AX12::Angle2Value(angle);
	m_Angle[id] = angle;
}

double Joint::GetAngle(int id)
{
	return m_Angle[id];
}

void Joint::SetRadian(int id, double radian)
{
	SetAngle(id, radian * (180.0 / 3.141592));
}

double Joint::GetRadian(int id)
{
	return GetAngle(id) * (180.0 / 3.141592);
}

