#include <math.h>
#include <stdio.h>

#include "Walk.h"
#include "Joint.h"
#include "AX12.h"
#include "Motion.h"
#include "Kinematics.h"

using namespace Robot;

Walk* Walk::m_Instance = new Walk();

Walk::Walk()
{
}

Walk::~Walk()
{
}

void Walk::SetBalanceEnable(bool enable)
{
	m_Balance_Enabled = enable;
}

bool Walk::IsBalanceEnable()
{
	return m_Balance_Enabled;
}

Walk* Walk::GetInstance() 
{ 
	return m_Instance; 
}

void Walk::Initialize()
{
	Set_Control(PERIOD_TIME, 600.0);
	Set_Control(DOUBLE_LEG_RATIO, 1);
	Set_Control(MOVE_GAIN, 3.0);
	Set_Control(MOVE_AMPLITUDE_X, 0.0);
	Set_Control(MOVE_AMPLITUDE_Y, 0.0);
	Set_Control(MOVE_AMPLITUDE_Z, 0.0);
	Set_Control(MOVE_AMPLITUDE_C, 0.0);
	Set_Control(SHIFT_AMPLITUDE_X, 0.0);
	Set_Control(SHIFT_AMPLITUDE_Y, 0.0);
	Set_Control(SHIFT_AMPLITUDE_Z, 0.0);
	Set_Control(SHIFT_AMPLITUDE_C, 0.0);
	Set_Control(OFFSET_R_FOOT_X, 0.0);
	Set_Control(OFFSET_R_FOOT_Y, 0.0);
	Set_Control(OFFSET_R_FOOT_Z, 0.0);
	Set_Control(OFFSET_R_FOOT_C, 0.0);
	Set_Control(OFFSET_L_FOOT_X, 0.0);
	Set_Control(OFFSET_L_FOOT_Y, 0.0);
	Set_Control(OFFSET_L_FOOT_Z, 0.0);
	Set_Control(OFFSET_L_FOOT_C, 0.0);
	Set_Control(OFFSET_HIP_PITCH, 0.0);
	Set_Control(OFFSET_HIP_ROLL, 0.0);
	Set_Control(OFFSET_ANKLE_PITCH, 0.0);
	Set_Control(OFFSET_ANKLE_ROLL, 0.0);

	Update_Time_Parameters();
	Update_Motion_Parameters();
}

double Walk::SinWave(double time, double period, double period_shift, double magnitude, double magnitude_shift)
{
	return magnitude * sin(2 * 3.141592 / period * time - period_shift) + magnitude_shift;
}

void Walk::Set_Control(int control, double value)
{
	m_QControls[control] = value;
}

double Walk::Get_Control(int control)
{
	return m_Controls[control];	
}

void Walk::Update_Time_Parameters()
{
	m_Controls[PERIOD_TIME] = m_QControls[PERIOD_TIME];
	m_Controls[DOUBLE_LEG_RATIO] = m_QControls[DOUBLE_LEG_RATIO];
	m_Single_Leg_Ratio = 1 - m_Controls[DOUBLE_LEG_RATIO];
	m_Double_Leg_Ratio = m_Controls[DOUBLE_LEG_RATIO];
	
	//m_Shift_Time_X = m_Period_Time / 2;
	m_Shift_Time_Y = m_Controls[PERIOD_TIME];
	m_Shift_Time_Z = m_Controls[PERIOD_TIME] / 2;
	//m_Shift_Time_C = m_Period_Time / 2;
	
	m_Move_Time_X = m_Controls[PERIOD_TIME] * m_Single_Leg_Ratio * 2;
	m_Move_Time_Y = m_Controls[PERIOD_TIME] * m_Single_Leg_Ratio * 2;
	m_Move_Time_Z = m_Controls[PERIOD_TIME] * m_Single_Leg_Ratio;
	m_Move_Time_C = m_Controls[PERIOD_TIME] * m_Single_Leg_Ratio * 2;
	
	m_Step_Time  = m_Controls[PERIOD_TIME] * m_Single_Leg_Ratio;
	m_Step_Time_Start_R = (1 - m_Single_Leg_Ratio) * m_Controls[PERIOD_TIME] / 4;
	m_Step_Time_End_R   = (1 + m_Single_Leg_Ratio) * m_Controls[PERIOD_TIME] / 4; 
	m_Step_Time_Start_L = (3 - m_Single_Leg_Ratio) * m_Controls[PERIOD_TIME] / 4; 
	m_Step_Time_End_L   = (3 + m_Single_Leg_Ratio) * m_Controls[PERIOD_TIME] / 4; 
}

void Walk::Update_Motion_Parameters()
{
	m_Controls[MOVE_GAIN] = m_QControls[MOVE_GAIN];

	m_Controls[MOVE_AMPLITUDE_Z] = m_QControls[MOVE_AMPLITUDE_Z];

	if(m_Controls[MOVE_GAIN] < 1.0)
	{
		m_Controls[MOVE_AMPLITUDE_X] = m_QControls[MOVE_AMPLITUDE_X];
		m_Controls[MOVE_AMPLITUDE_Y] = m_QControls[MOVE_AMPLITUDE_Y];
		m_Controls[MOVE_AMPLITUDE_C] = m_QControls[MOVE_AMPLITUDE_C];
	}
	else
	{
		// Move X
		if(m_Controls[MOVE_AMPLITUDE_X] < m_QControls[MOVE_AMPLITUDE_X])
			m_Controls[MOVE_AMPLITUDE_X] += m_Controls[MOVE_GAIN];	
			
		else if(m_Controls[MOVE_AMPLITUDE_X] > m_QControls[MOVE_AMPLITUDE_X])
			m_Controls[MOVE_AMPLITUDE_X] -= m_Controls[MOVE_GAIN];
			
		if(fabs(m_QControls[MOVE_AMPLITUDE_X] - m_Controls[MOVE_AMPLITUDE_X]) < m_Controls[MOVE_GAIN])
			m_Controls[MOVE_AMPLITUDE_X] = m_QControls[MOVE_AMPLITUDE_X];
		
		// Move Y	
		if(m_Controls[MOVE_AMPLITUDE_Y] < m_QControls[MOVE_AMPLITUDE_Y])
			m_Controls[MOVE_AMPLITUDE_Y] += m_Controls[MOVE_GAIN];	
			
		else if(m_Controls[MOVE_AMPLITUDE_Y] > m_QControls[MOVE_AMPLITUDE_Y])
			m_Controls[MOVE_AMPLITUDE_Y] -= m_Controls[MOVE_GAIN];
			
		if(fabs(m_QControls[MOVE_AMPLITUDE_Y] - m_Controls[MOVE_AMPLITUDE_Y]) < m_Controls[MOVE_GAIN])
			m_Controls[MOVE_AMPLITUDE_Y] = m_QControls[MOVE_AMPLITUDE_Y];
			
		// Move C
		if(m_Controls[MOVE_AMPLITUDE_C] < m_QControls[MOVE_AMPLITUDE_C])
			m_Controls[MOVE_AMPLITUDE_C] += m_Controls[MOVE_GAIN];
				
		else if(m_Controls[MOVE_AMPLITUDE_C] > m_QControls[MOVE_AMPLITUDE_C])
			m_Controls[MOVE_AMPLITUDE_C] -= m_Controls[MOVE_GAIN];
			
		if(fabs(m_QControls[MOVE_AMPLITUDE_C] - m_Controls[MOVE_AMPLITUDE_C]) < m_Controls[MOVE_GAIN])
			m_Controls[MOVE_AMPLITUDE_C] = m_QControls[MOVE_AMPLITUDE_C];
	}
	
	m_Controls[SHIFT_AMPLITUDE_X]  = m_QControls[SHIFT_AMPLITUDE_X];
	m_Controls[SHIFT_AMPLITUDE_Y]  = m_QControls[SHIFT_AMPLITUDE_Y];
	m_Controls[SHIFT_AMPLITUDE_Z]  = m_QControls[SHIFT_AMPLITUDE_Z];
	m_Controls[SHIFT_AMPLITUDE_C]  = m_QControls[SHIFT_AMPLITUDE_C];
	
	m_Controls[OFFSET_R_FOOT_X]    = m_QControls[OFFSET_R_FOOT_X];
	m_Controls[OFFSET_R_FOOT_Y]    = m_QControls[OFFSET_R_FOOT_Y];
	m_Controls[OFFSET_R_FOOT_Z]    = m_QControls[OFFSET_R_FOOT_Z];
	m_Controls[OFFSET_R_FOOT_C]    = m_QControls[OFFSET_R_FOOT_C];
	
	m_Controls[OFFSET_L_FOOT_X]    = m_QControls[OFFSET_L_FOOT_X];
	m_Controls[OFFSET_L_FOOT_Y]    = m_QControls[OFFSET_L_FOOT_Y];
	m_Controls[OFFSET_L_FOOT_Z]    = m_QControls[OFFSET_L_FOOT_Z];
	m_Controls[OFFSET_L_FOOT_C]    = m_QControls[OFFSET_L_FOOT_C];
	
	m_Controls[OFFSET_HIP_PITCH]   = m_QControls[OFFSET_HIP_PITCH];
	m_Controls[OFFSET_HIP_ROLL]    = m_QControls[OFFSET_HIP_ROLL];
	m_Controls[OFFSET_ANKLE_PITCH] = m_QControls[OFFSET_ANKLE_PITCH];
	m_Controls[OFFSET_ANKLE_ROLL]  = m_QControls[OFFSET_ANKLE_ROLL];
}

void Walk::Process()
{
	double shift_x, shift_y, shift_z, shift_c;
	double move_x_r, move_y_r, move_z_r, move_a_r, move_b_r, move_c_r; 
	double move_x_l, move_y_l, move_z_l, move_a_l, move_b_l, move_c_l;
	
	int direction[12] = {1, 1, 1, -1, -1, -1,     1, 1, -1, 1, 1, -1}; // TODO: direction should be defined in joint.h/joint.cpp?
	double endpoint[8];
	double angle[12];
	int out[12];
	
	// When starting a new walk sequence...
	if(m_Time == 0)
	{
		Update_Time_Parameters();	
		Update_Motion_Parameters();
	}
	
	// Calculate shifts
	shift_x = 0.0;
	shift_y = SinWave(m_Time, m_Shift_Time_Y, 0.0, m_Controls[SHIFT_AMPLITUDE_Y], 0.0);
	shift_z = SinWave(m_Time, m_Shift_Time_Z, 0.0, m_Controls[SHIFT_AMPLITUDE_Z], 0.0);
	shift_c = 0.0;
	
	// Calculate moves
	if(m_Time <= m_Step_Time_Start_R)
	{
		move_x_r = -m_Controls[MOVE_AMPLITUDE_X];
		move_y_r = -m_Controls[MOVE_AMPLITUDE_Y];
		move_z_r = 0.0;
		move_c_r = -m_Controls[MOVE_AMPLITUDE_C];
		
		move_x_l = m_Controls[MOVE_AMPLITUDE_X];
		move_y_l = m_Controls[MOVE_AMPLITUDE_Y];
		move_z_l = 0.0;
		move_c_l = m_Controls[MOVE_AMPLITUDE_C];
	}
	else if(m_Time <= m_Step_Time_End_R)
	{
		move_x_r = SinWave(m_Time - m_Step_Time_Start_R, m_Move_Time_X, 0.0, m_Controls[MOVE_AMPLITUDE_X] * 2.0, -m_Controls[MOVE_AMPLITUDE_X]);
		move_y_r = SinWave(m_Time - m_Step_Time_Start_R, m_Move_Time_Y, 0.0, m_Controls[MOVE_AMPLITUDE_Y] * 2.0, -m_Controls[MOVE_AMPLITUDE_Y]);
		move_z_r = SinWave(m_Time - m_Step_Time_Start_R, m_Move_Time_Z, 0.0, m_Controls[MOVE_AMPLITUDE_Z], 0.0);
		move_c_r = SinWave(m_Time - m_Step_Time_Start_R, m_Move_Time_C, 0.0, m_Controls[MOVE_AMPLITUDE_C] * 2.0, -m_Controls[MOVE_AMPLITUDE_C]);
		
		move_x_l = SinWave(m_Time - m_Step_Time_Start_R, m_Move_Time_X, 0.0, m_Controls[MOVE_AMPLITUDE_X] * -2.0, m_Controls[MOVE_AMPLITUDE_X]);
		move_y_l = SinWave(m_Time - m_Step_Time_Start_R, m_Move_Time_Y, 0.0, m_Controls[MOVE_AMPLITUDE_Y] * -2.0, m_Controls[MOVE_AMPLITUDE_Y]);
		move_z_l = 0.0;
		move_c_l = SinWave(m_Time - m_Step_Time_Start_R, m_Move_Time_C, 0.0, m_Controls[MOVE_AMPLITUDE_C] * -2.0, m_Controls[MOVE_AMPLITUDE_C]);
	}
	else if(m_Time <= m_Step_Time_Start_L)
	{
		move_x_r = m_Controls[MOVE_AMPLITUDE_X];
		move_y_r = m_Controls[MOVE_AMPLITUDE_Y];
		move_z_r = 0.0;
		move_c_r = m_Controls[MOVE_AMPLITUDE_C];
		
		move_x_l = -m_Controls[MOVE_AMPLITUDE_X];
		move_y_l = -m_Controls[MOVE_AMPLITUDE_Y];
		move_z_l = 0.0;
		move_c_l = -m_Controls[MOVE_AMPLITUDE_C];
	}
	else if(m_Time <= m_Step_Time_End_L)
	{
		move_x_r = SinWave(m_Time - m_Step_Time_Start_L, m_Move_Time_X, 0.0, m_Controls[MOVE_AMPLITUDE_X] * -2.0, m_Controls[MOVE_AMPLITUDE_X]);
		move_y_r = SinWave(m_Time - m_Step_Time_Start_L, m_Move_Time_Y, 0.0, m_Controls[MOVE_AMPLITUDE_Y] * -2.0, m_Controls[MOVE_AMPLITUDE_Y]);
		move_z_r = 0.0;
		move_c_r = SinWave(m_Time - m_Step_Time_Start_L, m_Move_Time_C, 0.0, m_Controls[MOVE_AMPLITUDE_C] * -2.0, m_Controls[MOVE_AMPLITUDE_C]);
		
		move_x_l = SinWave(m_Time - m_Step_Time_Start_L, m_Move_Time_X, 0.0, m_Controls[MOVE_AMPLITUDE_X] * 2.0, -m_Controls[MOVE_AMPLITUDE_X]);
		move_y_l = SinWave(m_Time - m_Step_Time_Start_L, m_Move_Time_Y, 0.0, m_Controls[MOVE_AMPLITUDE_Y] * 2.0, -m_Controls[MOVE_AMPLITUDE_Y]);
		move_z_l = SinWave(m_Time - m_Step_Time_Start_L, m_Move_Time_Z, 0.0, m_Controls[MOVE_AMPLITUDE_Z], 0.0);
		move_c_l = SinWave(m_Time - m_Step_Time_Start_L, m_Move_Time_C, 0.0, m_Controls[MOVE_AMPLITUDE_C] * 2.0, -m_Controls[MOVE_AMPLITUDE_C]);
	}
	else
	{
		move_x_r = -m_Controls[MOVE_AMPLITUDE_X];
		move_y_r = -m_Controls[MOVE_AMPLITUDE_Y];
		move_z_r = 0.0;
		move_c_r = -m_Controls[MOVE_AMPLITUDE_C];
		
		move_x_l = m_Controls[MOVE_AMPLITUDE_X];
		move_y_l = m_Controls[MOVE_AMPLITUDE_Y];
		move_z_l = 0.0;
		move_c_l = m_Controls[MOVE_AMPLITUDE_C];
	}
	
	// Right foot placement
	endpoint[0] = shift_x + move_x_r + m_Controls[OFFSET_R_FOOT_X];
	endpoint[1] = shift_y + move_y_r + m_Controls[OFFSET_R_FOOT_Y];
	endpoint[2] = shift_z + move_z_r + m_Controls[OFFSET_R_FOOT_Z];
	endpoint[3] = shift_c + move_c_r + m_Controls[OFFSET_R_FOOT_C];
	
	// Left foot placement
	endpoint[4] = shift_x + move_x_l + m_Controls[OFFSET_L_FOOT_X];
	endpoint[5] = shift_y + move_y_l + m_Controls[OFFSET_L_FOOT_Y];
	endpoint[6] = shift_z + move_z_l + m_Controls[OFFSET_L_FOOT_Z];
	endpoint[7] = shift_c + move_c_l + m_Controls[OFFSET_L_FOOT_C];
	
	// Increment time
	m_Time += UNIT_OF_TIME;
	if(m_Time >= m_Controls[PERIOD_TIME])
		m_Time = 0;
	
	// Run IK
	if((Kinematics::LegIK(&angle[0], endpoint[0], endpoint[1], endpoint[2], 0, 0, endpoint[3]) == 1) && (Kinematics::LegIK(&angle[6], endpoint[4], endpoint[5], endpoint[6], 0, 0, endpoint[7]) == 1))
	{
		for(int i = 0; i < 12; i++)
		{
			angle[i] *= 180.0 / 3.141592;
			out[i] = AX12::Angle2Value(direction[i] * angle[i]);	
		}
		
		m_Joint.SetValue(Joint::ID_R_HIP_YAW,     out[0]);
		m_Joint.SetValue(Joint::ID_R_HIP_ROLL,    out[1]);
		m_Joint.SetValue(Joint::ID_R_HIP_PITCH,   out[2] - m_Controls[OFFSET_HIP_PITCH]);
		m_Joint.SetValue(Joint::ID_R_KNEE,        out[3]);
		m_Joint.SetValue(Joint::ID_R_ANKLE_PITCH, out[4] - m_Controls[OFFSET_ANKLE_PITCH]);
		m_Joint.SetValue(Joint::ID_R_ANKLE_ROLL,  out[5] + m_Controls[OFFSET_ANKLE_ROLL]);
		
		m_Joint.SetValue(Joint::ID_L_HIP_YAW,     out[6]);
		m_Joint.SetValue(Joint::ID_L_HIP_ROLL,    out[7]);
		m_Joint.SetValue(Joint::ID_L_HIP_PITCH,   out[8] + m_Controls[OFFSET_HIP_PITCH]);
		m_Joint.SetValue(Joint::ID_L_KNEE,        out[9]);
		m_Joint.SetValue(Joint::ID_L_ANKLE_PITCH, out[10] + m_Controls[OFFSET_ANKLE_PITCH]);
		m_Joint.SetValue(Joint::ID_L_ANKLE_ROLL,  out[11] + m_Controls[OFFSET_ANKLE_ROLL]);
	}
}
