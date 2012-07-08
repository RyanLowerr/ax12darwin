#ifndef _JOINT_H_
#define _JOINT_H_

namespace Robot
{
	class Joint
	{
	public:
		enum
		{
			ID_R_SHOULDER_PITCH     = 1,
			ID_L_SHOULDER_PITCH     = 2,
			ID_R_SHOULDER_ROLL      = 3,
			ID_L_SHOULDER_ROLL      = 4,
			ID_R_ELBOW              = 5,
			ID_L_ELBOW              = 6,
			ID_R_HIP_YAW            = 7,
			ID_L_HIP_YAW            = 8,
			ID_R_HIP_ROLL           = 9,
			ID_L_HIP_ROLL           = 10,
			ID_R_HIP_PITCH          = 11,
			ID_L_HIP_PITCH          = 12,
			ID_R_KNEE               = 13,
			ID_L_KNEE               = 14,
			ID_R_ANKLE_PITCH        = 15,
			ID_L_ANKLE_PITCH        = 16,
			ID_R_ANKLE_ROLL         = 17,
			ID_L_ANKLE_ROLL         = 18,
			ID_HEAD_PAN             = 19,
			ID_HEAD_TILT            = 20,
			NUMBER_OF_JOINTS
		};
		
	protected:
		bool m_Enable[NUMBER_OF_JOINTS];
		int m_Value[NUMBER_OF_JOINTS];
		double m_Angle[NUMBER_OF_JOINTS];
		
	public:
		Joint();
		~Joint();
		
		void SetEnable(int id, bool enable);
		void SetEnableAll(bool enable);
		void SetEnableLegs(bool enable);
		void SetEnableHead(bool enable);
		void SetEnableArms(bool enable);
		bool GetEnable(int id);
		
		void SetValue(int id, int value);
		int GetValue(int id);
		
		void SetAngle(int id, double angle);
		double GetAngle(int id);
		
		void SetRadian(int id, double radian);
		double GetRadian(int id);
	};
}

#endif
