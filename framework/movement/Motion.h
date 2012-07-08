#ifndef _MOTION_H_
#define _MOTION_H_

#include "Joint.h"

namespace Robot
{
	class Motion
	{
		public:
			
			Joint m_Joint;
	
			static const int UNIT_OF_TIME = 1; 
		
			virtual void Initialize() = 0;
			virtual void Process() = 0;
	};
}

#endif
