#ifndef _AX_H_
#define _AX_H_

#include "Port.h"

namespace Robot
{
	class AX12
	{
		public:
			
			static const int MIN_VALUE            = 0;
			static const int MAX_VALUE            = 1023;
			static const int CENTER_VALUE         = 512;
			static const double MIN_ANGLE         = -150;
			static const double MAX_ANGLE         = 150;
			static const double RATIO_VALUE2ANGLE = 0.293;
			static const double RATIO_ANGLE2VALUE = 3.413;
			
			enum
			{
				SUCCESS    = 1,
				RX_CORRUPT = 2,
				RX_TIMEOUT = 3,
				TX_FAIL    = 4
			};
			
			enum
			{
				P_MODEL_NUMBER_L          = 0,
				P_MODEL_NUMBER_H          = 1,
				P_VERSION                 = 2,
				P_ID                      = 3,
				P_BAUD_RATE               = 4,
				P_RETURN_DELAY_TIME       = 5,
				P_CW_ANGLE_LIMIT_L        = 6,
				P_CW_ANGLE_LIMIT_H        = 7,
				P_CCW_ANGLE_LIMIT_L       = 8,
				P_CCW_ANGLE_LIMIT_H       = 9,
				P_LIMIT_TEMPERATURE       = 11,
				P_LOW_LIMIT_VOLTAGE       = 12,
				P_HIGH_LIMIT_VOLTAGE      = 13,
				P_MAX_TORQUE_L            = 14,
				P_MAX_TORQUE_H            = 15,
				P_RETURN_LEVEL            = 16,
				P_ALARM_LED               = 17,
				P_ALARM_SHUTDOWN          = 18,
				P_DOWN_CALIBRATION_L      = 20,
				P_DOWN_CALIBRATION_H      = 21,
				P_UP_CALIBRATION_L        = 22,
				P_UP_CALIBRATION_H        = 23,
				P_TORQUE_ENABLE           = 24,
				P_LED                     = 25,
				P_CW_COMPLIANCE_MARGIN    = 26,
				P_CCW_COMPLIANCE_MARGIN   = 27,
				P_CW_COMPLIANCE_SLOPE     = 28,
				P_CCW_COMPLIANCE_SLOPE    = 29,
				P_GOAL_POSITION_L         = 30,
				P_GOAL_POSITION_H         = 31,
				P_GOAL_SPEED_L            = 32,
				P_GOAL_SPEED_H            = 33,
				P_TORQUE_LIMIT_L          = 34,
				P_TORQUE_LIMIT_H          = 35,
				P_PRESENT_POSITION_L      = 36,
				P_PRESENT_POSITION_H      = 37,
				P_PRESENT_SPEED_L         = 38,
				P_PRESENT_SPEED_H         = 39,
				P_PRESENT_LOAD_L          = 40,
				P_PRESENT_LOAD_H          = 41,
				P_PRESENT_VOLTAGE         = 42,
				P_PRESENT_TEMPERATURE     = 43,
				P_REGISTERED_INSTRUCTION  = 44,
				P_MOVING                  = 46,
				P_LOCK                    = 47,
				P_PUNCH_L                 = 48,
				P_PUNCH_H                 = 49,
				MAXIMUM_ADDRESS
			}; 
			
			AX12(Port *port);
			~AX12();
			
			bool Connect();
			void Disconnect();
			
			int Ping(int id);
			int ReadByte(int id, int address, int *value);
			int ReadWord(int id, int address, int *value);
			int ReadTable(int id, int start_address, int end_address, unsigned char *table);
			int WriteByte(int id, int address, int value);
			int WriteWord(int id, int address, int value);
			int SyncWrite(int address, int length, int number, int *param);
			int Reset(int id);
			
			static int MakeWord(int lowbyte, int highbyte);
			static int GetLowByte(int word);
			static int GetHighByte(int word);
		
			static int Angle2Value(double angle);
			static double Value2Angle(int value);
		
		private:
			
			bool mDebug;
			Port* mpPort;
			
			int TxRx(unsigned char *txpacket, unsigned char *rxpacket);
			unsigned char CalculateChecksum(unsigned char *packet);
	};
}

#endif
