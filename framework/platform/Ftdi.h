#ifndef _FTDI_H_
#define _FTDI_H_

#include "Ftdi.h"
#include "Port.h"

namespace Robot
{
	class FTDI : public Port
	{
		public:
		
			FTDI(char *name);
			~FTDI();
		
			void SetName(char *name);
			char* GetName();
		
			/* ---------------------------------------------------------- */
			bool Open();
			void Close();
			void Flush();
			int Write(unsigned char *packet, int packetLength);
			int Read(unsigned char *packet, int packetLength);
		
			void SetPacketTimeout(int timeout);
			bool IsPacketTimeout();
			double GetCurrentTime();
			double GetPacketTime();
			/* ---------------------------------------------------------- */
			
		private:
		
			bool m_Debug;
			int m_Socket_fd;
			char m_PortName[20];
			double m_PacketTimeStart;
			double m_ByteTransferTime;
			double m_PacketTime;
	};
}

#endif
