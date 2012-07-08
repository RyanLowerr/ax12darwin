#ifndef _PORT_H_
#define _PORT_H_

namespace Robot
{
	class Port
	{
		public:
			virtual bool Open() = 0;
			virtual void Close() = 0;
			virtual void Flush() = 0;
			virtual int Write(unsigned char *packet, int packetLength) = 0;
			virtual int Read(unsigned char *packet, int packetLength) = 0;
		
			virtual void SetPacketTimeout(int timeout) = 0;
			virtual bool IsPacketTimeout() = 0;
			virtual double GetCurrentTime() = 0;
			virtual double GetPacketTime() = 0;
	};
}

#endif
