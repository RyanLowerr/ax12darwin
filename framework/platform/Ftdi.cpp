#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include "Ftdi.h"

using namespace Robot;

//
FTDI::FTDI(char *name)
{
	SetName(name);
	
	m_Debug = false;
	
	m_Socket_fd = -1;
	
	m_PacketTimeStart = 0;
	m_ByteTransferTime = 0.0;
	m_PacketTime = 0;
}

//
FTDI::~FTDI()
{
	Close();
}

//
void FTDI::SetName(char *name)
{
	strcpy(m_PortName, name);
}

//
char* FTDI::GetName()
{
	return (char*)m_PortName;
}

//
bool FTDI::Open()
{
	struct termios terminalOptions;
	struct serial_struct serialOptions;
	double baudrate = 153846.0; //1000000.0;
	baudrate = 1000000.0;
	
	if(m_Debug == true)
		printf("\nAtempting to open %s ... ", m_PortName);
	
	if((m_Socket_fd = open(m_PortName, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0)
		goto OPEN_ERROR;
	
	if(m_Debug == true)
		printf("success!");
	
	// Set terminal baudrate to 38400bps first
	memset(&terminalOptions, 0, sizeof(terminalOptions));
	terminalOptions.c_cflag      = B38400|CS8|CLOCAL|CREAD; // Control options
	terminalOptions.c_lflag      = 0;                       // Local options
	terminalOptions.c_iflag      = IGNPAR;                  // Input options
	terminalOptions.c_oflag      = 0;                       // Output options
	terminalOptions.c_cc[VTIME]  = 0;                       // Control characteristics
	terminalOptions.c_cc[VMIN]   = 0;
	tcsetattr(m_Socket_fd, TCSANOW, &terminalOptions);
	
	// Setting non-standard baudrate
	if(m_Debug == true)
		printf("\nSetting baudrate to %.1fbps ... ", baudrate);
	
	// Use ioctl TTOCGSERIAL to get the serial_structure for the device
	if(ioctl(m_Socket_fd, TIOCGSERIAL, &serialOptions) < 0)
		goto OPEN_ERROR;
	
	// Set ASYNC_SPD_MASK and ASYNC_SPD_CUST flags and set the serial_structure's custom divisor
	serialOptions.flags &= ~ASYNC_SPD_MASK;
	serialOptions.flags |= ASYNC_SPD_CUST;
	serialOptions.custom_divisor = serialOptions.baud_base / baudrate;
	
	// Use ioctl to write back the serial_structure of the device
	if(ioctl(m_Socket_fd, TIOCSSERIAL, &serialOptions) < 0)
		goto OPEN_ERROR;
	
	if(m_Debug == true)
		printf("success!");
	
	Flush();
	
	return true;
	
	OPEN_ERROR:
	if(m_Debug == true)
		printf("failed!");
	Close();
	return false;
}

//
void FTDI::Close()
{
	if(m_Socket_fd != -1)
		close(m_Socket_fd);
		
	m_Socket_fd = -1;
}

//
void FTDI::Flush()
{
	tcflush(m_Socket_fd, TCIFLUSH);
}

//
int FTDI::Write(unsigned char *packet, int packetLength)
{
	return write(m_Socket_fd, packet, packetLength);
}

//
int FTDI::Read(unsigned char *packet, int packetLength)
{
	return read(m_Socket_fd, packet, packetLength);
}

//
void FTDI::SetPacketTimeout(int lengthPacket)
{
	m_PacketTimeStart = GetCurrentTime();
	m_PacketTime = m_ByteTransferTime * lengthPacket + 5.0;
}

//
bool FTDI::IsPacketTimeout()
{
	if(GetPacketTime() > m_PacketTime)
		return true;
		
	return false;
}

//
double FTDI::GetCurrentTime()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (double)(tv.tv_sec * 1000.0) + (double)(tv.tv_usec / 1000.0);
}

//
double FTDI::GetPacketTime()
{
	return (double)(GetCurrentTime() - m_PacketTimeStart);
}
