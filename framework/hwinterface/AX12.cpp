#include <stdio.h>
#include "AX12.h"
#include "Port.h"

using namespace Robot;

#define ID                      2
#define LENGTH                  3
#define INSTRUCTION             4
#define ERROR                   4
#define PARAMETER               5
#define MAX_TX_LENGTH           256
#define MAX_RX_LENGTH           256

#define INSTRUCTION_PING        1
#define INSTRUCTION_READ        2
#define INSTRUCTION_WRITE       3
#define INSTRUCTION_REG_WRITE   4
#define INSTRUCTION_ACTION      5
#define INSTRUCTION_RESET       6
#define INSTRUCTION_SYNC_WRITE  131

#define BROADCAST_ID            254

AX12::AX12(Port* port)
{
	mpPort = port;
	mDebug = true;
}

AX12::~AX12()
{
}

bool AX12::Connect()
{
	if(mDebug == true)
			printf("\nOpening port connected to the AX12 bus ... ");
			
	if(mpPort->Open() == false)
	{
		if(mDebug == true)
			printf("Fail");
		return false;
	}
	
	if(mDebug == true)
			printf("Ok");
	
	return true;
}

void AX12::Disconnect()
{
	if(mDebug == true)
		printf("\nClosing port connected to the AX12 bus.");
	mpPort->Close();
}

int AX12::TxRx(unsigned char *txpacket, unsigned char *rxpacket)
{
	int result;
	int length = txpacket[LENGTH] + 4;
	
	// Add the header to the front of the tx packet
	txpacket[0] = (unsigned char) 0xff;
	txpacket[1] = (unsigned char) 0xff;
	
	// Add the checksum to the end of the tx packet
	txpacket[length-1] = (unsigned char) CalculateChecksum(txpacket);
	
	if(mDebug == true)
	{
		// Print the instruction type of the txpacket
		printf("\nInstruction: ");
		switch(txpacket[INSTRUCTION])
		{
			case INSTRUCTION_PING:
				printf("ping");
				break;
			case INSTRUCTION_READ:
				printf("read");
				break;
			case INSTRUCTION_WRITE:
				printf("write");
				break;
			case INSTRUCTION_REG_WRITE:
				printf("register write");
				break;
			case INSTRUCTION_ACTION:
				printf("action");
				break;
			case INSTRUCTION_RESET:
				printf("reset");
				break;
			case INSTRUCTION_SYNC_WRITE:
				printf("sync write");
				break;
		}
		
		// Print the txpacket
		printf("\ntx packet: ");
		for(int i = 0; i  < length; i++)
			printf("%.2X ", txpacket[i]);
	}
	
	// Flush the port connected to the ax12 bus
	mpPort->Flush();
	
	// Write out the tx packet to the ax12 bus
	if(mpPort->Write(txpacket, length) == length)
	{
		// Reading in the rx packet from the ax12 bus...
		if(txpacket[ID] != BROADCAST_ID)
		{
			int to_length = 0;
			int get_length = 0;
			
			if(txpacket[INSTRUCTION] == INSTRUCTION_READ)
				to_length = txpacket[PARAMETER+1] + 6;
			else
				to_length = 6;
				
			if(mDebug == true)
				printf("\nrx packet: ");
			
			// Setting the rx packet timeout
			mpPort->SetPacketTimeout(to_length);
			
			// Read the rx packet
			while(1)
			{
				length = mpPort->Read(&rxpacket[get_length], to_length - get_length);
				
				if(mDebug == true)
					for(int i = 0; i < length; i++)
						printf("%.2X ", rxpacket[get_length + i]);
				
				get_length += length;
				
				// Check for rx packet complete
				if(get_length == to_length)
				{
					// Check for the rx packet's header
					if(rxpacket[0] = 0xff && rxpacket[1] == 0xff)
					{
						// Check the rx packet's checksum
						unsigned char checksum = CalculateChecksum(rxpacket);
						
						if(mDebug == true)
							printf("\nchecksum: %.2X", checksum);
						
						if(rxpacket[get_length-1] == checksum)
							result = 1; //success
						else
							result = 2; // rx corrupt
					}
					else
					{
						result = 2; // rx corrupt
					}
					
					break;
				}
				
				// Check for rx packet timeout
				if(mpPort->IsPacketTimeout() == true)
				{
					result = 3; // rx timeout
					break;
				}
			}
		}
		else
		{
			result = 1; // success
		}
	}
	else
	{
		result = 4; // tx fail
	}
	
	if(mDebug == true)
	{
		printf("\ntime: %.2f ms  ", mpPort->GetPacketTime());
		printf("\nresult: ");
		switch(result)
		{
		case 1:
			printf("sucess");
			break;
		case 2:
			printf("rx corrupt");
			break;
		case 3:
			printf("rx timeout");
			break;
		case 4:
			printf("tx fail");
			break;
		default:
			printf("unknown issue");
			break;
		}
	}
	
	return result;
}

unsigned char AX12::CalculateChecksum(unsigned char *packet)
{
	unsigned char checksum = 0x00;
	for(int i = ID; i < packet[LENGTH] + 3; i++)
		checksum += packet[i];
		
	return (~checksum);
}

int AX12::Ping(int id)
{
	unsigned char txpacket[MAX_TX_LENGTH];
	unsigned char rxpacket[MAX_RX_LENGTH];
	int result;
	
	txpacket[ID]          = (unsigned char) id;
	txpacket[LENGTH]      = (unsigned char) 2;
	txpacket[INSTRUCTION] = (unsigned char) INSTRUCTION_PING;
	
	result = TxRx(txpacket, rxpacket);
	
	return result;
}

int AX12::ReadByte(int id, int address, int *value)
{
	unsigned char txpacket[MAX_TX_LENGTH];
	unsigned char rxpacket[MAX_RX_LENGTH];
	int result;
	
	txpacket[ID]          = (unsigned char) id;
	txpacket[LENGTH]      = (unsigned char) 4;
	txpacket[INSTRUCTION] = (unsigned char) INSTRUCTION_READ;
	txpacket[PARAMETER]   = (unsigned char) address;
	txpacket[PARAMETER+1] = (unsigned char) 1;
	
	result = TxRx(txpacket, rxpacket);
	
	if(result == 1)
		*value = rxpacket[PARAMETER];
	
	return result;
}

int AX12::ReadWord(int id, int address, int *value)
{
	unsigned char txpacket[MAX_TX_LENGTH];
	unsigned char rxpacket[MAX_RX_LENGTH];
	int result;
	
	txpacket[ID]          = (unsigned char) id;
	txpacket[LENGTH]      = (unsigned char) 4;
	txpacket[INSTRUCTION] = (unsigned char) INSTRUCTION_READ;
	txpacket[PARAMETER]   = (unsigned char) address;
	txpacket[PARAMETER+1] = (unsigned char) 2;
	
	result = TxRx(txpacket, rxpacket);
	
	if(result == 1)
		*value = MakeWord(rxpacket[PARAMETER], rxpacket[PARAMETER+1]);
	
	return result;
}

int AX12::ReadTable(int id, int start_address, int end_address, unsigned char *table)  
{
	unsigned char txpacket[MAX_TX_LENGTH];
	unsigned char rxpacket[MAX_RX_LENGTH];
	int result;
	int length = end_address - start_address + 1;
	
	txpacket[ID]          = (unsigned char) id;
	txpacket[LENGTH]      = (unsigned char) 4;
	txpacket[INSTRUCTION] = (unsigned char) INSTRUCTION_READ;
	txpacket[PARAMETER]   = (unsigned char) start_address;
	txpacket[PARAMETER+1] = (unsigned char) length;
	
	result = TxRx(txpacket, rxpacket);
	
	if(result == 1)
		for(int i = 0; i < length; i++)
			table[start_address + i] = rxpacket[PARAMETER + i];
	
	return result;
}

int AX12::WriteByte(int id, int address, int value)
{
	unsigned char txpacket[MAX_TX_LENGTH];
	unsigned char rxpacket[MAX_RX_LENGTH];
	int result;
	
	txpacket[ID]          = (unsigned char) id;
	txpacket[LENGTH]      = (unsigned char) 4;
	txpacket[INSTRUCTION] = (unsigned char) INSTRUCTION_WRITE;
	txpacket[PARAMETER]   = (unsigned char) address;
	txpacket[PARAMETER+1] = (unsigned char) value;
	
	result = TxRx(txpacket, rxpacket);
	
	return result;
}

int AX12::WriteWord(int id, int address, int value)
{
	unsigned char txpacket[MAX_TX_LENGTH];
	unsigned char rxpacket[MAX_RX_LENGTH];
	int result;
	
	txpacket[ID]          = (unsigned char) id;
	txpacket[LENGTH]      = (unsigned char) 5;
	txpacket[INSTRUCTION] = (unsigned char) INSTRUCTION_WRITE;
	txpacket[PARAMETER]   = (unsigned char) address;
	txpacket[PARAMETER+1] = (unsigned char) GetLowByte(value);
	txpacket[PARAMETER+2] = (unsigned char) GetHighByte(value);
	
	result = TxRx(txpacket, rxpacket);
	
	return result;
}

int AX12::SyncWrite(int address, int length, int number, int *param)
{
	unsigned char txpacket[MAX_TX_LENGTH];
	unsigned char rxpacket[MAX_RX_LENGTH];
	int n;
	int result;
	
	txpacket[ID]          = (unsigned char) BROADCAST_ID;
	txpacket[INSTRUCTION] = (unsigned char) INSTRUCTION_SYNC_WRITE;
	txpacket[PARAMETER]   = (unsigned char) address;
	txpacket[PARAMETER+1] = (unsigned char) length;
	
	for (n = 0; n < ((length + 1) * number); n++)
		txpacket[PARAMETER+2+n] = (unsigned char) param[n];
	
	txpacket[LENGTH]      = (unsigned char) (length + 1) * number + 4;
	
	result = TxRx(txpacket, rxpacket);
	
	return result;
}

int AX12::Reset(int id)
{
	unsigned char txpacket[MAX_TX_LENGTH];
	unsigned char rxpacket[MAX_RX_LENGTH];
	int result;
	
	txpacket[ID]          = (unsigned char) id;
	txpacket[LENGTH]      = (unsigned char) 2;
	txpacket[INSTRUCTION] = (unsigned char) INSTRUCTION_RESET;
	
	result = TxRx(txpacket, rxpacket);
	
	return result;
}

int AX12::MakeWord(int lowbyte, int highbyte)
{
	unsigned short word;
	word = highbyte;
	word = word << 8;
	word = word + lowbyte;
	return (int)word;
}

int AX12::GetLowByte(int word)
{
	unsigned short lowbyte;
	lowbyte = word & 0xff;
	return (int)lowbyte;
}

int AX12::GetHighByte(int word)
{
	unsigned short highbyte;
	highbyte = word & 0xff00;
	return (int)(highbyte >> 8);
}

int AX12::Angle2Value(double angle)
{
	return (int)(angle * RATIO_ANGLE2VALUE) + CENTER_VALUE;
}

double AX12::Value2Angle(int value)
{
	return (double)(value - CENTER_VALUE) * RATIO_VALUE2ANGLE;
}
