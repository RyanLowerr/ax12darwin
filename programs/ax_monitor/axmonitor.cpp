#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "AX12.h"
#include "Ftdi.h"

#define PROGRAM_VERSION "v0.0"

using namespace std;
using namespace Robot;

FTDI AXPort((char *)"/dev/ttyUSB0");
AX12 AX12Bus(&AXPort);

void Help()
{
	printf("\nAX12 Monitor %s", PROGRAM_VERSION);
	printf("\n");
	printf("\nexit   - Exits the program.");
	printf("\nhelp   - Prints the help info.");
	printf("\nid     - Change the working device ID.");
	printf("\nscan   - Scans the bus for connected devices.");
	printf("\nping   - Checks for the existance of the device on the bus.");
	printf("\ntable  - Read the control table for a specific device.");
	printf("\nreadb  - Read byte from the control table of a specific device.");
	printf("\nwriteb - Write byte to the control table of a specific device.");
	printf("\nreadw  - Read word from the control table of a specific device.");
	printf("\nwritew - Write word to the control table of a specific device.");
	printf("\nreset  - Returns device to factory defaults.");
	printf("\n");
}

void BadCommand()
{
	printf("\nBad Command!");
	printf("\nType 'help' to print a list of valid commands and thier uses.");
}

void BadParam()
{
	printf("\nBad Param!");
	printf("\nType 'help' to print a list of valid commands and thier uses.");
}

bool CheckNumParams(int num_required, int num_params)
{
	return (num_required != num_params);
}

bool CheckParamInRange(int low, int high, int param)
{
	return ((param > low) && (param < high));
}

bool CheckParamOutRange(int low, int high, int param)
{
	return ((param < low) || (param > high));
}

void Prompt(int id)
{
	printf("\n\nID:%d > ", id);
}

int main()
{
	char input[200];
	int input_length;
	char *token;
	int id = 1;
	char command[20];
	char param[20][20];
	int int_param[20];
	int num_param;
	unsigned char table[50];
	int value;
	
	Help();
	
	if(AX12Bus.Connect() == true)
	{
		while(1)
		{
			Prompt(id);
			
			// Get the user's input
			gets(input);
			fflush(stdin);
			input_length = strlen(input);
			
			if(input_length == 0)
				continue;
				
			token = strtok(input, " ");
			if(token == 0)
				continue;
			
			// Coppy the command from the user's input
			strcpy(command, token);
			
			// Coppy the paramaters from the user's input
			token = strtok(0, " ");
			num_param = 0;
			while(token != 0)
			{
				strcpy(param[num_param++], token);
				token = strtok(0, " ");
			}
			
			// Exit
			if(strcmp(command, "exit") == 0)
			{
				AXPort.Close();
				break;
			}
			// Id
			else if(strcmp(command, "id") == 0)
			{
				if(CheckNumParams(1, num_param))
				{
					BadParam();
					continue;
				}
				
				if(CheckParamOutRange(1, 253, atoi(param[0]))) 
				{
					BadParam();
					continue;
				}
				
				id = atoi(param[0]);
			}
			// Scan
			else if(strcmp(command, "scan") == 0)
			{
				if(CheckNumParams(2, num_param))
				{
					BadParam();
					continue;
				}
				
				if(CheckParamOutRange(1, 253, atoi(param[0]))) 
				{
					BadParam();
					continue;
				}
				
				if(CheckParamOutRange(1, 253, atoi(param[1]))) 
				{
					BadParam();
					continue;
				}
				
				if(atoi(param[0]) > atoi(param[1]))
				{
					BadParam();
					continue;
				}
				
				printf("\nScanning bus (ids %d to %d) ...", atoi(param[0]), atoi(param[1]));
				for(int i = atoi(param[0]); i < atoi(param[1]) + 1; i++)
				{
					printf("\n  ID: %d ... ", i);
					if(AX12Bus.Ping(i) == 1)
						printf("OK");
					else
						printf("Fail");
				}
			}
			// Ping
			else if(strcmp(command, "ping") == 0)
			{
				printf("\nPinging ID: %d ... ", id);
				if(AX12Bus.Ping(id) == 1)
					printf("Success!");
				else
					printf("Fail ping!");
			}
			// Table
			else if(strcmp(command, "table") == 0)
			{
				printf("\nReading control table ... ");
				if(AX12Bus.ReadTable(id, AX12Bus.P_MODEL_NUMBER_L, AX12Bus.P_PUNCH_H, table) == 1)
				{
					printf("\n %.2d MODEL NUMBER           %d", AX12Bus.P_MODEL_NUMBER_L,         AX12Bus.MakeWord(table[AX12Bus.P_MODEL_NUMBER_L], table[AX12Bus.P_MODEL_NUMBER_H]));
					printf("\n %.2d VERSION                %d", AX12Bus.P_VERSION,                table[AX12Bus.P_VERSION]);
					printf("\n %.2d ID                     %d", AX12Bus.P_ID,                     table[AX12Bus.P_ID]);
					printf("\n %.2d BAUD RATE              %d", AX12Bus.P_BAUD_RATE,              table[AX12Bus.P_BAUD_RATE]);
					printf("\n %.2d RETURN DELAY TIME      %d", AX12Bus.P_RETURN_DELAY_TIME,      table[AX12Bus.P_RETURN_DELAY_TIME]);
					printf("\n %.2d CW ANGLE LIMIT         %d", AX12Bus.P_CW_ANGLE_LIMIT_L,       AX12Bus.MakeWord(table[AX12Bus.P_CW_ANGLE_LIMIT_L], table[AX12Bus.P_CW_ANGLE_LIMIT_H]));
					printf("\n %.2d CCW ANGLE LIMIT        %d", AX12Bus.P_CCW_ANGLE_LIMIT_L,      AX12Bus.MakeWord(table[AX12Bus.P_CCW_ANGLE_LIMIT_L], table[AX12Bus.P_CCW_ANGLE_LIMIT_H]));
					printf("\n %.2d LIMIT TEMPERATURE      %d", AX12Bus.P_LIMIT_TEMPERATURE,      table[AX12Bus.P_LIMIT_TEMPERATURE]);
					printf("\n %.2d LOW LIMIT VOLTAGE      %d", AX12Bus.P_LOW_LIMIT_VOLTAGE,      table[AX12Bus.P_LOW_LIMIT_VOLTAGE]);
					printf("\n %.2d HIGH LIMIT VOLTAGE     %d", AX12Bus.P_HIGH_LIMIT_VOLTAGE,     table[AX12Bus.P_HIGH_LIMIT_VOLTAGE]);
					printf("\n %.2d MAX TORQUE             %d", AX12Bus.P_MAX_TORQUE_L,           AX12Bus.MakeWord(table[AX12Bus.P_MAX_TORQUE_L], table[AX12Bus.P_MAX_TORQUE_H]));
					printf("\n %.2d RETURN LEVEL           %d", AX12Bus.P_RETURN_LEVEL,           table[AX12Bus.P_RETURN_LEVEL]);
					printf("\n %.2d ALARM LED              %d", AX12Bus.P_ALARM_LED,              table[AX12Bus.P_ALARM_LED]);
					printf("\n %.2d ALARM SHUTDOWN         %d", AX12Bus.P_ALARM_SHUTDOWN,         table[AX12Bus.P_ALARM_SHUTDOWN]);
					printf("\n %.2d DOWN CALIBRATION       %d", AX12Bus.P_DOWN_CALIBRATION_L,     AX12Bus.MakeWord(table[AX12Bus.P_DOWN_CALIBRATION_L], table[AX12Bus.P_DOWN_CALIBRATION_H]));
					printf("\n %.2d UP CALIBRATION         %d", AX12Bus.P_UP_CALIBRATION_L,       AX12Bus.MakeWord(table[AX12Bus.P_UP_CALIBRATION_L], table[AX12Bus.P_UP_CALIBRATION_H]));
					printf("\n %.2d TORQUE ENABLE          %d", AX12Bus.P_TORQUE_ENABLE,          table[AX12Bus.P_TORQUE_ENABLE]);
					printf("\n %.2d LED                    %d", AX12Bus.P_LED,                    table[AX12Bus.P_LED]);
					printf("\n %.2d CW COMPLIANCE MARGIN   %d", AX12Bus.P_CW_COMPLIANCE_MARGIN,   table[AX12Bus.P_CW_COMPLIANCE_MARGIN]);
					printf("\n %.2d CCW COMPLIANCE MARGIN  %d", AX12Bus.P_CCW_COMPLIANCE_MARGIN,  table[AX12Bus.P_CCW_COMPLIANCE_MARGIN]);
					printf("\n %.2d CW COMPLIANCE SLOPE    %d", AX12Bus.P_CW_COMPLIANCE_SLOPE,    table[AX12Bus.P_CW_COMPLIANCE_SLOPE]);
					printf("\n %.2d CCW COMPLIANCE SLOPE   %d", AX12Bus.P_CCW_COMPLIANCE_SLOPE,   table[AX12Bus.P_CCW_COMPLIANCE_SLOPE]);
					printf("\n %.2d GOAL POSITION          %d", AX12Bus.P_GOAL_POSITION_L,        AX12Bus.MakeWord(table[AX12Bus.P_GOAL_POSITION_L], table[AX12Bus.P_GOAL_POSITION_H]));
					printf("\n %.2d GOAL SPEED             %d", AX12Bus.P_GOAL_SPEED_L,           AX12Bus.MakeWord(table[AX12Bus.P_GOAL_SPEED_L], table[AX12Bus.P_GOAL_SPEED_H]));
					printf("\n %.2d TORQUE LIMIT           %d", AX12Bus.P_TORQUE_LIMIT_L,         AX12Bus.MakeWord(table[AX12Bus.P_TORQUE_LIMIT_L], table[AX12Bus.P_TORQUE_LIMIT_H]));
					printf("\n %.2d PRESENT POSITION       %d", AX12Bus.P_PRESENT_POSITION_L,     AX12Bus.MakeWord(table[AX12Bus.P_PRESENT_POSITION_L], table[AX12Bus.P_PRESENT_POSITION_H]));
					printf("\n %.2d PRESENT SPEED          %d", AX12Bus.P_PRESENT_SPEED_L,        AX12Bus.MakeWord(table[AX12Bus.P_PRESENT_SPEED_L], table[AX12Bus.P_PRESENT_SPEED_H]));
					printf("\n %.2d PRESENT LOAD           %d", AX12Bus.P_PRESENT_LOAD_L,         AX12Bus.MakeWord(table[AX12Bus.P_PRESENT_LOAD_L], table[AX12Bus.P_PRESENT_LOAD_H]));
					printf("\n %.2d PRESENT VOLTAGE        %d", AX12Bus.P_PRESENT_VOLTAGE,        table[AX12Bus.P_PRESENT_VOLTAGE]);
					printf("\n %.2d PRESENT TEMPERATURE    %d", AX12Bus.P_PRESENT_TEMPERATURE,    table[AX12Bus.P_PRESENT_TEMPERATURE]);
					printf("\n %.2d REGISTERED INSTRUCTION %d", AX12Bus.P_REGISTERED_INSTRUCTION, table[AX12Bus.P_REGISTERED_INSTRUCTION]);
					printf("\n %.2d MOVING                 %d", AX12Bus.P_MOVING,                 table[AX12Bus.P_MOVING]);
					printf("\n %.2d LOCK                   %d", AX12Bus.P_LOCK,                   table[AX12Bus.P_LOCK]);
					printf("\n %.2d PUNCH                  %d", AX12Bus.P_PUNCH_L,                AX12Bus.MakeWord(table[AX12Bus.P_PUNCH_L], table[AX12Bus.P_PUNCH_H]));
				}
				else
					printf("Fail table read!");
			}
			// Read Byte
			else if(strcmp(command, "readb") == 0)
			{
				if(CheckNumParams(1, num_param))
				{
					BadParam();
					continue;
				}
				
				if(CheckParamOutRange(AX12Bus.P_MODEL_NUMBER_L, AX12Bus.P_PUNCH_H, atoi(param[0]))) 
				{
					BadParam();
					continue;
				}
				
				printf("\nReading byte ... ");
				if(AX12Bus.ReadByte(id, atoi(param[0]), &value) == 1)
					printf("%d", value);
				else
					printf("Fail readb!");
			}
			// Write Byte
			else if(strcmp(command, "writeb") == 0)
			{
				if(CheckNumParams(2, num_param))
				{
					BadParam();
					continue;
				}
				
				if(CheckParamOutRange(AX12Bus.P_MODEL_NUMBER_L, AX12Bus.P_PUNCH_H, atoi(param[0])))
				{
					BadParam();
					continue;
				}
				
				if(CheckParamOutRange(0, 255, atoi(param[1]))) 
				{
					BadParam();
					continue;
				}
				
				printf("\nWriting byte ... ");
				if(AX12Bus.WriteByte(id, atoi(param[0]), atoi(param[1])) == 1)
					printf("OK");
				else
					printf("Fail writeb!");
			}
			// Read Word
			else if(strcmp(command, "readw") == 0)
			{
				if(CheckNumParams(1, num_param))
				{
					BadParam();
					continue;
				}
				
				if(CheckParamOutRange(AX12Bus.P_MODEL_NUMBER_L, AX12Bus.P_PUNCH_H, atoi(param[0]))) 
				{
					BadParam();
					continue;
				}
				
				printf("\nReading Word ... ");
				if(AX12Bus.ReadWord(id, atoi(param[0]), &value) == 1)
					printf("%d", value);
				else
					printf("Fail readw!");
			}
			// Write Word
			else if(strcmp(command, "writew") == 0)
			{
				if(CheckNumParams(2, num_param))
				{
					BadParam();
					continue;
				}
				
				if(CheckParamOutRange(AX12Bus.P_MODEL_NUMBER_L, AX12Bus.P_PUNCH_H, atoi(param[0]))) 
				{
					BadParam();
					continue;
				}
				
				if(CheckParamOutRange(0, 1023, atoi(param[1]))) 
				{
					BadParam();
					continue;
				}
				
				printf("\nWriting word ... ");
				if(AX12Bus.WriteWord(id, atoi(param[0]), atoi(param[1])) == 1)
					printf("OK");
				else
					printf("Fail writew!");
			}
			// Reset
			else if(strcmp(command, "reset") == 0)
			{
				printf("\nResetting ... ");
				if(AX12Bus.Reset(id) == 1)
					printf("OK");
				else
					printf("Fail reset!");
			}
			// Help
			else if(strcmp(command, "help") == 0)
			{
				Help();
			}
			// Unknown
			else
			{
				BadCommand();
			}
		}
	}
	else
		printf("\nUnable to open port!\n");
	
	AX12Bus.Disconnect();
	printf("\n");
	return 0;
}
