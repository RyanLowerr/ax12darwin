#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "AX12.h"
#include "Ftdi.h"
#include "Manager.h"
#include "Walk.h"
#include "Head.h"

#define PROGRAM_VERSION "v0.0"

using namespace Robot;

void clearscreen()
{
	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
}

void programname()
{
	printf("\n\nAX12 Darwin Walking Tuner %s", PROGRAM_VERSION);
}

void prompt()
{
	printf("\n> ");
}

void walkinfo()
{
	printf("\n");
	printf("\n 0   Period Time         %.2f", Walk::GetInstance()->Get_Control(Walk::PERIOD_TIME));
	printf("\n 1   Double Leg Ratio    %.2f", Walk::GetInstance()->Get_Control(Walk::DOUBLE_LEG_RATIO));
	printf("\n 2   Move Gain           %.2f", Walk::GetInstance()->Get_Control(Walk::MOVE_GAIN));
	printf("\n 3   Move Amplitude X    %.2f", Walk::GetInstance()->Get_Control(Walk::MOVE_AMPLITUDE_X));
	printf("\n 4   Move Amplitude Y    %.2f", Walk::GetInstance()->Get_Control(Walk::MOVE_AMPLITUDE_Y));
	printf("\n 5   Move Amplitude Z    %.2f", Walk::GetInstance()->Get_Control(Walk::MOVE_AMPLITUDE_Z));
	printf("\n 6   Move Amplitude C    %.2f", Walk::GetInstance()->Get_Control(Walk::MOVE_AMPLITUDE_C));
	printf("\n 7   Shift Amplitude X   %.2f", Walk::GetInstance()->Get_Control(Walk::SHIFT_AMPLITUDE_X));
	printf("\n 8   Shift Amplitude Y   %.2f", Walk::GetInstance()->Get_Control(Walk::SHIFT_AMPLITUDE_Y));
	printf("\n 9   Shift Amplitude Z   %.2f", Walk::GetInstance()->Get_Control(Walk::SHIFT_AMPLITUDE_Z));
	printf("\n 10  Shift Amplitude C   %.2f", Walk::GetInstance()->Get_Control(Walk::SHIFT_AMPLITUDE_C));
	printf("\n 11  Offset RFoot X      %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_R_FOOT_X));
	printf("\n 12  Offset RFoot Y      %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_R_FOOT_Y));
	printf("\n 13  Offset RFoot Z      %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_R_FOOT_Z));
	printf("\n 14  Offset RFoot C      %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_R_FOOT_C));
	printf("\n 15  Offset LFoot X      %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_L_FOOT_X));
	printf("\n 16  Offset LFoot Y      %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_L_FOOT_Y));
	printf("\n 17  Offset LFoot Z      %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_L_FOOT_Z));
	printf("\n 18  Offset LFoot C      %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_L_FOOT_C));
	printf("\n 19  Offset Hip Pitch    %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_HIP_PITCH));
	printf("\n 20  Offset Hip Roll     %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_HIP_ROLL));
	printf("\n 21  Offset Ankle Pitch  %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_ANKLE_PITCH));
	printf("\n 22  Offset Ankle Roll   %.2f", Walk::GetInstance()->Get_Control(Walk::OFFSET_ANKLE_ROLL));
	printf("\n");
}

int main()
{
	//
	programname();

	// Setup the FTDI usb adaptor
	printf("\n\nSetting up the FTDI USB adaptor ...");
	FTDI AXPort((char *)"/dev/ttyUSB0");
	
	// Setup the AX12 bus with the FTDI port
	printf("\n\nSetting up the AX12 bus with the FTDI port ...");
	AX12 AXBus(&AXPort);
	
	// Attempt to initalize the motion manager
	printf("\n\nInitalizing the motion manager ... ");
	if(MotionManager::GetInstance()->Initialize(&AXBus) == false)
	{
		printf("Fail!");
	}
	else
	{
		// Add the Walk motion to the motion manager
		printf("\n\nAdding motion 'Walk' to the motion manager ...");
		MotionManager::GetInstance()->AddMotion((Walk*)Walk::GetInstance());
		Walk::GetInstance()->m_Joint.SetEnableAll(true);
		Walk::GetInstance()->m_Joint.SetEnableHead(false);
		Walk::GetInstance()->m_Joint.SetEnableArms(false);
		
		// Wait for user to press any key before continuing
		printf("\n\nProgram Ready. Press the any key to begin!\n");
		getchar();
		
		Walk::GetInstance()->Set_Control(Walk::PERIOD_TIME, 1500.0);
		Walk::GetInstance()->Set_Control(Walk::DOUBLE_LEG_RATIO, 0.1);
		Walk::GetInstance()->Set_Control(Walk::OFFSET_R_FOOT_Z, 25.0);
		Walk::GetInstance()->Set_Control(Walk::OFFSET_L_FOOT_Z, 25.0);
		Walk::GetInstance()->Set_Control(Walk::OFFSET_L_FOOT_X, 10.0);
		Walk::GetInstance()->Set_Control(Walk::OFFSET_HIP_PITCH, 20.0);
		
		// Start the motion manager.
		MotionManager::GetInstance()->Running(true);
		
		
		// Main loop (froot?)
		char input[200];
		int input_length;
		char *token;
		char command[20];
		char param[20][20];
		int int_param[20];
		int num_param;
		int value;
	
		while(1)
		{
			clearscreen();
			programname();
			walkinfo();
			prompt();
			
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
			
			if(strcmp(command, "exit") == 0)
			{
				AXPort.Close();
				MotionManager::GetInstance()->Running(false);
				break;
			}
			else
			{
				Walk::GetInstance()->Set_Control(atoi(command), atoi(param[0]));
			}
				
		}
		
	}
	
	printf("\nTerminating progarm\n");
	return 0;
}
