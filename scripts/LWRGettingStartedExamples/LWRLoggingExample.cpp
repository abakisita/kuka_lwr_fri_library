//  ---------------------- Doxygen info ----------------------
//! \file LWRLoggingExample.cpp
//!
//! \brief
//! Sample application for the class LWRJointPositionController using the
//! data logging functionality
//!
//! \details
//! This simple application feature a sample of how to use the
//! joint position controller \em and how to use the data logging
//! functionality of the KUKA Fast Research Interface
//! for the Light-Weight Robot IV. For details about the actual
//! interface class (i.e., class LWRJointPositionController), please
//! refer to the file LWRJointPositionController.h as well as to the
//! file containing the base controller interface class
//! (i.e., class LWRBaseControllerInterface), LWRBaseControllerInterface.h.
//!
//! \date December 2014
//!
//! \version 1.2
//!
//!	\author Torsten Kroeger, tkr@stanford.edu\n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//! \n
//! \n
//! \copyright Copyright 2014 Stanford University\n
//! \n
//! Licensed under the Apache License, Version 2.0 (the "License");\n
//! you may not use this file except in compliance with the License.\n
//! You may obtain a copy of the License at\n
//! \n
//! http://www.apache.org/licenses/LICENSE-2.0\n
//! \n
//! Unless required by applicable law or agreed to in writing, software\n
//! distributed under the License is distributed on an "AS IS" BASIS,\n
//! WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n
//! See the License for the specific language governing permissions and\n
//! limitations under the License.\n
//! 
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <LWRJointPositionController.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>


#ifndef NUMBER_OF_JOINTS
#define NUMBER_OF_JOINTS			7
#endif

#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif



//*******************************************************************************************
// main()
//
int main(int argc, char *argv[])
{
	unsigned int				CycleCounter	=	0;

	int							ResultValue		=	0
							,	i				=	0;

	float						FunctionValue	=	0.0
							,	LoopVariable	=	0.0
							,	JointValuesInRad[NUMBER_OF_JOINTS]
		 					,	InitialJointValuesInRad[NUMBER_OF_JOINTS];

	LWRJointPositionController	*Robot;

	Robot	=	new LWRJointPositionController("/home/lwrcontrol/etc/980039-FRI-Driver.init");

	fprintf(stdout, "RobotJointPositionController object created. Starting the robot...\n");

	ResultValue	=	Robot->StartRobot();

	if (ResultValue == EOK)
	{
		fprintf(stdout, "Robot successfully started.\n");
	}
	else
	{
		fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue));
	}

	fprintf(stdout, "Current system state:\n%s\n", Robot->GetCompleteRobotStateAndInformation());

	Robot->GetMeasuredJointPositions(InitialJointValuesInRad);

	fprintf(stdout, "Preparing data logger...\n");
	// Data logging method 1
	Robot->PrepareLogging("OptionalStringForFileIdentification");
	fprintf(stdout, "Data logger prepared.\n");

	while (LoopVariable < 5.0 * PI)
	{
		Robot->WaitForKRCTick();

		if (!Robot->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			break;
		}

		CycleCounter++;

		if (CycleCounter == 2000)
		{
			// Data logging method 2
			ResultValue	=	Robot->StartLogging();

			if (ResultValue == EOK)
			{
				// this printf method is real-time capable
				Robot->printf("Data logging successfully started.\n");
			}
			else
			{
				Robot->printf("ERROR, cannot start data logging.\n");
			}
		}

		if (CycleCounter == 12000)
		{
			// Data logging method 3
			ResultValue	=	Robot->StopLogging();

			if (ResultValue == EOK)
			{
				Robot->printf("Data logging successfully stopped.\n");
			}
			else
			{
				Robot->printf("ERROR, cannot stop data logging.\n");
			}
		}

		FunctionValue	=	0.3 * sin(LoopVariable);
		FunctionValue	*=	FunctionValue;

		for (i = 0; i < NUMBER_OF_JOINTS; i++)
		{
			JointValuesInRad[i]	=	InitialJointValuesInRad[i] + FunctionValue;
		}

		Robot->SetCommandedJointPositions(JointValuesInRad);

		LoopVariable	+=	(float)0.001;
	}

	fprintf(stdout, "Writing data file...\n");
	// Data logging method 4
	Robot->WriteLoggingDataFile();
	fprintf(stdout, "Data file written.\n");

	fprintf(stdout, "Stopping the robot...\n");
	ResultValue	=	Robot->StopRobot();

	if (ResultValue != EOK)
	{
		fprintf(stderr, "An error occurred during stopping the robot...\n");
	}
	else
	{
		fprintf(stdout, "Robot successfully stopped.\n");
	}

	fprintf(stdout, "Deleting the object...\n");
	delete Robot;
	fprintf(stdout, "Object deleted...\n");

	return(EXIT_SUCCESS);
}
