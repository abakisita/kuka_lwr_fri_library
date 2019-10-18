//  ---------------------- Doxygen info ----------------------
//! \file MoveToCandle.cpp
//!
//! \brief
//! Implementation file for performing a joint space motion to 
//! the candle position of the robot
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


#include <FastResearchInterface.h>
#include <Console.h>
#include <errno.h>
#include <string.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <TypeIRML.h>

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif

#ifndef NUMBER_OF_JOINTS
#define NUMBER_OF_JOINTS			7
#endif


// ****************************************************************
// MoveToCandle()
//
void MoveToCandle(FastResearchInterface *FRI)
{
	unsigned int				i							=	0		;

	int							ResultValue					=	0		;


	float						JointValuesInRad[NUMBER_OF_JOINTS]		;

	double						CycleTime					=	0.002	;

	TypeIRML					*RML						=	NULL	;

	TypeIRMLInputParameters		*IP							=	NULL	;

	TypeIRMLOutputParameters	*OP							=	NULL	;

	RML					=	new TypeIRML(		NUMBER_OF_JOINTS
											,	CycleTime			);

	IP					=	new TypeIRMLInputParameters(NUMBER_OF_JOINTS);

	OP					=	new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);

	memset(JointValuesInRad						, 0x0, NUMBER_OF_JOINTS * sizeof(float));

	if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
	{	
		printf("Program is going to stop the robot.\n");
		FRI->StopRobot();

		FRI->GetMeasuredJointPositions(JointValuesInRad);
		FRI->SetCommandedJointPositions(JointValuesInRad);
				
		printf("Restarting the joint position control scheme.\n");
		ResultValue	=	FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
		
		if ((ResultValue != EOK) && (ResultValue != EALREADY))
		{
			printf("An error occurred during starting up the robot...\n");
			delete	RML;
			delete	IP;
			delete	OP;
			
			return;			
		}
	}
		
	printf("Moving to the candle position..\n");
	
	FRI->GetMeasuredJointPositions(JointValuesInRad);
	
	for ( i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
		IP->TargetPosition->VecData			[i]	=	(double)0.0;
		IP->MaxVelocity->VecData			[i] =	(double)50.0;		
		IP->MaxAcceleration->VecData		[i] =	(double)50.0;
		IP->SelectionVector->VecData		[i] =	true;
	}
	
	ResultValue	=	TypeIRML::RML_WORKING;
	
	while ((FRI->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
	{
		FRI->WaitForKRCTick();

		ResultValue	=	RML->GetNextMotionState_Position(		*IP
															,	OP	);

		if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
		{
			printf("MoveToCandle(): ERROR during trajectory generation (%d).", ResultValue);
		}
											
		for ( i = 0; i < NUMBER_OF_JOINTS; i++)
		{
			JointValuesInRad[i]	=	RAD((double)(OP->NewPosition->VecData[i]));
		}

		FRI->SetCommandedJointPositions(JointValuesInRad);	
		
		*(IP->CurrentPosition)		=	*(OP->NewPosition);
		*(IP->CurrentVelocity)		=	*(OP->NewVelocity);
	}

	if (!FRI->IsMachineOK())
	{
		printf("MoveToCandle(): ERROR, machine is not ready.");
		
		delete	RML;
		delete	IP;
		delete	OP;
		
		return;
	}

	printf("Stopping the robot.\n");
	FRI->StopRobot();	

	delete	RML;
	delete	IP;
	delete	OP;
	
	return;
}
