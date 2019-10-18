#include <FastResearchInterface.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <mutex>
#include <iostream>
#include <kdl/frames_io.hpp>


#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK		2000

#define SIZE_OF_TRANSFER_STRING					32

//*******************************************************************************************
// main()

int main(int argc, char *argv[])
{
	bool Run = true, StartRobotCalled = false;
	char c = 0,	d =	0, TransferString[SIZE_OF_TRANSFER_STRING];
	unsigned int ControlScheme = FastResearchInterface::JOINT_POSITION_CONTROL, i =	0, LoopValue = 0;
	int ResultValue = 0;
	float FloatValues[SIZE_USER_DATA], TmpFloatValues[SIZE_USER_DATA], DesiredTorqueValues[NUMBER_OF_JOINTS],
		  JointStiffnessValues[NUMBER_OF_JOINTS], JointDampingValues[NUMBER_OF_JOINTS],
		  CartStiffnessValues[NUMBER_OF_CART_DOFS],	MeasuredPose[12], CmndPose[NUMBER_OF_CART_DOFS],
		  CartDampingValues[NUMBER_OF_CART_DOFS];
	float eeFT[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	memset(TransferString, 0x0, SIZE_OF_TRANSFER_STRING	* sizeof(char));
	memset(FloatValues, 0x0, SIZE_USER_DATA * sizeof(float));
	memset(TmpFloatValues, 0x0, SIZE_USER_DATA * sizeof(float));
	memset(DesiredTorqueValues, 0x0	, NUMBER_OF_JOINTS * sizeof(float));

	fprintf(stdout, "You may need superuser permission to run this program.\n");
	fflush(stdout);
	FastResearchInterface	*FRI;
	
	FRI = new FastResearchInterface("/home/padalkar/FRILibrary/src/FastResearchInterfaceLibrary/etc/980039-FRI-Driver.init");

	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		JointStiffnessValues	[i] =	(float)10.0;
		JointDampingValues		[i]	=	(float)0.7;
	}

	for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
	{
		CartStiffnessValues		[i]	=	(float)10.0;
		CartDampingValues		[i]	=	(float)0.7;
	}
	FRI->SetCommandedCartDamping(CartStiffnessValues);
	FRI->SetCommandedCartStiffness(CartDampingValues);
	FRI->SetCommandedJointDamping(JointDampingValues);
	FRI->SetCommandedJointStiffness(JointStiffnessValues);

	while (Run)
	{
		
		FRI->WaitForKRCTick();

		if (!FRI->IsMachineOK())
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
		
		}
		ControlScheme	=	FastResearchInterface::CART_IMPEDANCE_CONTROL;
		printf("Control strategy set to joint impedance control.\n");
		ResultValue	=	FRI->StartRobot(ControlScheme);
		if (ResultValue != EOK)
		{
			printf("An error occurred during starting up the robot...\n");
		}
		else
		{
			StartRobotCalled	=	true;
		}
		break;
		Run = false;
	}

	while (true)
	{
		FRI->WaitForKRCTick();
		std::cout << std::endl;

		if (!FRI->IsMachineOK() && false)
		{
			fprintf(stderr, "ERROR, the machine is not ready anymore.\n");
			fprintf(stderr, "Exiting\n");
		}
		FRI->GetMeasuredCartPose(MeasuredPose);
		FRI->GetCommandedCartPose(CmndPose);
		
		for (int i = 0; i < 12; i++)
		{
			std::cout << MeasuredPose[i] - CmndPose[i] << "   ";
		}
	}
	


	delete FRI;

	printf("\nGood bye.\n\n");

	return(EXIT_SUCCESS);
}
