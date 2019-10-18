//  ---------------------- Doxygen info ----------------------
//! \file StartRobot.cpp
//!
//! \brief
//! Implementation file for the method StartRobot() of the class FastResearchInterface
//!
//! \details
//! The class FastResearchInterface provides a basic low-level interface
//! to the KUKA Light-Weight Robot IV For details, please refer to the file
//! FastResearchInterface.h.
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
#include <FRICommunication.h>
#include <OSAbstraction.h>
#include <errno.h>
#include <string.h>



// ****************************************************************
// StartRobot()
//
int FastResearchInterface::StartRobot(		const unsigned int &ControlMode
									  	,	const float &TimeOutValueInSeconds)
{
	int					ResultValue				=	0;

	float				CommandValues[2 * NUMBER_OF_JOINTS]
					,	InitialSystemTimeValue	=	GetSystemTimeInSeconds(true);

	memset(CommandValues, 0x0, 2 * NUMBER_OF_JOINTS * sizeof(float));

	if (this->GetFRIMode() == FRI_STATE_OFF)
	{
		// wait until the FRI is in monitor mode
		while (		((GetSystemTimeInSeconds() - InitialSystemTimeValue)	<	TimeOutValueInSeconds	)
				&&	(this->GetFRIMode()										!=	FRI_STATE_MON			))
		{
			delay(1);
		}

		if (this->GetFRIMode() == FRI_STATE_OFF)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC has not opened the FRI yet or no UDP connection can be established.\n");
			return(ENOTCONN);
		}
	}

	if (this->GetFRIMode() == FRI_STATE_CMD)
	{
		if (this->CurrentControlScheme	== ControlMode)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): The robot has already been started.\n");
			return(EALREADY);
		}
		else
		{
			this->StopRobot();
		}
	}

	if (this->GetKRLIntValue(15) == 20)
	{
		this->StopRobot();
	}


	this->CurrentControlScheme	=	ControlMode;

	this->SetControlScheme(this->CurrentControlScheme);

	// The KRL program running on the robot controller waits for a
	// integer value change at position 15 (i.e., 16 in KRL).
	// A value of 10 lets the KRL program call friStart()
	this->SetKRLIntValue(15, 10);

	// wait for the next data telegram of the KRC unit
	pthread_mutex_lock(&(this->MutexForControlData));
	this->NewDataFromKRCReceived	=	false;
	pthread_mutex_unlock(&(this->MutexForControlData));
	ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

	while (		((GetSystemTimeInSeconds() - InitialSystemTimeValue)	< TimeOutValueInSeconds	)
			&&	(this->GetFRIMode()										!= FRI_STATE_CMD		)	)
	{
		ResultValue	=	this->WaitForKRCTick((unsigned int)(this->CycleTime * 3000000.0));

		if (ResultValue != EOK)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
			return(ENOTCONN);
		}
	}

	if (this->GetFRIMode() == FRI_STATE_CMD)
	{
		while (		((GetSystemTimeInSeconds() - InitialSystemTimeValue) < TimeOutValueInSeconds)
				&&	(!(this->IsMachineOK()))
				&&	(this->GetKRLIntValue(15) != 20)	)
		{
			ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

			if (ResultValue != EOK)
			{
				this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
				return(ENOTCONN);
			}
		}

		if (this->IsMachineOK())
		{
			delay(12);	// KUKA interpolation cycle time
			return(EOK);
		}
		else
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, command mode reached, but the robot could be turned on correctly. Check KRC state.");
			return(ETIME);
		}
	}
	else
	{
		this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, could not switch to command mode after timeout.\n");
		return(ETIME);
	}
}

