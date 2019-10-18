//  ---------------------- Doxygen info ----------------------
//! \file StopRobot.cpp
//!
//! \brief
//! Implementation file for the method StopRobot of the class FastResearchInterface
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
#include <errno.h>
#include <OSAbstraction.h>

#define TIMEOUT_VALUE_IN_SECONDS_TO_REACH_MONITOR_MODE	10.0


// ****************************************************************
// StartRobot()
//
int FastResearchInterface::StopRobot(void)
{
	unsigned int		DataTelegramCounter		=	0;

	int					ResultValue				=	0;

	// The KRL program running on the robot controller waits for a
	// integer value change at position 15 (i.e., 16 in KRL).
	// A value of 20 lets the KRL program call friStop()
	this->SetKRLIntValue(15, 20);

	// wait for the next data telegram of the KRC unit
	pthread_mutex_lock(&(this->MutexForControlData));
	this->NewDataFromKRCReceived	=	false;
	pthread_mutex_unlock(&(this->MutexForControlData));
	ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

	if (ResultValue != EOK)
	{
		this->OutputConsole->printf("FastResearchInterface::StopRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
		return(ENOTCONN);
	}

	// wait until we are in monitor mode again
	while (		((double)DataTelegramCounter * this->CycleTime	<	TIMEOUT_VALUE_IN_SECONDS_TO_REACH_MONITOR_MODE)
			&&	(this->GetFRIMode()								!=	FRI_STATE_MON		)	)
	{
		ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

		if (ResultValue != EOK)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
			return(ENOTCONN);
		}

		DataTelegramCounter++;
	}

	// wait until the KRC unit ready for the reception of the next command
	while (		((double)DataTelegramCounter * this->CycleTime	<	TIMEOUT_VALUE_IN_SECONDS_TO_REACH_MONITOR_MODE)
			&&	(this->GetKRLIntValue(15) != 10)	)
	{
		ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

		if (ResultValue != EOK)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
			return(ENOTCONN);
		}

		DataTelegramCounter++;
	}

	if ((double)DataTelegramCounter * this->CycleTime	<	TIMEOUT_VALUE_IN_SECONDS_TO_REACH_MONITOR_MODE)
	{
		delay(12);	// KUKA interpolation cycle time
		return(EOK);
	}
	else
	{
		return(ETIME);
	}
}

