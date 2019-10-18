//  ---------------------- Doxygen info ----------------------
//! \file WaitForTicks.cpp
//!
//! \brief
//! Implementation file for the simple timer methods of the class FastResearchInterface
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
#include <pthread.h>
#include <errno.h>
#include <OSAbstraction.h>




// ****************************************************************
// WaitForKRCTick()
//
int FastResearchInterface::WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds)
{
	int		ReturnValue		=	EOK;

#ifdef _NTO_

	struct timespec			TimeoutValue
						,	CurrentTime;

	if (TimeoutValueInMicroSeconds > 0)
	{
		clock_gettime(CLOCK_REALTIME, &CurrentTime);

		TimeoutValue.tv_nsec	=	(CurrentTime.tv_nsec + (TimeoutValueInMicroSeconds * 1000)) % 1000000000;
		TimeoutValue.tv_sec		=	CurrentTime.tv_sec + (TimeoutValueInMicroSeconds % 1000000)
										+ (CurrentTime.tv_nsec + (TimeoutValueInMicroSeconds * 1000)) / 1000000000;

		pthread_mutex_lock(&(this->MutexForControlData));
		while (!this->NewDataFromKRCReceived)
		{

			ReturnValue	=	 pthread_cond_timedwait(	&(this->CondVarForDataReceptionFromKRC)
					   	 							,	&(this->MutexForControlData)
					   	 							,	&TimeoutValue							);
		}
		this->NewDataFromKRCReceived	=	false;
		pthread_mutex_unlock(&(this->MutexForControlData));

		return(ReturnValue);
	}

#endif

	pthread_mutex_lock(&(this->MutexForControlData));
	while (!this->NewDataFromKRCReceived)
	{

		ReturnValue	=	 pthread_cond_wait(		&(this->CondVarForDataReceptionFromKRC)
				   	 					   	,	&(this->MutexForControlData)			);
	}
	this->NewDataFromKRCReceived	=	false;
	pthread_mutex_unlock(&(this->MutexForControlData));


	return(ReturnValue);
}



// ****************************************************************
// WaitForTimerTick()
//
int FastResearchInterface::WaitForTimerTick(const unsigned int &TimeoutValueInMicroSeconds)
{
	int		ReturnValue		=	0;

#ifdef _NTO_

	struct timespec			TimeoutValue
						,	CurrentTime;

	if (TimeoutValueInMicroSeconds > 0)
	{
		clock_gettime(CLOCK_REALTIME, &CurrentTime);

		TimeoutValue.tv_nsec	=	(CurrentTime.tv_nsec + (TimeoutValueInMicroSeconds * 1000)) % 1000000000;
		TimeoutValue.tv_sec		=	CurrentTime.tv_sec + (TimeoutValueInMicroSeconds % 1000000)
										+ (CurrentTime.tv_nsec + (TimeoutValueInMicroSeconds * 1000)) / 1000000000;

		pthread_mutex_lock(&(this->MutexForCondVarForTimer));
		while (!this->TimerFlag)
		{

			ReturnValue	=	 pthread_cond_timedwait(	&(this->CondVarForTimer)
					   	 							,	&(this->MutexForCondVarForTimer)
					   	 							,	&TimeoutValue							);
		}
		this->TimerFlag	=	false;
		pthread_mutex_unlock(&(this->MutexForCondVarForTimer));

		return(ReturnValue);
	}

#endif

	pthread_mutex_lock(&(this->MutexForCondVarForTimer));
	while (!this->TimerFlag)
	{

		ReturnValue	=	 pthread_cond_wait(		&(this->CondVarForTimer)
				   	 					   	,	&(this->MutexForCondVarForTimer)			);
	}

	this->TimerFlag	=	false;
	pthread_mutex_unlock(&(this->MutexForCondVarForTimer));

	return(ReturnValue);
}
