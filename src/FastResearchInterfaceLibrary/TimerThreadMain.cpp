//  ---------------------- Doxygen info ----------------------
//! \file TimerThreadMain.cpp
//!
//! \brief
//! Implementation file for the timer thread of the class FastResearchInterface
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
#include <stdio.h>
#include <OSAbstraction.h>


#ifdef _NTO_



#include <sys/neutrino.h>
#include <sched.h>


#define	TIMER_PULSE					(_PULSE_CODE_MINAVAIL)



// ****************************************************************
// TimerThreadMain()
//
void* FastResearchInterface::TimerThreadMain(void *ObjectPointer)
{
	int								OurChannelID	=	0
								,	ReceptionID		=	0;

	timer_t 						TimerID;

	struct sigevent 				Event;

	struct itimerspec 				Timer;

	struct _pulse  					PulseMsg;

	FastResearchInterface			*ThisObject		=	(FastResearchInterface*)ObjectPointer;

	OurChannelID				=	ChannelCreate(0); //create communication channel

	// Initialize event data structure
	// attach the timer to the channel OurChannelID
	Event.sigev_notify			=	SIGEV_PULSE;
	Event.sigev_coid			=	ConnectAttach(0, 0, OurChannelID, _NTO_SIDE_CHANNEL, 0);
	Event.sigev_priority		=	getprio(0);
	Event.sigev_code			=	TIMER_PULSE;

	timer_create(CLOCK_REALTIME, &Event, &TimerID);

	// Configure the timer
	Timer.it_value.tv_sec		=	0L;
	Timer.it_value.tv_nsec		=	(long int)(1000000000.0 * ThisObject->CycleTime);	// wait one cycle time interval before start
	Timer.it_interval.tv_sec	=	0L;
	Timer.it_interval.tv_nsec	=	(long int)(1000000000.0 * ThisObject->CycleTime);

	pthread_mutex_lock(&(ThisObject->MutexForThreadCreation));
	ThisObject->ThreadCreated	=	true;
	pthread_mutex_unlock(&(ThisObject->MutexForThreadCreation));

	pthread_cond_signal(&(ThisObject->CondVarForThreadCreation));

	// Start the timer
	timer_settime(TimerID, 0, &Timer, NULL);

	pthread_mutex_lock(&(ThisObject->MutexForCondVarForTimer));

	while(ThisObject->TimerThreadIsRunning)
	{
		pthread_mutex_unlock(&(ThisObject->MutexForCondVarForTimer));

		ReceptionID	= MsgReceive(OurChannelID, &PulseMsg, sizeof(PulseMsg), NULL);

		pthread_mutex_lock(&(ThisObject->MutexForCondVarForTimer));
		if (ReceptionID == 0)
		{
			ThisObject->TimerFlag = true;
			pthread_cond_signal(&(ThisObject->CondVarForTimer));
		}
	}
	pthread_mutex_unlock(&(ThisObject->MutexForCondVarForTimer));

	if (timer_delete(TimerID) != 0)
	{
		ThisObject->OutputConsole->printf("FastResearchInterface::TimerThreadMain(): ERROR, cannot delete timer...\n");
	}

	pthread_exit(NULL);
}


#endif // _NTO_


#if defined(WIN32) || defined(WIN64) || defined(_WIN64)



// ****************************************************************
// TimerThreadMain()
//
void* FastResearchInterface::TimerThreadMain(void *ObjectPointer)
{
	int								OurChannelID	=	0
								,	ReceptionID		=	0;

	FastResearchInterface			*ThisObject		=	(FastResearchInterface*)ObjectPointer;

	pthread_mutex_lock(&(ThisObject->MutexForThreadCreation));
	ThisObject->ThreadCreated	=	true;
	pthread_mutex_unlock(&(ThisObject->MutexForThreadCreation));

	pthread_cond_signal(&(ThisObject->CondVarForThreadCreation));

	pthread_mutex_lock(&(ThisObject->MutexForCondVarForTimer));

	while(ThisObject->TimerThreadIsRunning)
	{
		pthread_mutex_unlock(&(ThisObject->MutexForCondVarForTimer));

		delay(1);

		pthread_mutex_lock(&(ThisObject->MutexForCondVarForTimer));
		if (ReceptionID == 0)
		{
			ThisObject->TimerFlag = true;
			pthread_cond_signal(&(ThisObject->CondVarForTimer));
		}
	}
	pthread_mutex_unlock(&(ThisObject->MutexForCondVarForTimer));

	pthread_exit(NULL);
	
	return (NULL);
}


#endif // WIN32