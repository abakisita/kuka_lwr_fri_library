//  ---------------------- Doxygen info ----------------------
//! \file KRCCommunicationThreadMain.cpp
//!
//! \brief
//! Implementation file for the thread of the class FastResearchInterface that
//! communicates with the KRC unit
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
#include <sched.h>
#include <string.h>
#include <stdio.h>
#include <UDPSocket.h>
#include <FRICommunication.h>
#include <OSAbstraction.h>


#if defined(WIN32) || defined(WIN64) || defined(_WIN64)// \ToDo Make this clean through the OSAbstraction
#include <Windows.h>	
#endif



// ****************************************************************
// KRCCommunicationThreadMain()
//
void* FastResearchInterface::KRCCommunicationThreadMain(void *ObjectPointer)
{
	int								SequenceCounter					=	0
								,	ResultValue						=	0;

	float							ZeroVector[NUMBER_OF_JOINTS];

	UDPSocket 						KRC;

	FRIDataReceivedFromKRC 			LocalReadData;

	FRIDataSendToKRC				LocalCommandData;

	FastResearchInterface			*ThisObject						=	(FastResearchInterface*)ObjectPointer;

	memset(ZeroVector, 0x0, NUMBER_OF_JOINTS * sizeof(float));
	
#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
	// \ToDo Make this clean through the OSAbstraction
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif	

	pthread_mutex_lock(&(ThisObject->MutexForThreadCreation));
	ThisObject->ThreadCreated	=	true;
	pthread_mutex_unlock(&(ThisObject->MutexForThreadCreation));

	pthread_cond_signal(&(ThisObject->CondVarForThreadCreation));

	for(;;)
	{
		// receive data from the KRC unit
		ResultValue	=	KRC.ReceiveFRIDataFromKRC(&LocalReadData);

		if (ResultValue != 0)
		{
			ThisObject->OutputConsole->printf("FastResearchInterface::KRCCommunicationThreadMain(): ERROR during the reception of a UDP data package.\n");
		}

		if (ThisObject->CurrentControlScheme == FastResearchInterface::JOINT_TORQUE_CONTROL)
		{
			ThisObject->SetCommandedJointDamping(ZeroVector);
			ThisObject->SetCommandedJointStiffness(ZeroVector);
		}

		pthread_mutex_lock(&(ThisObject->MutexForControlData));

		ThisObject->NewDataFromKRCReceived	=	true;
		ThisObject->ReadData				=	LocalReadData;

		if (!(ThisObject->KRCCommunicationThreadIsRunning))
		{
			pthread_mutex_unlock(&(ThisObject->MutexForControlData));
			break;
		}

		LocalCommandData	=	ThisObject->CommandData;

		SequenceCounter++;
		LocalCommandData.Header.FRISequenceCounterForUDPPackages	=	SequenceCounter;
		LocalCommandData.Header.FRIReflectedSequenceCounterForUDPPackages	=	LocalReadData.Header.FRISequenceCounterForUDPPackages;
		LocalCommandData.Header.FRIDatagramID	=	FRI_DATAGRAM_ID_CMD;
		LocalCommandData.Header.FRIPackageSizeInBytes	=	sizeof(FRIDataSendToKRC);

		pthread_mutex_unlock(&(ThisObject->MutexForControlData));

		pthread_cond_broadcast(&(ThisObject->CondVarForDataReceptionFromKRC));

		// send data to KRC unit
		ResultValue						=	KRC.SendFRIDataToKRC(&LocalCommandData);

		if (ResultValue != 0)
		{
			ThisObject->OutputConsole->printf("FastResearchInterface::KRCCommunicationThreadMain(): ERROR during the sending of a UDP data package.\n");
		}

		pthread_mutex_lock(&(ThisObject->MutexForLogging));

		if (ThisObject->LoggingIsActive)
		{
			pthread_mutex_unlock(&(ThisObject->MutexForLogging));
			ThisObject->DataLogger->AddEntry(		LocalReadData
											 	,	LocalCommandData);
		}
		else
		{
			pthread_mutex_unlock(&(ThisObject->MutexForLogging));
		}
	}

	pthread_exit(NULL);
	
	return (NULL);
}


