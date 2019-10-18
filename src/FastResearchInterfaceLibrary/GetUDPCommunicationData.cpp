//  ---------------------- Doxygen info ----------------------
//! \file GetUDPCommunicationData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! providing data about the UDP communication channel
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
#include <FRICommunication.h>


// ****************************************************************
// GetFRICycleTime()
//
float FastResearchInterface::GetFRICycleTime(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.InterfaceState.FRISampleTimePeriodForDataSentToKRC;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetCommunicationTimingQuality()
//
int FastResearchInterface::GetCommunicationTimingQuality(void)
{
	int		ReturnValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.InterfaceState.FRIQuality;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetUDPAverageRateOfAnsweredPackages()
//
float FastResearchInterface::GetUDPAverageRateOfAnsweredPackages(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.InterfaceState.FRIStatistics.AverageRateOfAnsweredPackages;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetUDPAverageLatencyInSeconds()
//
float FastResearchInterface::GetUDPAverageLatencyInSeconds(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.InterfaceState.FRIStatistics.AverageLatencyInSeconds;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetUDPAverageJitterInSeconds()
//
float FastResearchInterface::GetUDPAverageJitterInSeconds(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.InterfaceState.FRIStatistics.AverageJitterInSeconds;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetUDPPackageLossRate()
//
float FastResearchInterface::GetUDPPackageLossRate(void)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.InterfaceState.FRIStatistics.AverageRateOfMissedPackages;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetNumberOfMissedUDPPackages()
//
unsigned int FastResearchInterface::GetNumberOfMissedUDPPackages(void)
{
	unsigned int		ReturnValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.InterfaceState.FRIStatistics.AbsoluteNumberOfMissedPackages;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetValueOfKRCSequenceCounter()
//
unsigned int FastResearchInterface::GetValueOfKRCSequenceCounter(void)
{
	unsigned int		ReturnValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.Header.FRISequenceCounterForUDPPackages;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}
