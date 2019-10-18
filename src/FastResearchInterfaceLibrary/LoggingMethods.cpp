//  ---------------------- Doxygen info ----------------------
//! \file LoggingMethods.cpp
//!
//! \brief
//! Implementation file for the four logging methods of the class FastResearchInterface
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
#include <DataLogging.h>
#include <errno.h>
#include <OSAbstraction.h>


// ****************************************************************
// PrepareLogging()
//
int FastResearchInterface::PrepareLogging(const char *FileIdentifier)
{
	if (this->LoggingState != FastResearchInterface::WriteLoggingDataFileCalled)
	{
		return(EINVAL);
	}
	else
	{
		this->LoggingState = FastResearchInterface::PrepareLoggingCalled;
	}

	return(this->DataLogger->PrepareLogging(this->CurrentControlScheme, FileIdentifier));
}


// ****************************************************************
// StartLogging()
//
int FastResearchInterface::StartLogging(void)
{
	if (this->LoggingState != FastResearchInterface::PrepareLoggingCalled)
	{
		return(EINVAL);
	}
	else
	{
		this->LoggingState = FastResearchInterface::StartLoggingCalled;
	}

	pthread_mutex_lock(&(this->MutexForLogging));
	this->LoggingIsActive	=	true;
	pthread_mutex_unlock(&(this->MutexForLogging));

	return(EOK);
}


// ****************************************************************
// StopLogging()
//
int FastResearchInterface::StopLogging(void)
{
	if (this->LoggingState != FastResearchInterface::StartLoggingCalled)
	{
		return(EINVAL);
	}
	else
	{
		this->LoggingState = FastResearchInterface::StopLoggingCalled;
	}

	pthread_mutex_lock(&(this->MutexForLogging));
	this->LoggingIsActive	=	false;
	pthread_mutex_unlock(&(this->MutexForLogging));

	return(EOK);
}


// ****************************************************************
// WriteLoggingDataFile()
//
int FastResearchInterface::WriteLoggingDataFile(void)
{
	if (this->LoggingState == FastResearchInterface::WriteLoggingDataFileCalled)
	{
		return(EINVAL);
	}

	this->LoggingState = FastResearchInterface::WriteLoggingDataFileCalled;

	if (this->LoggingState == FastResearchInterface::StartLoggingCalled)
	{
		pthread_mutex_lock(&(this->MutexForLogging));
		this->LoggingIsActive	=	false;
		pthread_mutex_unlock(&(this->MutexForLogging));
	}

	return(this->DataLogger->WriteToFile());
}

