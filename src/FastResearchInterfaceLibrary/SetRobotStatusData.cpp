//  ---------------------- Doxygen info ----------------------
//! \file SetRobotStatusData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! setting robot status data by the user application of this class
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
// SetKRLBoolValues()
//
void FastResearchInterface::SetKRLBoolValues(const bool *KRLBoolValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < SIZE_USER_DATA; i++)
	{
		if (KRLBoolValues[i])
		{
			this->CommandData.SharedKRLVariables.FRIBoolValuesInKRC |= (1 << i);
		}
		else
		{
			this->CommandData.SharedKRLVariables.FRIBoolValuesInKRC &= ( ~( 1 << i) );
		}
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetKRLIntValues()
//
void FastResearchInterface::SetKRLIntValues(const int *KRLIntValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < SIZE_USER_DATA; i++)
	{
		this->CommandData.SharedKRLVariables.FRIIntegerValuesInKRC[i]	=	KRLIntValues[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetKRLFloatValues()
//
void FastResearchInterface::SetKRLFloatValues(const float *KRLFloatValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < SIZE_USER_DATA; i++)
	{
		this->CommandData.SharedKRLVariables.FRIFloatingPointValuesInKRC[i]	=	KRLFloatValues[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetKRLBoolValue()
//
void FastResearchInterface::SetKRLBoolValue(	const unsigned int	&Index
											,	const bool			&Value	)
{
	pthread_mutex_lock(&(this->MutexForControlData));
	if (Value)
	{
		this->CommandData.SharedKRLVariables.FRIBoolValuesInKRC |= (1 << Index);
	}
	else
	{
		this->CommandData.SharedKRLVariables.FRIBoolValuesInKRC &= ( ~( 1 << Index) );
	}
	pthread_mutex_unlock(&(this->MutexForControlData));
	return;
}


// ****************************************************************
// SetKRLIntValue()
//
void FastResearchInterface::SetKRLIntValue(		const unsigned int	&Index
										   	,	const int			&Value	)
{
	pthread_mutex_lock(&(this->MutexForControlData));
	this->CommandData.SharedKRLVariables.FRIIntegerValuesInKRC[Index]	=	Value;
	pthread_mutex_unlock(&(this->MutexForControlData));
	return;
}


// ****************************************************************
// SetKRLFloatValue()
//
void FastResearchInterface::SetKRLFloatValue(	const unsigned int	&Index
											 ,	const float			&Value	)
{
	pthread_mutex_lock(&(this->MutexForControlData));
	this->CommandData.SharedKRLVariables.FRIFloatingPointValuesInKRC[Index]	=	Value;
	pthread_mutex_unlock(&(this->MutexForControlData));
	return;
}

