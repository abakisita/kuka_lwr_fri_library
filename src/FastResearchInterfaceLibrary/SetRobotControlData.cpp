//  ---------------------- Doxygen info ----------------------
//! \file SetRobotControlData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! setting robot control data by the user application of this class
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
// SetCommandedJointPositions()
//
void FastResearchInterface::SetCommandedJointPositions(const float *CommandedJointPositions)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		this->CommandData.CommandValues.FRICommandedJointPositionVectorInRad[i]	=	CommandedJointPositions[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedJointTorques()
//
void FastResearchInterface::SetCommandedJointTorques(const float *CommandedJointTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		this->CommandData.CommandValues.FRICommandedAdditionalJointTorqueVectorInNm[i]	=	CommandedJointTorques[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedJointStiffness()
//
void FastResearchInterface::SetCommandedJointStiffness(const float *CommandedJointStiffness)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		this->CommandData.CommandValues.FRICommandedJointStiffnessVectorInNmPerRad[i]	=	CommandedJointStiffness[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedJointDamping()
//
void FastResearchInterface::SetCommandedJointDamping(const float *CommandedJointDamping)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		this->CommandData.CommandValues.FRICommandedNormalizedJointDampingVector[i]	=	CommandedJointDamping[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedCartPose()
//
void FastResearchInterface::SetCommandedCartPose(const float *CommandedCartPose)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
	{
		this->CommandData.CommandValues.FRICommandedCartesianFrame[i]	=	CommandedCartPose[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedCartForcesAndTorques()
//
void FastResearchInterface::SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
	{
		this->CommandData.CommandValues.FRICommandedAdditionalCartesianForceTorqueVector[i]	=	CartForcesAndTorques[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedCartStiffness()
//
void FastResearchInterface::SetCommandedCartStiffness(const float *CommandedCartStiffness)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
	{
		this->CommandData.CommandValues.FRICommandedCartesianStiffnessVector[i]	=	CommandedCartStiffness[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedCartDamping()
//
void FastResearchInterface::SetCommandedCartDamping(const float *CommandedCartDamping)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
	{
		this->CommandData.CommandValues.FRICommandedNormalizedCartesianDampingVector[i]	=	CommandedCartDamping[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}
