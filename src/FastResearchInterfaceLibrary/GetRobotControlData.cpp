//  ---------------------- Doxygen info ----------------------
//! \file GetRobotControlData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! providing robot control data to the user application of this class
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
// GetMeasuredJointPositions()
//
void FastResearchInterface::GetMeasuredJointPositions(float *MeasuredJointPositions)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		MeasuredJointPositions[i]	=	this->ReadData.MeasuredData.FRIMeasuredJointPositionVectorInRad[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCommandedJointPositions()
//
void FastResearchInterface::GetCommandedJointPositions(float *CommandedJointPositions)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		CommandedJointPositions[i]	=	this->ReadData.MeasuredData.FRICommandedJointPostionVectorFromKRC[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCommandedJointPositionOffsets()
//
void FastResearchInterface::GetCommandedJointPositionOffsets(float *CommandedJointPositionOffsets)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		CommandedJointPositionOffsets[i]	=	this->ReadData.MeasuredData.FRICommandedJointPostionOffsetVectorFromKRC[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetMeasuredJointTorques()
//
void FastResearchInterface::GetMeasuredJointTorques(float *MeasuredJointTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		MeasuredJointTorques[i]	=	this->ReadData.MeasuredData.FRIMeasuredJointTorqueVectorInNm[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetEstimatedExternalJointTorques()
//
void FastResearchInterface::GetEstimatedExternalJointTorques(float *EstimatedExternalJointTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		EstimatedExternalJointTorques[i]	=	this->ReadData.MeasuredData.FRIEstimatedExternalJointTorqueVectorInNm[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetMeasuredCartPose()
//
void FastResearchInterface::GetMeasuredCartPose(float *MeasuredCartPose)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
	{
		MeasuredCartPose[i]	=	this->ReadData.MeasuredData.FRIMeasuredCartesianFrame[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCommandedCartPose()
//
void FastResearchInterface::GetCommandedCartPose(float *CommandedCartPose)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
	{
		CommandedCartPose[i]	=	this->ReadData.MeasuredData.FRICommandedCartesianFrameFromKRC[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCommandedCartPoseOffsets()
//
void FastResearchInterface::GetCommandedCartPoseOffsets(float *CommandedCartPoseOffsets)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
	{
		CommandedCartPoseOffsets[i]	=	this->ReadData.MeasuredData.FRICommandedCartesianFrameOffsetFromKRC[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetEstimatedExternalCartForcesAndTorques()
//
void FastResearchInterface::GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
	{
		EstimatedExternalCartForcesAndTorques[i]	=	this->ReadData.MeasuredData.FRIEstimatedCartesianForcesAndTorques[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}

