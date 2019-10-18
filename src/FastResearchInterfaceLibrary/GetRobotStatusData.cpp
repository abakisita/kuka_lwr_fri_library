//  ---------------------- Doxygen info ----------------------
//! \file GetRobotStatusData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! providing robot status data to the user application of this class
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
// GetFRIMode()
//
unsigned int FastResearchInterface::GetFRIMode(void)
{
	unsigned int	ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.InterfaceState.FRIState;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}

// ****************************************************************
// GetCurrentControlScheme()
//
unsigned int FastResearchInterface::GetCurrentControlScheme(void)
{
	unsigned int		ResultValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ResultValue	=	this->ReadData.Robot.FRIRobotControl;
	pthread_mutex_unlock(&(this->MutexForControlData));

	switch (ResultValue)
	{
	case FRI_CONTROL_POSITION :
		return(FastResearchInterface::JOINT_POSITION_CONTROL);
		break;
	case FRI_CONTROL_CART_IMP:
		return(FastResearchInterface::CART_IMPEDANCE_CONTROL);
		break;
	case FRI_CONTROL_JNT_IMP:
		if (this->CurrentControlScheme == FastResearchInterface::JOINT_TORQUE_CONTROL)
		{
			return(FastResearchInterface::JOINT_TORQUE_CONTROL);
		}
		else
		{
			return(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
		}
		break;
	}

	return(0);
}


// ****************************************************************
// IsRobotArmPowerOn()
//
bool FastResearchInterface::IsRobotArmPowerOn(void)
{
	bool		ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	(this->ReadData.Robot.FRIRobotPower != 0);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// DoesAnyDriveSignalAnError()
//
bool FastResearchInterface::DoesAnyDriveSignalAnError(void)
{
	bool		ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	(this->ReadData.Robot.FRIDriveError != 0);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// DoesAnyDriveSignalAWarning()
//
bool FastResearchInterface::DoesAnyDriveSignalAWarning(void)
{
	bool		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	(this->ReadData.Robot.FRIDriveWarning != 0);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetDriveTemperatures()
//
void FastResearchInterface::GetDriveTemperatures(float *Temperatures)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		Temperatures[i]	=	this->ReadData.Robot.FRIDriveTemperature[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetKRLBoolValues()
//
void FastResearchInterface::GetKRLBoolValues(bool *KRLBoolValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < SIZE_USER_DATA; i++)
	{
		KRLBoolValues[i]	=	((this->ReadData.SharedKRLVariables.FRIBoolValuesInKRC & (1 << i)) != 0);
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetKRLIntValues()
//
void FastResearchInterface::GetKRLIntValues(int *KRLIntValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < SIZE_USER_DATA; i++)
	{
		KRLIntValues[i]	=	this->ReadData.SharedKRLVariables.FRIIntegerValuesInKRC[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetKRLFloatValues()
//
void FastResearchInterface::GetKRLFloatValues(float *KRLFloatValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < SIZE_USER_DATA; i++)
	{
		KRLFloatValues[i]	=	this->ReadData.SharedKRLVariables.FRIFloatingPointValuesInKRC[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetKRLBoolValue()
//
bool FastResearchInterface::GetKRLBoolValue(const unsigned int &Index)
{
	bool		ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	((this->ReadData.SharedKRLVariables.FRIBoolValuesInKRC & (1 << Index)) != 0);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetKRLIntValue()
//
int FastResearchInterface::GetKRLIntValue(const unsigned int &Index)
{
	int		ReturnValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.SharedKRLVariables.FRIIntegerValuesInKRC[Index];
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetKRLFloatValue()
//
float FastResearchInterface::GetKRLFloatValue(const unsigned int &Index)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.SharedKRLVariables.FRIFloatingPointValuesInKRC[Index];
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetCurrentJacobianMatrix()
//
void FastResearchInterface::GetCurrentJacobianMatrix(float **JacobianMatrix)
{
	unsigned int		i	=	0
					,	j	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_CART_DOFS; i++)
	{
		for (j = 0; j < NUMBER_OF_JOINTS; j++)
		{
			//JacobianMatrix[i][j]	=	this->ReadData.data.FRIJacobianMatrix[(i==3)?(5):((i==5)?(3):(i))*NUMBER_OF_JOINTS+j];
			JacobianMatrix[i][j]	=	this->ReadData.MeasuredData.FRIJacobianMatrix[i*NUMBER_OF_JOINTS+j];
		}
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCurrentMassMatrix()
//
void FastResearchInterface::GetCurrentMassMatrix(float **MassMatrix)
{
	unsigned int		i	=	0
					,	j	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		for (j = 0; j < NUMBER_OF_JOINTS; j++)
		{
			MassMatrix[i][j]	=	this->ReadData.MeasuredData.FRIMassMatrix[i*NUMBER_OF_JOINTS+j];
		}
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCurrentGravityVector()
//
void FastResearchInterface::GetCurrentGravityVector(float *GravityVector)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		GravityVector[i]	=	this->ReadData.MeasuredData.FRIGravityVectorInJointSpace[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// IsMachineOK()
//
bool FastResearchInterface::IsMachineOK(void)
{
	bool		ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	(	(this->ReadData.InterfaceState.FRIState == FRI_STATE_CMD)
					&&	(this->ReadData.Robot.FRIRobotPower != 0)
					&&	(this->ReadData.Robot.FRIDriveError == 0)				);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


