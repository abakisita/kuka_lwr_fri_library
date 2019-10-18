//  ---------------------- Doxygen info ----------------------
//! \file SetControlScheme.cpp
//!
//! \brief
//! Implementation file for the method SetControlScheme() of the class FastResearchInterface
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
#include <string.h>
#include <FRICommunication.h>
#include <OSAbstraction.h>


// ****************************************************************
// SetControlScheme()
//
int FastResearchInterface::SetControlScheme(const unsigned int &ControlScheme)
{
	unsigned int		i				=	0;

	int					ResultValue		=	0;

	float				FloatValues[2 * NUMBER_OF_FRAME_ELEMENTS];

	memset(FloatValues, 0x0, 2 * NUMBER_OF_JOINTS * sizeof(float));

	if (this->GetFRIMode() == FRI_STATE_MON)
	{
		switch (ControlScheme)
		{
		case FastResearchInterface::JOINT_POSITION_CONTROL:
			pthread_mutex_lock(&(this->MutexForControlData));
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	=	0;
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	|=	MASK_CMD_JNTPOS;
			pthread_mutex_unlock(&(this->MutexForControlData));

			// let the KRL program start the joint position controller
			this->SetKRLIntValue(14, 10);
			break;
		case FastResearchInterface::CART_IMPEDANCE_CONTROL:
			pthread_mutex_lock(&(this->MutexForControlData));
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	=	0;
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	|=	MASK_CMD_CARTPOS;
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	|=	MASK_CMD_TCPFT;
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	|=	MASK_CMD_CARTSTIFF;
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	|=	MASK_CMD_CARTDAMP;
			pthread_mutex_unlock(&(this->MutexForControlData));

			// let the KRL program start the Cartesian impedance controller
			this->SetKRLIntValue(14, 20);
			break;
		case FastResearchInterface::JOINT_TORQUE_CONTROL:
		case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
			pthread_mutex_lock(&(this->MutexForControlData));
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	=	0;
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	|=	MASK_CMD_JNTPOS;
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	|=	MASK_CMD_JNTTRQ;
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	|=	MASK_CMD_JNTSTIFF;
			this->CommandData.CommandValues.FRIRobotCommandDataFlags	|=	MASK_CMD_JNTDAMP;
			pthread_mutex_unlock(&(this->MutexForControlData));

			// let the KRL program start the joint impedance controller
			this->SetKRLIntValue(14, 30);
			break;
		default:
			return(EINVAL);
		}

		if (ControlScheme == FastResearchInterface::CART_IMPEDANCE_CONTROL)
		{
			this->SetKRLIntValue(13, 0);
			this->SetCommandedCartForcesAndTorques(FloatValues);
			for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
			{
				FloatValues[i]	=	(float)0.7;
			}
			this->SetCommandedCartDamping(FloatValues);
			for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
			{
				FloatValues[i]	=	(i < 3)?(1000.0):(100.0);
			}
			this->SetCommandedCartStiffness(FloatValues);

			this->GetCommandedCartPose(&(FloatValues[0]));
			this->GetCommandedCartPoseOffsets(&(FloatValues[NUMBER_OF_FRAME_ELEMENTS]));

			// Regarding the documentation, we should do this
			/* -------------------------------------------------------------
			for (i = 0; i < NUMBER_OF_FRAME_ELEMENTS; i++)
			{
				FloatValues[i]	+=	FloatValues[i + NUMBER_OF_FRAME_ELEMENTS];
			}
			//------------------------------------------------------------- */

			this->SetCommandedCartPose(FloatValues);
		}
		else
		{
			this->SetCommandedJointTorques(FloatValues);
			if (ControlScheme == FastResearchInterface::JOINT_TORQUE_CONTROL)
			{
				// setting this value to one turns off the dynamic model
				// after friStart() has been called
				this->SetKRLIntValue(13, 1);
				this->SetCommandedJointDamping(FloatValues);
				this->SetCommandedJointStiffness(FloatValues);
			}
			else
			{
				this->SetKRLIntValue(13, 0);
			}
			this->GetMeasuredJointPositions(&(FloatValues[0]));
			this->GetCommandedJointPositionOffsets(&(FloatValues[NUMBER_OF_JOINTS]));

			// Regarding the documentation, we should do this
			/* -------------------------------------------------------------
			for (i = 0; i < NUMBER_OF_JOINTS; i++)
			{
				FloatValues[i] +=	FloatValues[i + NUMBER_OF_JOINTS];
			}
			------------------------------------------------------------- */

			this->SetCommandedJointPositions(FloatValues);
		}

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

		return(EOK);
	}
	else
	{
		if (this->GetFRIMode() == FRI_STATE_CMD)
		{
			return(EBUSY);
		}
		else
		{
			return(ENOTCONN);
		}
	}
}
