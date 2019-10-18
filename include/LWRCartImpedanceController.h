//  ---------------------- Doxygen info ----------------------
//! \file LWRCartImpedanceController.h
//!
//! \brief
//! Header file for the class LWRCartImpedanceController
//!
//! \details
//! The class LWRCartImpedanceController constitutes an access
//! easy-to-use Cartesian impedance control interface through the KUKA
//! Fast Research Interface of the Light-Weight Robot IV.
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


#ifndef __LWRCartImpedanceController__
#define __LWRCartImpedanceController__


#include <FastResearchInterface.h>
#include <LWRBaseControllerInterface.h>


//  ---------------------- Doxygen info ----------------------
//! \class LWRCartImpedanceController
//!
//! \brief
//! Cartesian impedance controller interface for the KUKA Light-Weight
//! Robot IV
//  ----------------------------------------------------------

class LWRCartImpedanceController : public LWRBaseControllerInterface
{


public:


//  ---------------------- Doxygen info ----------------------
//! \fn LWRCartImpedanceController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
//!
//! \brief
//! Constructor
//!
//! \copydetails LWRBaseControllerInterface::LWRBaseControllerInterface()
//  ----------------------------------------------------------
	LWRCartImpedanceController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~LWRCartImpedanceController(void)
//!
//! \brief
//! Destructor
//!
//! \copydetails LWRBaseControllerInterface::~LWRBaseControllerInterface()
//  ----------------------------------------------------------
	~LWRCartImpedanceController(void)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int StartRobot(const float &TimeOutValueInSeconds	=	120.0)
//!
//! \brief
//! \copybrief FastResearchInterface::StartRobot()
//!
//! \details
//! \copydetails FastResearchInterface::StartRobot()
//  ----------------------------------------------------------
	inline int StartRobot(const float &TimeOutValueInSeconds	=	120.0)
	{
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(		FastResearchInterface::CART_IMPEDANCE_CONTROL
									 	,	TimeOutValueInSeconds));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMeasuredCartPose(float *MeasuredCartPose)
//!
//! \brief
//! \copybrief FastResearchInterface::GetMeasuredCartPose()
//!
//! \details
//! \copydetails FastResearchInterface::GetMeasuredCartPose()
//  ----------------------------------------------------------
	inline void GetMeasuredCartPose(float *MeasuredCartPose)
	{
		return(this->FRI->GetMeasuredCartPose(MeasuredCartPose));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedCartPose(const float *CommandedCartPose)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedCartPose()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedCartPose()
//  ----------------------------------------------------------
	inline void SetCommandedCartPose(const float *CommandedCartPose)
	{
		return(this->FRI->SetCommandedCartPose(CommandedCartPose));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedCartForcesAndTorques()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedCartForcesAndTorques()
//  ----------------------------------------------------------
	inline void SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques)
	{
		return(this->FRI->SetCommandedCartForcesAndTorques(CartForcesAndTorques));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedCartStiffness(const float *CommandedCartStiffness)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedCartStiffness()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedCartStiffness()
//  ----------------------------------------------------------
	inline void SetCommandedCartStiffness(const float *CommandedCartStiffness)
	{
		return(this->FRI->SetCommandedCartStiffness(CommandedCartStiffness));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedCartDamping(const float *CommandedCartDamping)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedCartDamping()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedCartDamping()
//  ----------------------------------------------------------
	inline void SetCommandedCartDamping(const float *CommandedCartDamping)
	{
		return(this->FRI->SetCommandedCartDamping(CommandedCartDamping));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques)
//!
//! \brief
//! \copybrief FastResearchInterface::GetEstimatedExternalCartForcesAndTorques()
//!
//! \details
//! \copydetails FastResearchInterface::GetEstimatedExternalCartForcesAndTorques()
//  ----------------------------------------------------------
	inline void GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques)
	{
		return(this->FRI->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesAndTorques));
	}

};	// class LWRCartImpedanceController

#endif
