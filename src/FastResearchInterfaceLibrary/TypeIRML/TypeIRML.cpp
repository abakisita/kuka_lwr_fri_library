//  ---------------------- Doxygen info ----------------------
//! \file TypeIRML.cpp
//!
//! \brief
//! Implementation file for the class of the Type I On-Line Trajectory Generator
//!
//! \sa TypeIRML.h
//!
//! \date December 2014
//!
//! \version 1.2
//!
//!	\author Torsten Kroeger, info@reflexxes.com\n
//! \n
//! Reflexxes GmbH\n
//! Sandknoell 7\n
//! D-24805 Hamdorf\n
//! GERMANY\n
//! \n
//! http://www.reflexxes.com\n
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

#include <TypeIRML.h>
#include <TypeIRMLPolynomial.h>
#include <TypeIRMLMath.h>
#include <TypeIRMLVector.h>
#include <TypeIRMLInputParameters.h>
#include <TypeIRMLOutputParameters.h>
#include <TypeIRMLDecision.h>
#include <TypeIRMLProfiles.h>
#include <string.h>
#include <math.h>

using namespace TypeIRMLMath;

//************************************************************************************
// Constructor


TypeIRML::TypeIRML(const unsigned int &NoOfDOFs, const double &CycleTimeInSeconds)
{
	this->CycleTime									=	CycleTimeInSeconds;
	this->NumberOfDOFs								=	NoOfDOFs;
	this->TrajectoryExecutionTimeForTheUser			=	-1.0;
	this->InternalClockInSeconds					=	0.0;

	this->CurrentInputParameters					=	new TypeIRMLInputParameters	(NumberOfDOFs);
	this->OutputParameters							=	new TypeIRMLOutputParameters(NumberOfDOFs);

	this->Polynomials								=	new TypeIMotionPolynomials	[NumberOfDOFs];
}


//************************************************************************************
// Destructor

TypeIRML::~TypeIRML()
{
	delete(this->CurrentInputParameters		);
	delete(this->OutputParameters			);
	delete[](this->Polynomials				);
}


//************************************************************************************
// GetNextMotionState_Position

int TypeIRML::GetNextMotionState_Position(		const double*	CurrentPosition
											,	const double*	CurrentVelocity
											,	const double*	MaxVelocity
											,	const double*	MaxAcceleration
											,	const double*	TargetPosition
											,	const bool*		SelectionVector
											,	double*			NewPosition
											,	double*			NewVelocity	)
{
	int				i							=	0
				,	ReturnValue					=	TypeIRML::RML_ERROR;

	double			MinimumSynchronizationTime	=	0.0;

	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		(*this->CurrentInputParameters->SelectionVector)	[i]	=	SelectionVector	[i];
		if (SelectionVector[i])
		{
			if (MaxVelocity[i] < RML_MIN_VALUE_FOR_MAXVELOCITY)
			{
				return (TypeIRML::RML_MAX_VELOCITY_ERROR);
			}
			if (MaxAcceleration[i] < RML_MIN_VALUE_FOR_MAXACCELERATION)
			{
				return (TypeIRML::RML_MAX_ACCELERATION_ERROR);
			}

			(*this->CurrentInputParameters->CurrentPosition	)	[i]	=	CurrentPosition	[i];
			(*this->CurrentInputParameters->CurrentVelocity	)	[i]	=	CurrentVelocity	[i];
			(*this->CurrentInputParameters->MaxVelocity		)	[i]	=	MaxVelocity		[i];
			(*this->CurrentInputParameters->MaxAcceleration	)	[i]	=	MaxAcceleration	[i];
			(*this->CurrentInputParameters->TargetPosition	)	[i]	=	TargetPosition	[i];
		}
	}

	// *******************************************************************************
	// * Step 1: Calculate the minimum possile synchronization time
	// *******************************************************************************

	MinimumSynchronizationTime = TypeIRML::CalculateMinimumSynchronizationTime(*(this->CurrentInputParameters));

	// *******************************************************************************
	// * Step 2: Synchronize all selected degrees of freedom
	// *******************************************************************************

	TypeIRML::SynchronizeTrajectory(		*(this->CurrentInputParameters)
										,	MinimumSynchronizationTime
										,	this->Polynomials);

	// *******************************************************************************
	// * Step 3: Calculate output values
	// *******************************************************************************

	ReturnValue = TypeIRML::CalculateOutputValues(		this->Polynomials
													,	*(this->CurrentInputParameters->SelectionVector)
													,	OutputParameters);

	TrajectoryExecutionTimeForTheUser = MinimumSynchronizationTime;

	if (ReturnValue != TypeIRML::RML_ERROR)
	{
		for (i = 0; i < this->NumberOfDOFs; i++)
		{
			if (SelectionVector[i])
			{
				NewPosition[i] = (*OutputParameters->NewPosition)[i];
				NewVelocity[i] = (*OutputParameters->NewVelocity)[i];
			}
		}
	}

	return (ReturnValue);
}


//************************************************************************************
// GetNextMotionState_Position

int TypeIRML::GetNextMotionState_Position(		const TypeIRMLInputParameters	&IP
											,	TypeIRMLOutputParameters		*OP	)
{
	return(GetNextMotionState_Position(		IP.CurrentPosition->VecData
										,	IP.CurrentVelocity->VecData
										,	IP.MaxVelocity->VecData
										,	IP.MaxAcceleration->VecData
										,	IP.TargetPosition->VecData
										,	IP.SelectionVector->VecData
										,	OP->NewPosition->VecData
										,	OP->NewVelocity->VecData		));
}


//************************************************************************************
// CalculateMinimumSynchronizationTime()

double TypeIRML::CalculateMinimumSynchronizationTime(const TypeIRMLInputParameters &IP) const
{
	int				i							=	0;

	double			CurrentPosition				=	0.0
				,	CurrentVelocity				=	0.0								
				,	MaxVelocity					=	0.0									
				,	MaxAcceleration				=	0.0								
				,	TargetPosition				=	0.0
				,	ExecutionTimeForCurrentDOF	=	0.0
				,	MinimumExecutionTime		=	0.0;

	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		if ((*IP.SelectionVector)[i])
		{
			CurrentPosition				=	(*IP.CurrentPosition	)[i];
			CurrentVelocity				=	(*IP.CurrentVelocity	)[i];
			MaxVelocity					=	(*IP.MaxVelocity		)[i];
			MaxAcceleration				=	(*IP.MaxAcceleration	)[i];
			TargetPosition				=	(*IP.TargetPosition		)[i];
			ExecutionTimeForCurrentDOF	=	0.0;

			if (!TypeIRMLMath::Decision_1001(CurrentVelocity))
			{
				CurrentPosition		=	-CurrentPosition	;
				CurrentVelocity		=	- CurrentVelocity	;	
				TargetPosition		=	-TargetPosition		;
			}

			if (!TypeIRMLMath::Decision_1002(		CurrentVelocity
												,	MaxVelocity))
			{
				// v --> vmax
				ExecutionTimeForCurrentDOF	+=	(CurrentVelocity - MaxVelocity) / MaxAcceleration;
				CurrentPosition				+=	0.5 * (pow2(CurrentVelocity) - pow2(MaxVelocity))
												/ MaxAcceleration;
				CurrentVelocity				=	MaxVelocity;
			}

			if (!TypeIRMLMath::Decision_1003(		CurrentPosition
												,	CurrentVelocity
												,	MaxAcceleration
												,	TargetPosition))
			{
				// v --> 0
				ExecutionTimeForCurrentDOF	+=	CurrentVelocity / MaxAcceleration;
				CurrentPosition				+=	0.5 * pow2(CurrentVelocity) / MaxAcceleration;
				CurrentVelocity				=	0.0;

				// switch signs
				CurrentPosition		=	-CurrentPosition	;
				CurrentVelocity		=	-CurrentVelocity	;
				TargetPosition		=	-TargetPosition		;

			}

			if (TypeIRMLMath::Decision_1004(		CurrentPosition
												,	CurrentVelocity
												,	MaxVelocity
												,	MaxAcceleration
												,	TargetPosition))
			{
				ExecutionTimeForCurrentDOF +=	TypeIRMLMath::ProfileStep1PosTrap(		CurrentPosition
																					,	CurrentVelocity
																					,	MaxVelocity
																					,	MaxAcceleration
																					,	TargetPosition);
			}
			else
			{
				ExecutionTimeForCurrentDOF +=	TypeIRMLMath::ProfileStep1PosTri(		CurrentPosition
																					,	CurrentVelocity
																					,	MaxAcceleration
																					,	TargetPosition);
			}
		}

		if (ExecutionTimeForCurrentDOF > MinimumExecutionTime)
		{
			MinimumExecutionTime = ExecutionTimeForCurrentDOF;
		}
	}

	return (MinimumExecutionTime);
}


//************************************************************************************
// SynchronizeTrajectory()

void TypeIRML::SynchronizeTrajectory(		const TypeIRMLInputParameters	&IP
										,	const double					&SynchronizationTime
										,	TypeIMotionPolynomials			*PolynomialArray)
{
	bool			SignsWereSwitched			=	false;

	int				i							=	0;

	double			CurrentPosition				=	0.0
				,	CurrentVelocity				=	0.0								
				,	MaxVelocity					=	0.0									
				,	MaxAcceleration				=	0.0								
				,	TargetPosition				=	0.0
				,	ElapsedTime					=	0.0;

	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		if ((*IP.SelectionVector)[i])
		{
			CurrentPosition						=	(*IP.CurrentPosition	)[i];
			CurrentVelocity						=	(*IP.CurrentVelocity	)[i];
			MaxVelocity							=	(*IP.MaxVelocity		)[i];
			MaxAcceleration						=	(*IP.MaxAcceleration	)[i];
			TargetPosition						=	(*IP.TargetPosition		)[i];

			SignsWereSwitched					=	false;
			ElapsedTime							=	0.0;

			Polynomials[i].ValidPolynomials	=	0;

			if (!TypeIRMLMath::Decision_2001(CurrentVelocity))
			{
				SignsWereSwitched	=	!SignsWereSwitched	;
				CurrentPosition		=	-CurrentPosition	;
				CurrentVelocity		=	-CurrentVelocity	;	
				TargetPosition		=	-TargetPosition		;
			}

			if (!TypeIRMLMath::Decision_2002(		CurrentVelocity
												,	MaxVelocity))
			{
				TypeIRMLMath::ProfileStep2V_To_Vmax(	&CurrentPosition									
													,	&CurrentVelocity
													,	MaxVelocity
													,	MaxAcceleration
													,	&ElapsedTime					
													,	SignsWereSwitched
													,	&(PolynomialArray[i]));
			}

			if (TypeIRMLMath::Decision_2003(		CurrentPosition
												,	CurrentVelocity
												,	MaxAcceleration
												,	TargetPosition))
			{

				if (TypeIRMLMath::Decision_2004(		CurrentPosition
													,	CurrentVelocity
													,	MaxAcceleration
													,	TargetPosition
													,	SynchronizationTime))
				{
					TypeIRMLMath::ProfileStep2PosTrap(		&CurrentPosition
														,	&CurrentVelocity
														,	MaxAcceleration
														,	TargetPosition
														,	SynchronizationTime
														,	&ElapsedTime					
														,	SignsWereSwitched
														,	&(PolynomialArray[i]));

				}
				else
				{
					TypeIRMLMath::ProfileStep2NegHldNegLin(		&CurrentPosition
															,	&CurrentVelocity
															,	MaxAcceleration
															,	TargetPosition
															,	SynchronizationTime
															,	&ElapsedTime					
															,	SignsWereSwitched
															,	&(PolynomialArray[i]));
				}
			}
			else
			{
				TypeIRMLMath::ProfileStep2V_To_Zero(	&CurrentPosition									
													,	&CurrentVelocity
													,	MaxAcceleration
													,	&ElapsedTime					
													,	SignsWereSwitched
													,	&(PolynomialArray[i]));

				// Switch signs
				SignsWereSwitched	=	!SignsWereSwitched	;
				CurrentPosition		=	-CurrentPosition	;
				CurrentVelocity		=	-CurrentVelocity	;	
				TargetPosition		=	-TargetPosition		;

				TypeIRMLMath::ProfileStep2PosTrap(		&CurrentPosition
													,	&CurrentVelocity
													,	MaxAcceleration
													,	TargetPosition
													,	SynchronizationTime
													,	&ElapsedTime					
													,	SignsWereSwitched
													,	&(PolynomialArray[i]));
			}
		}
	}
				
	return;
}


//************************************************************************************
// CalculateOutputValues()

int TypeIRML::CalculateOutputValues(		const TypeIMotionPolynomials	*PolynomialArray
										,	const TypeIRMLBoolVector		&SelectionVector
										,	TypeIRMLOutputParameters		*OP)
{
	int				i							=	0
				,	SegmentCounter				=	0
				,	ReturnValue					=	TypeIRML::RML_FINAL_STATE_REACHED;


	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		SegmentCounter = 0;

		if (SelectionVector[i])
		{
			while (this->CycleTime >= PolynomialArray[i].PolynomialTimes[SegmentCounter])
			{
				SegmentCounter++;
				if (SegmentCounter > PolynomialArray[i].ValidPolynomials)
				{
					return(TypeIRML::RML_ERROR);
				}
			}
			
			(*OP->NewPosition)[i] = PolynomialArray[i].PositionPolynomial[SegmentCounter].CalculateValue(this->CycleTime);
			(*OP->NewVelocity)[i] = PolynomialArray[i].VelocityPolynomial[SegmentCounter].CalculateValue(this->CycleTime);


			if (SegmentCounter + 1 < PolynomialArray[i].ValidPolynomials)
			{
				ReturnValue = TypeIRML::RML_WORKING;
			}
		}
	}

	return (ReturnValue);
}


//************************************************************************************
// GetNextMotionState_Velocity

int TypeIRML::GetNextMotionState_Velocity(		const double*	CurrentPosition
											,	const double*	CurrentVelocity
											,	const double*	MaxAcceleration
											,	const double*	TargetVelocity
											,	const bool*		SelectionVector
											,	double*			NewPosition
											,	double*			NewVelocity	)
{
	int				i							=	0
				,	ReturnValue					=	TypeIRML::RML_FINAL_STATE_REACHED;

	double			MinimumExecutionTime		=	0.0;

	this->TrajectoryExecutionTimeForTheUser		=	0.0;
	
	for (i = 0; i < this->NumberOfDOFs; i++)
	{
		if (SelectionVector[i])
		{
			if (MaxAcceleration[i] < RML_MIN_VALUE_FOR_MAXACCELERATION)
			{
				return (TypeIRML::RML_MAX_ACCELERATION_ERROR);
			}

			MinimumExecutionTime	=	fabs(CurrentVelocity[i] - TargetVelocity[i]) / MaxAcceleration[i];

			if (MinimumExecutionTime > this->TrajectoryExecutionTimeForTheUser)
			{
				this->TrajectoryExecutionTimeForTheUser = MinimumExecutionTime;
			}

			if (MinimumExecutionTime > this->CycleTime)
			{
				if (CurrentVelocity[i] > TargetVelocity[i])
				{
					NewVelocity[i]	=	CurrentVelocity[i] - MaxAcceleration[i] * this->CycleTime;
				}
				else
				{
					NewVelocity[i]	=	CurrentVelocity[i] + MaxAcceleration[i] * this->CycleTime;						
				}
				NewPosition[i]	=	CurrentPosition[i] + 0.5 * (NewVelocity[i] + CurrentVelocity[i]) * this->CycleTime;

				ReturnValue		=	TypeIRML::RML_WORKING;
			}
			else
			{
				NewVelocity[i]	=	TargetVelocity[i];
				NewPosition[i]	=	CurrentPosition[i] + 0.5 * MinimumExecutionTime
									* (CurrentVelocity[i] - TargetVelocity[i])
									+ TargetVelocity[i] * this->CycleTime;
			}
		}
	}

	return(ReturnValue);
}


//************************************************************************************
// GetNextMotionState_Velocity

int TypeIRML::GetNextMotionState_Velocity(		const TypeIRMLInputParameters	&IP
											,	TypeIRMLOutputParameters		*OP	)
{
	return(GetNextMotionState_Velocity(		IP.CurrentPosition->VecData
										,	IP.CurrentVelocity->VecData
										,	IP.MaxAcceleration->VecData
										,	IP.TargetVelocity->VecData
										,	IP.SelectionVector->VecData
										,	OP->NewPosition->VecData
										,	OP->NewVelocity->VecData		));
}


//*******************************************************************************************
// GetExecutionTime()

double TypeIRML::GetExecutionTime(void) const
{
	return(TrajectoryExecutionTimeForTheUser);
}

