//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLDecision.cpp
//!
//! \brief
//! Implementation file for decisions of the two decision trees of the 
//! Type I On-Line Trajectory Generation algorithm
//!
//! \sa TypeIRMLDecision.h
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



#include <TypeIRMLDecision.h>
#include <TypeIRMLMath.h>


//************************************************************************************
// Decision_1001()

bool TypeIRMLMath::Decision_1001(const double &CurrentVelocity)
{
	return(CurrentVelocity >= 0.0);
}


//************************************************************************************
// Decision_1002()

bool TypeIRMLMath::Decision_1002(		const double &CurrentVelocity
									,	const double &MaxVelocity)
{
	return(CurrentVelocity <= MaxVelocity);
}


//************************************************************************************
// Decision_1003()

bool TypeIRMLMath::Decision_1003(		const double &CurrentPosition
									,	const double &CurrentVelocity
									,	const double &MaxAcceleration
									,	const double &TargetPosition)
{
	return((CurrentPosition + 0.5 * pow2(CurrentVelocity) / MaxAcceleration) <= TargetPosition);
}


//************************************************************************************
// Decision_1004()

bool TypeIRMLMath::Decision_1004(		const double &CurrentPosition
									,	const double &CurrentVelocity
									,	const double &MaxVelocity
									,	const double &MaxAcceleration
									,	const double &TargetPosition)
{
	return( (CurrentPosition + (2.0 * pow2(MaxVelocity) - pow2(CurrentVelocity))
				/ (2.0 * MaxAcceleration) ) <= TargetPosition );
}


//************************************************************************************
// Decision_2001()

bool TypeIRMLMath::Decision_2001(const double &CurrentVelocity)
{
	return(TypeIRMLMath::Decision_1001(CurrentVelocity));
}


//************************************************************************************
// Decision_2002()

bool TypeIRMLMath::Decision_2002(		const double &CurrentVelocity
									,	const double &MaxVelocity)
{
	return(TypeIRMLMath::Decision_1002(		CurrentVelocity
										,	MaxVelocity));
}


//************************************************************************************
// Decision_2003()

bool TypeIRMLMath::Decision_2003(		const double &CurrentPosition
									,	const double &CurrentVelocity
									,	const double &MaxAcceleration
									,	const double &TargetPosition)
{
	return(TypeIRMLMath::Decision_1003(		CurrentPosition
										,	CurrentVelocity
										,	MaxAcceleration
										,	TargetPosition	));
}


//************************************************************************************
// Decision_2004()

bool TypeIRMLMath::Decision_2004(		const double &CurrentPosition
									,	const double &CurrentVelocity
									,	const double &MaxAcceleration
									,	const double &TargetPosition
									,	const double &SynchronizationTime)
{
	return((CurrentPosition + SynchronizationTime * CurrentVelocity
			- 0.5 * pow2(CurrentVelocity) / MaxAcceleration) <= TargetPosition);
}