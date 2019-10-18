//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLPolynomial.cpp
//!
//! \brief
//! Implementation file for polynomials designed for the Type I On-Line Trajectory
//! Generation algorithm
//!
//! \sa TypeIRMLPolynomial.h
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



#include <TypeIRMLPolynomial.h>
#include <TypeIRMLMath.h>


//************************************************************************************
// Constructor


TypeIRMLMath::TypeIRMLPolynomial::TypeIRMLPolynomial()
{
	a0		= 0.0;
	a1		= 0.0;
	a2		= 0.0;
	DeltaT	= 0.0;
	Degree	= 0;
}


//************************************************************************************
// Destructor

TypeIRMLMath::TypeIRMLPolynomial::~TypeIRMLPolynomial()
{}


//************************************************************************************
// SetCoefficients()
// f(x) = a_2 * (t - DeltaT)^2 + a_1 * (t - DeltaT) + a_0

void TypeIRMLMath::TypeIRMLPolynomial::SetCoefficients(		const double	&Coeff2
														,	const double	&Coeff1
														,	const double	&Coeff0
														,	const double	&Diff)
{
	a0		= Coeff0;
	a1		= Coeff1;
	a2		= Coeff2;
	DeltaT	= Diff;

	if (a2 != 0.0)
	{
		Degree = 2;
		return;
	}

	if (a1 != 0.0)
	{
		Degree = 1;
		return;
	}

	Degree = 0;
	return;
}


//*******************************************************************************************
// CalculateValue()
// calculates f(t)

double TypeIRMLMath::TypeIRMLPolynomial::CalculateValue(const double &t) const
{
	return(	((Degree == 2)?
			(a2 * (t - DeltaT) * (t - DeltaT) + a1 * (t - DeltaT) + a0):
			((Degree == 1)?
			(a1 * (t - DeltaT) + a0):
			(a0))));
}
