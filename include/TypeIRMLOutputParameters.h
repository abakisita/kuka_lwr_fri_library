//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLOutputParameters.h
//!
//! \brief
//! Header file for the output parameters of the Reflexxes Motion Library
//! (Type I)
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


#ifndef __TypeIRMLOutputParameters__
#define __TypeIRMLOutputParameters__


#include <TypeIRMLVector.h>


namespace TypeIRMLMath
{


//  ---------------------- Doxygen info ----------------------
//! \class TypeIRMLOutputParameters
//!
//! \brief
//! This class describes the output parameters of the Reflexxes Motion Library
//! (Type I)
//! 
//! \details
//! It is part of the namespace TypeIRMLMath.
//! 
//! \sa TypeIRML
//! \sa TypeIRMLInputParameters
//  ----------------------------------------------------------
class TypeIRMLOutputParameters
{

public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLOutputParameters(const unsigned int &NumberOfDOFs)
//!
//! \brief
//! Constructor of the class TypeIRMLOutputParameters
//!
//! \details
//! Allocates memory for the specified number of degrees of freedom
//!
//! \warning
//! A call of this method is \em not real-time capable!!!
//!
//! \param NumberOfDOFs
//! Number of degrees of freedom
//  ----------------------------------------------------------

	TypeIRMLOutputParameters(const unsigned int &NumberOfDOFs)
	{
		this->NewPosition			=	new TypeIRMLDoubleVector(NumberOfDOFs);
		this->NewVelocity			=	new TypeIRMLDoubleVector(NumberOfDOFs);

		this->NewPosition->Set(0.0);
		this->NewVelocity->Set(0.0);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLOutputParameters(const TypeIRMLOutputParameters & OP)
//!
//! \brief
//! Copy constructor of the class TypeIRMLOutputParameters
//!
//! \details
//! Allocates memory for the specified number of degrees of freedom.
//!
//! \warning
//! \brief A call of this method is \em not real-time capable!!!
//!
//! \param OP
//! Original object reference
//  ----------------------------------------------------------
	TypeIRMLOutputParameters(const TypeIRMLOutputParameters & OP)
	{
		this->NewPosition	=	new TypeIRMLDoubleVector(OP.NewPosition->VectorDimension);
		this->NewVelocity	=	new TypeIRMLDoubleVector(OP.NewVelocity->VectorDimension);

		this->NewPosition	=	OP.NewPosition;
		this->NewVelocity	=	OP.NewVelocity;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLOutputParameters &operator = (const TypeIRMLOutputParameters &OP)
//!
//! \brief
//! Copy operator
//!
//! \param OP
//! Set of output parameters
//  ----------------------------------------------------------
	TypeIRMLOutputParameters &operator = (const TypeIRMLOutputParameters &OP)
	{
		this->NewPosition	=	OP.NewPosition;
		this->NewVelocity	=	OP.NewVelocity;

		return (*this);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIRMLOutputParameters(void)
//!
//! \brief
//! Destructor of class TypeIRMLOutputParameters, frees heap memory
//  ----------------------------------------------------------
	~TypeIRMLOutputParameters(void)
	{
		delete this->NewPosition	;
		delete this->NewVelocity	;

		this->NewPosition	=	NULL;
		this->NewVelocity	=	NULL;
	}

	TypeIRMLDoubleVector*	NewPosition;
	TypeIRMLDoubleVector*	NewVelocity;
};



}	// namespace

#endif
