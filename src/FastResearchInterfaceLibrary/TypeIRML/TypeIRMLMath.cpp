//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLMath.cpp
//!
//! \brief
//! Implementation file for functions and definitions of constant values
//! and macros
//! 
//! \sa TypeIRMLMath.h
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



#include <TypeIRMLMath.h>
#include <math.h>



//****************************************************************************
// RMLSqrt()

double TypeIRMLMath::RMLSqrt(const double &Value)
{
	return( ( Value <= 0.0 ) ? ( 0.0 ) : ( sqrt( Value ) ) );
}
