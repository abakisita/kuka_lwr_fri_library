//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLMath.h
//!
//! \brief
//! Header file for functions and definitions of constant values and macros
//! 
//! \details
//! Header file for definitions of constant values and macros to be used
//! for within in the library of the Type I On-Line Trajectory Algorithm.
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


#ifndef __TypeIRMLMath__
#define __TypeIRMLMath__


//*******************************************************************************************
// Include files



namespace TypeIRMLMath
{

//*******************************************************************************************
// Definitions and macros




//  ---------------------- Doxygen info ----------------------
//! \def RML_INFINITY
//!
//! \brief
//! A value for infinity \f$ \infty = 10^{100} \f$
//  ----------------------------------------------------------
#define		RML_INFINITY				((double)1.0e100)


//  ---------------------- Doxygen info ----------------------
//! \def RML_DENOMINATOR_EPSILON
//!
//! \brief
//! A threshold value for zero to be used for denominators
//  ----------------------------------------------------------
#define		RML_DENOMINATOR_EPSILON		((double)1.0e-12)


//  ---------------------- Doxygen info ----------------------
//! \def RML_MIN_VALUE_FOR_MAXVELOCITY
//!
//! \brief
//! Positive threshold value to determine the minimum allowed value for
//! the maximum velocity value
//  ----------------------------------------------------------
#define		RML_MIN_VALUE_FOR_MAXVELOCITY		((double)1.0e-4)


//  ---------------------- Doxygen info ----------------------
//! \def RML_MIN_VALUE_FOR_MAXACCELERATION
//!
//! \brief
//! Positive threshold value to determine the minimum allowed value
//! for the maximum acceleration value
//  ----------------------------------------------------------
#define		RML_MIN_VALUE_FOR_MAXACCELERATION	((double)1.0e-4)


//  ---------------------- Doxygen info ----------------------
//! \def pow2(A)
//!
//! \brief
//! A to the power of 2 
//!
//! \param A
//! Basis
//!
//! \return
//! Result value
//  ----------------------------------------------------------
#define pow2(A)							((A)*(A))


//  ---------------------- Doxygen info ----------------------
//! \fn double RMLSqrt(const double &Value)
//!
//! \brief Calculates the real square root of a given value
//!
//! \details
//! If the value is negative a value of zero will be returned.
//!
//! \param Value
//! Square root radicand
//!
//! \return
//! Square root value (real)
//  ----------------------------------------------------------
double RMLSqrt(const double &Value);


}	// namespace TypeIRMLMath

#endif
