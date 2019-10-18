//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLDecision.h
//!
//! \brief
//! Header file for decisions of the two decision trees of the 
//! Type I On-Line Trajectory Generation algorithm
//!
//! \details
//! This file contains all neccessary decisions for the Type I On-Line
//! Trajectory Generation algorithm. All functions are part
//! of the namespace TypeIRMLMath.
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


#ifndef __TypeIRMLDecision__
#define __TypeIRMLDecision__



namespace TypeIRMLMath
{	

//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1001(const double &CurrentVelocity)
//!
//! \brief
//! Is (vi >= 0)?
//  ----------------------------------------------------------
bool Decision_1001(		const double &CurrentVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1002(const double &CurrentVelocity, const double &MaxVelocity)
//!
//! \brief
//! Is (vi >= vmax)?
//  ----------------------------------------------------------
//is (vi <= vmax)?
bool Decision_1002(		const double &CurrentVelocity
					,	const double &MaxVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1003(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief
//! If v->0, is (p<=ptrgt)?
//  ----------------------------------------------------------
bool Decision_1003(		const double &CurrentPosition
					,	const double &CurrentVelocity
					,	const double &MaxAcceleration
					,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_1004(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief
//! If v->+vmax->0, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_1004(		const double &CurrentPosition
					,	const double &CurrentVelocity
					,	const double &MaxVelocity
					,	const double &MaxAcceleration
					,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2001(const double &CurrentVelocity)
//!
//! \brief
//! Is (vi >= 0)?
//  ----------------------------------------------------------
bool Decision_2001(		const double &CurrentVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2002(const double &CurrentVelocity, const double &MaxVelocity)
//!
//! \brief
//! Is (vi <= vmax)?
//  ----------------------------------------------------------
bool Decision_2002(		const double &CurrentVelocity
					,	const double &MaxVelocity);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2003(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief
//! If v->0, is (p<=ptrgt)?
//  ----------------------------------------------------------
bool Decision_2003(		const double &CurrentPosition
					,	const double &CurrentVelocity
					,	const double &MaxAcceleration
					,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Decision_2004(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition, const double &SynchronizationTime)
//!
//! \brief
//! If v->hold->0, so that t=tsync, is p<=ptrgt?
//  ----------------------------------------------------------
bool Decision_2004(		const double &CurrentPosition
					,	const double &CurrentVelocity
					,	const double &MaxAcceleration
					,	const double &TargetPosition
					,	const double &SynchronizationTime);

}	// namespace TypeIRMLMath

#endif
