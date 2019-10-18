//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLProfiles.h
//!
//! \brief
//! Header file for the calculation of all motion
//! profiles for the Type I On-Line Trajectory Generation algorithm
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


#ifndef __TypeIRMLProfiles__
#define __TypeIRMLProfiles__


#include <TypeIRMLPolynomial.h>


namespace TypeIRMLMath
{	

//  ---------------------- Doxygen info ----------------------
//! \fn double ProfileStep1PosTri(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief
//! Calculates the execution time of the PosTri velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \return
//! Execution time for this profile in seconds
//  ----------------------------------------------------------
double ProfileStep1PosTri(		const double &CurrentPosition
							,	const double &CurrentVelocity
							,	const double &MaxAcceleration
							,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn double ProfileStep1PosTrap(const double &CurrentPosition, const double &CurrentVelocity, const double &MaxVelocity, const double &MaxAcceleration, const double &TargetPosition)
//!
//! \brief Calculates the execution time of the PosTrap velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \return
//! Execution time for this profile in seconds
//  ----------------------------------------------------------
double ProfileStep1PosTrap(		const double &CurrentPosition
							,	const double &CurrentVelocity
							,	const double &MaxVelocity
							,	const double &MaxAcceleration
							,	const double &TargetPosition);


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2V_To_Vmax(double *CurrentPosition, double *CurrentVelocity, const double &MaxVelocity, const double &MaxAcceleration, double *ElapsedTime,	const bool &SignsAreInverted, TypeIRMLMath::TypeIMotionPolynomials *P)
//!
//! \brief Calculates the polynomial parameters to bring the velocity
//! value to the maximum velocity
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxVelocity
//! Maximum allowed velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i}^{\,max} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param ElapsedTime
//! Elapsed time in seconds, this value may be modified by the method.
//!
//! \param SignsAreInverted
//! Indicates whether the signs of the above values are inverted
//!
//! \param P
//! The set of polynomials that are parameterized by this method
//  ----------------------------------------------------------
void ProfileStep2V_To_Vmax(		double 									*CurrentPosition
							,	double 									*CurrentVelocity
							,	const double							&MaxVelocity
							,	const double							&MaxAcceleration
							,	double									*ElapsedTime					
							,	const bool								&SignsAreInverted
							,	TypeIRMLMath::TypeIMotionPolynomials	*P					);


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2V_To_Zero(double *CurrentPosition, double *CurrentVelocity, const double &MaxAcceleration, double *ElapsedTime, const bool &SignsAreInverted, TypeIRMLMath::TypeIMotionPolynomials *P)
//!
//! \brief Calculates the polynomial parameters to bring the velocity
//! value to zero
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param ElapsedTime
//! Elapsed time in seconds, this value may be modified by the method.
//!
//! \param SignsAreInverted
//! Indicates whether the signs of the above values are inverted
//!
//! \param P
//! The set of polynomials that are parameterized by this method
//  ----------------------------------------------------------
void ProfileStep2V_To_Zero(		double 									*CurrentPosition
							,	double 									*CurrentVelocity
							,	const double							&MaxAcceleration
							,	double									*ElapsedTime					
							,	const bool								&SignsAreInverted
							,	TypeIRMLMath::TypeIMotionPolynomials	*P					);


//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2PosTrap(double *CurrentPosition, double *CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition, const double &SynchronizationTime, double *ElapsedTime, const bool &SignsAreInverted, TypeIRMLMath::TypeIMotionPolynomials *P)
//!
//! \brief Calculates all trajectory parameters the PosTrap
//! velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param SynchronizationTime
//! Synchronization time \f$\ t_{i}^{\,sync} \f$ 
//!
//! \param ElapsedTime
//! Elapsed time in seconds, this value may be modified by the method.
//!
//! \param SignsAreInverted
//! Indicates whether the signs of the above values are inverted
//!
//! \param P
//! The set of polynomials that are parameterized by this method
//  ----------------------------------------------------------
void ProfileStep2PosTrap(		double 									*CurrentPosition
							,	double									*CurrentVelocity
							,	const double 							&MaxAcceleration
							,	const double 							&TargetPosition
							,	const double 							&SynchronizationTime
							,	double									*ElapsedTime					
							,	const bool								&SignsAreInverted
							,	TypeIRMLMath::TypeIMotionPolynomials	*P						);

//  ---------------------- Doxygen info ----------------------
//! \fn void ProfileStep2NegHldNegLin(double *CurrentPosition, double *CurrentVelocity, const double &MaxAcceleration, const double &TargetPosition, const double &SynchronizationTime, double *ElapsedTime, const bool &SignsAreInverted, TypeIRMLMath::TypeIMotionPolynomials *P)
//!
//! \brief Calculates all trajectory parameters the NegHldNegLin
//! velocity profile
//!
//! \param CurrentPosition
//! Current position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i} \f$
//!
//! \param CurrentVelocity
//! Current velocity value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}V_{i} \f$
//!
//! \param MaxAcceleration
//! Maximum allowed acceleration value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}A_{i}^{\,max} \f$
//!
//! \param TargetPosition
//! Target position value for DOF \f$ k \f$ at instant
//! \f$ T_{i} \f$, \f$\ _{k}P_{i}^{\,trgt} \f$
//!
//! \param SynchronizationTime
//! Synchronization time \f$\ t_{i}^{\,sync} \f$ 
//!
//! \param ElapsedTime
//! Elapsed time in seconds, this value may be modified by the method.
//!
//! \param SignsAreInverted
//! Indicates whether the signs of the above values are inverted
//!
//! \param P
//! The set of polynomials that are parameterized by this method
//  ----------------------------------------------------------
void ProfileStep2NegHldNegLin(		double		 							*CurrentPosition
								,	double	 								*CurrentVelocity
								,	const double 							&MaxAcceleration
								,	const double 							&TargetPosition
								,	const double 							&SynchronizationTime
								,	double									*ElapsedTime					
								,	const bool								&SignsAreInverted
								,	TypeIRMLMath::TypeIMotionPolynomials	*P						);


}	// namespace TypeIRMLMath

#endif
