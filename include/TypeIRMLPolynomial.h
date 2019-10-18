//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLPolynomial.h
//!
//! \brief
//! Header file for polynomials designed for the Type I On-Line Trajectory
//! Generation algorithm
//!
//! \details
//! Header file for a polynomial class designed for the Type I
//! On-Line Trajectory Generation algorithm. This class is part
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


#ifndef __TypeIRMLPolynomial__
#define __TypeIRMLPolynomial__



namespace TypeIRMLMath
{


//  ---------------------- Doxygen info ----------------------
//! \def MAXIMAL_NO_OF_POLYNOMIALS
//!
//! \brief
//! Maximum number of polynomials
//! 
//! \details
//! Nominal 5 + 1 for the time after the target state has been reached
//  ----------------------------------------------------------
#define MAXIMAL_NO_OF_POLYNOMIALS		6


//  ---------------------- Doxygen info ----------------------
//! \class TypeIRMLPolynomial
//! 
//! \brief This class realizes polynomials of degree three as required for
//! the Type I On-Line Trajectory Generator
//! 
//! \sa struct MotionPolynomials
//  ----------------------------------------------------------
class TypeIRMLPolynomial
{
public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLPolynomial(void)
//! 
//! \brief
//! Constructor of the class TypeIRMLPolynomial
//  ----------------------------------------------------------
	TypeIRMLPolynomial(void);


//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIRMLPolynomial(void)
//! 
//! \brief
//! Destructor of the class TypeIRMLPolynomial
//  ----------------------------------------------------------
	~TypeIRMLPolynomial(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCoefficients(const double &Coeff2, const double &Coeff1, const double &Coeff0, const double &Diff)
//! 
//! \brief
//!  Sets the coefficients for the polynomial of degree two
//! 
//! \details
//! \f$ f(t) = a_2\,\cdot\,(t - \Delta T)^2 + a_1\,\cdot\,(t - \Delta T) + a_0 \f$
//! 
//! \param Coeff2
//! \f$\ \Longrightarrow \ a_2\f$ \n
//! 
//! \param Coeff1
//! \f$\ \Longrightarrow \ a_1\f$ \n
//! 
//! \param Coeff0
//! \f$\ \Longrightarrow \ a_0\f$ \n
//! 
//! \param Diff
//! \f$\ \Longrightarrow \ \Delta T\f$.
//  ----------------------------------------------------------
	void		SetCoefficients(	const double	&Coeff2
								,	const double	&Coeff1
								,	const double	&Coeff0
								,	const double	&Diff);


//  ---------------------- Doxygen info ----------------------
//! \fn double CalculateValue(const double &t) const
//! 
//! \brief
//! Calculates the function value of f(t) at t
//! 
//! \details
//! \f$ f(t) = a_2\,\cdot\,(t - \Delta T)^2 + a_1\,\cdot\,(t - \Delta T) + a_0 \f$
//!
//! \param t
//! Function input value
//!
//! \return
//! The function value at \f$t\f$, \f$ f(t) \f$
//  ----------------------------------------------------------
	double		CalculateValue(const double &t) const;


private:

	unsigned int		Degree;

	double				a2
					,	a1
					,	a0
					,	DeltaT;


};	// class TypeIRMLPolynomial


//  ---------------------- Doxygen info ----------------------
//! \struct TypeIMotionPolynomials
//!
//! \brief
//! Three arrays of TypeIRMLMath::TypeIRMLPolynomial
//!
//! \details
//! This data structure contains three arrays of polynomials required for
//! the Type I On-Line Trajectory Generation algorithm. Furthermore, this 
//! data structure contains the times until each single three-tuple of
//! polynomials is valid and the number of used polynomial three-tuples
//! that are currently in use.
//! 
//! \sa class TypeIRMLPolynomial
//  ----------------------------------------------------------
struct TypeIMotionPolynomials
{

//  ---------------------- Doxygen info ----------------------
//! \var double PolynomialTimes [MAXIMAL_NO_OF_POLYNOMIALS]
//!
//! \brief
//! An array of ending times in seconds
//! 
//! \details
//! An array of ending times in seconds, until which a polynomial is valid
//! (e.g., \c PolynomialTimes[4] determines the ending time of the fourth
//! polynomial).
//  ----------------------------------------------------------
	double					PolynomialTimes			[MAXIMAL_NO_OF_POLYNOMIALS]	;
	
	
//  ---------------------- Doxygen info ----------------------
//! \var TypeIRMLPolynomial PositionPolynomial [MAXIMAL_NO_OF_POLYNOMIALS]
//!
//! \brief
//! An array of position polynomials
//!
//! \details
//! An array of position polynomials, that is, objects of the
//! class TypeIRMLPolynomial, that is,
//! \f$\ _{k}^{l}p_{i}(t)\ \forall\ l\ \in\ \{1,\,\dots,\,L\} \f$,
//! where \f$ L \f$ is value of \c MAXIMAL_NO_OF_POLYNOMIALS.
//  ----------------------------------------------------------	
	TypeIRMLPolynomial		PositionPolynomial		[MAXIMAL_NO_OF_POLYNOMIALS]	;
	
	
//  ---------------------- Doxygen info ----------------------
//! \var TypeIRMLPolynomial VelocityPolynomial [MAXIMAL_NO_OF_POLYNOMIALS]
//!
//! \brief
//! An array of velocity polynomials
//!
//! \details
//! An array of velocity polynomials, that is, objects of the
//! class TypeIRMLPolynomial, that is, 
//! \f$\ _{k}^{l}v_{i}(t)\ \forall\ l\ \in\ \{1,\,\dots,\,L\} \f$,
//! where \f$ L \f$ is value of \c MAXIMAL_NO_OF_POLYNOMIALS.
//  ----------------------------------------------------------	
	TypeIRMLPolynomial		VelocityPolynomial		[MAXIMAL_NO_OF_POLYNOMIALS]	;
	

//  ---------------------- Doxygen info ----------------------
//! \var unsigned char ValidPolynomials
//!
//! \brief
//! The number of polynomials in use (0 ... \c MAXIMAL_NO_OF_POLYNOMIALS).
//  ----------------------------------------------------------	
	unsigned char			ValidPolynomials									;
};


}	// namespace TypeIRMLMath


#endif
