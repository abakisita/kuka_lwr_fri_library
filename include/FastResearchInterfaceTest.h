//  ---------------------- Doxygen info ----------------------
//! \file FastResearchInterfaceTest.h
//!
//! \brief
//! Header file for some functions of FastResearchInterfaceTest application
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


#ifndef __FastResearchInterfaceTest__
#define __FastResearchInterfaceTest__

void RunTrajectory						(FastResearchInterface	*FRI);
void HapticsDemo						(FastResearchInterface	*FRI);
void ImpedanceDemo						(FastResearchInterface	*FRI);
void MoveToCandle						(FastResearchInterface	*FRI);
void Cart1DDemo							(FastResearchInterface	*FRI);
void LineCompliance						(FastResearchInterface	*FRI);
void RunTrajectorySimple				(FastResearchInterface	*FRI, float Joint_values[7]);
void StateSpaceSwitching				(FastResearchInterface	*FRI);
void SensorGuidedSwitching				(FastResearchInterface	*FRI);
void TransOTG							(FastResearchInterface	*FRI);
void ICRAOTG							(FastResearchInterface	*FRI);
void Anya								(FastResearchInterface	*FRI);
void KinectJointSpace					(FastResearchInterface	*FRI);
void KinectCartSpace					(FastResearchInterface	*FRI);
void StateSpaceSwitchingWithCompliance	(FastResearchInterface	*FRI);
void StanfordTrackingCartSpace			(FastResearchInterface	*FRI);
void StanfordTrackingJointSpace			(FastResearchInterface	*FRI);
void JediBot							(FastResearchInterface	*FRI);
void RobertsDemo						(FastResearchInterface	*FRI);
void SweepTest							(FastResearchInterface	*FRI);
void SweepTestPosCtrl					(FastResearchInterface	*FRI);




enum RPYSolution
{
	RPYSolution1	=	1,
	RPYSolution2	=	2
};


void CalculateRPYAnglesFromFrame(const double Frame[12], double RPYAnglesInDegrees[3], const RPYSolution Solution);
void CalculateRPYAnglesFromFrame(const float Frame[12], float RPYAnglesInDegrees[3], const RPYSolution Solution);

void CalculateFrameFromRPYAngles(const double RPYAnglesInDegrees[3], double Frame[12]);
void CalculateFrameFromRPYAngles(const float RPYAnglesInDegrees[3], float Frame[12]);

double atan2_New(const double b, const double a);



inline void PrintFrame(float *Frame)
{
	unsigned int		i		=	0
					,	j		=	0;

	for ( i = 0; i < 3; i++)
	{
		printf("( ");
		for ( j = 0; j < 4; j++)
		{
			printf("   %8.3f   ", Frame[4 * i + j]);
		}
		printf(" )\n");
	}
	printf("(	   0.000		 0.000		 0.000		 1.000	)\n\n");
}


inline void PrintFrame(double *Frame)
{
	unsigned int		i		=	0
					,	j		=	0;

	for ( i = 0; i < 3; i++)
	{
		printf("( ");
		for ( j = 0; j < 4; j++)
		{
			printf("   %8.3f   ", Frame[4 * i + j]);
		}
		printf(" )\n");
	}
	printf("(	   0.000		 0.000		 0.000		 1.000	)\n\n");
}

inline void PrintFrame(double Frame[3][3])
{
	unsigned int		i		=	0
					,	j		=	0;

	float				Values[12];

	for ( i = 0; i < 3; i++)
	{
		for ( j = 0; j < 3; j++)
		{
			Values[4 * i + j]	=	Frame[i][j];
		}
	}

	PrintFrame(Values);
}



#endif
