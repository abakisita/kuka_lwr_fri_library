//  ---------------------- Doxygen info ----------------------
//! \file FRICommunication.h
//!
//! \brief
//! Header file for definitions of data structures used for
//! intercommunication between the remote PC and the KRC
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



#ifndef __FRICommunication__
#define __FRICommunication__


//  ----------------------------------------------------------
// Define signed and unsigned integer data structures

#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
#if (_MSC_VER < 1300)
typedef signed char	   int8_t;
typedef signed short	  int16_t;
typedef signed int		int32_t;
typedef unsigned char	 uint8_t;
typedef unsigned short	uint16_t;
typedef unsigned int	  uint32_t;
#else
typedef signed __int8	 int8_t;
typedef signed __int16	int16_t;
typedef signed __int32	int32_t;
typedef unsigned __int8   uint8_t;
typedef unsigned __int16  uint16_t;
typedef unsigned __int32  uint32_t;
#endif
#else
#include <inttypes.h>
#endif
//  ----------------------------------------------------------



//  ---------------------- Doxygen info ----------------------
//! \def NUMBER_OF_CART_DOFS
//!
//! \brief
//! Number of elements of Cartesian vectors.
//  ----------------------------------------------------------
#define NUMBER_OF_CART_DOFS				6


//  ---------------------- Doxygen info ----------------------
//! \def NUMBER_OF_FRAME_ELEMENTS
//!
//! \brief
//! Number of relevant elements in 4x4 homogeneous
//! transformation matrices. Only the top 3x4 elements are used,
//! and the bottom row of the matrix (0 0 0 1) is not 
//! contained in this number.
//  ----------------------------------------------------------
#define NUMBER_OF_FRAME_ELEMENTS		12


//  ---------------------- Doxygen info ----------------------
//! \def NUMBER_OF_JOINTS
//!
//! \brief
//! Number of joints of the KUKA LWR 4.
//  ----------------------------------------------------------
#define NUMBER_OF_JOINTS				7


//  ---------------------- Doxygen info ----------------------
//! \def SIZE_USER_DATA
//!
//! \brief
//! Number of bytes for "shared memory" between the KRL 
//! environment and the remote PC using this FRI library
//  ----------------------------------------------------------
#define SIZE_USER_DATA					16

  
//  ---------------------- Doxygen info ----------------------
//! \def FRI_DATAGRAM_ID_CMD
//!
//! \brief
//! Mask to identify FRIDataSendToKRC 
//!
//! \sa FastResearchInterface::KRCCommunicationThreadMain()
//  ----------------------------------------------------------
#define FRI_DATAGRAM_ID_CMD				0x1005


//  ---------------------- Doxygen info ----------------------
//! \def MASK_CMD_JNTPOS
//!
//! \brief
//! Flag for joint position control values
//  ----------------------------------------------------------
#define MASK_CMD_JNTPOS					0x0001


//  ---------------------- Doxygen info ----------------------
//! \def MASK_CMD_JNTTRQ
//!
//! \brief
//! Flag for joint torque control values
//  ----------------------------------------------------------
#define MASK_CMD_JNTTRQ					0x0004


//  ---------------------- Doxygen info ----------------------
//! \def MASK_CMD_JNTSTIFF
//!
//! \brief
//! Flag for joint stiffness control values
//  ----------------------------------------------------------
#define MASK_CMD_JNTSTIFF				0x0010


//  ---------------------- Doxygen info ----------------------
//! \def MASK_CMD_JNTDAMP
//!
//! \brief
//! Flag for joint damping control values
//  ----------------------------------------------------------
#define MASK_CMD_JNTDAMP				0x0020


//  ---------------------- Doxygen info ----------------------
//! \def MASK_CMD_CARTPOS
//!
//! \brief
//! Flag for Cartesian position control values
//  ----------------------------------------------------------
#define MASK_CMD_CARTPOS				0x0100


//  ---------------------- Doxygen info ----------------------
//! \def MASK_CMD_TCPFT
//!
//! \brief
//! Flag for Cartesian force/torque control values
//  ----------------------------------------------------------
#define MASK_CMD_TCPFT					0x0400


//  ---------------------- Doxygen info ----------------------
//! \def MASK_CMD_CARTSTIFF
//!
//! \brief
//! Flag for Cartesian stiffness control values
//  ----------------------------------------------------------
#define MASK_CMD_CARTSTIFF				0x1000


//  ---------------------- Doxygen info ----------------------
//! \def MASK_CMD_CARTDAMP
//!
//! \brief
//! Flag for Cartesian damping control values
//  ----------------------------------------------------------
#define MASK_CMD_CARTDAMP				0x2000


//  ---------------------- Doxygen info ----------------------
//! \def SIZE_COMMUNICATION_STATISTICS
//!
//! \brief
//! Size of the data structure FRICommunicationStatistics in bytes
//!
//! \sa FRICommunicationStatistics
//  ----------------------------------------------------------
#define SIZE_COMMUNICATION_STATISTICS	20


//  ---------------------- Doxygen info ----------------------
//! \def SIZE_COMMAND_DATA
//!
//! \brief
//! Size of the data structure FRICommandData in bytes
//!
//! \sa FRICommandData
//  ----------------------------------------------------------
#define SIZE_COMMAND_DATA				236


//  ---------------------- Doxygen info ----------------------
//! \def SIZE_KRL_DATA
//!
//! \brief
//! Size of the data structure FRIKRLData in bytes
//!
//! \sa FRIKRLData
//  ----------------------------------------------------------
#define SIZE_KRL_DATA					132

//  ---------------------- Doxygen info ----------------------
//! \def SIZE_HEADER
//!
//! \brief
//! Size of the data structure FRIHeader in bytes
//!
//! \sa FRIHeader
//  ----------------------------------------------------------
#define SIZE_HEADER						8


//  ---------------------- Doxygen info ----------------------
//! \def SIZE_ROBOT_STATE_DATA
//!
//! \brief
//! Size of the data structure FRIRobotStateData in bytes
//!
//! \sa FRIRobotStateData
//  ----------------------------------------------------------
#define SIZE_ROBOT_STATE_DATA			36


//  ---------------------- Doxygen info ----------------------
//! \def SIZE_FRI_STATE_DATA
//!
//! \brief
//! Size of the data structure FRIStateData in bytes
//!
//! \sa FRIStateData
//  ----------------------------------------------------------
#define SIZE_FRI_STATE_DATA				(20 + SIZE_COMMUNICATION_STATISTICS)


//  ---------------------- Doxygen info ----------------------
//! \def SIZE_MEASURED_ROBOT_DATA
//!
//! \brief
//! Size of the data structure FRIMeasuredRobotData in bytes
//!
//! \sa FRIMeasuredRobotData
//  ----------------------------------------------------------
#define SIZE_MEASURED_ROBOT_DATA		700


//  ---------------------- Doxygen info ----------------------
//! \def SIZE_DATA_RECEIVED_FROM_KRC
//!
//! \brief
//! Size of the data structure FRIDataReceivedFromKRC in bytes
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
#define SIZE_DATA_RECEIVED_FROM_KRC		(SIZE_HEADER + SIZE_KRL_DATA + SIZE_FRI_STATE_DATA + SIZE_ROBOT_STATE_DATA + SIZE_MEASURED_ROBOT_DATA)


//  ---------------------- Doxygen info ----------------------
//! \def SIZE_DATA_SENT_TO_KRC
//!
//! \brief
//! Size of the data structure FRIDataSendToKRC in bytes
//!
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
#define SIZE_DATA_SENT_TO_KRC			(SIZE_HEADER + SIZE_KRL_DATA + SIZE_COMMAND_DATA)


//  ---------------------- Doxygen info ----------------------
//! \enum FRIStateSet
//!
//! \brief
//! Enumeration of FRI states
//  ----------------------------------------------------------
enum FRIStateSet
{
	//! Internal off state (not related to the KRC state)
	FRI_STATE_OFF	=	0,
	//! Monitor mode
	FRI_STATE_MON	=	1,
	//! Command mode
	FRI_STATE_CMD	=	2
};


//  ---------------------- Doxygen info ----------------------
//! \enum FRICommunicationQualityLevelSet
//!
//! \brief
//! Enumeration of FRI communication quality levels (defined by KUKA)
//  ----------------------------------------------------------
enum FRICommunicationQualityLevelSet
{
	FRI_COMMUNICATION_QUALITY_UNACCEPTABLE	=	0,
	FRI_COMMUNICATION_QUALITY_BAD			=	1,
	FRI_COMMUNICATION_QUALITY_OK			=	2,
	FRI_COMMUNICATION_QUALITY_PERFECT		=	3 
};


//  ---------------------- Doxygen info ----------------------
//! \enum ControlModeSet
//!
//! \brief
//! Enumeration of available controllers
//  ----------------------------------------------------------
enum ControlModeSet
{
	//! Joint position control
	FRI_CONTROL_POSITION					=	1,
	//! Cartesian impedance control
	FRI_CONTROL_CART_IMP					=	2,
	//! Joint impedance control
	FRI_CONTROL_JNT_IMP						=	3
};


//  ---------------------- Doxygen info ----------------------
//! \struct FRICommunicationStatistics
//!
//! \brief
//! Data structure containing communication statistics (average rating
//! over the last 100 packets)
//  ----------------------------------------------------------
typedef struct
{
	//! Average rate of answered packages (hopefully close to one)
	float		AverageRateOfAnsweredPackages	;
	//! Average latency in seconds
	float		AverageLatencyInSeconds			;
	//! Average jitter in seconds
	float		AverageJitterInSeconds			;
	//! Average rate of missed packages (hopefully close to zero)
	float		AverageRateOfMissedPackages		;
	//! Absolute number of missed packages (zero if real-time constraints are met)
	uint32_t	AbsoluteNumberOfMissedPackages	;
} FRICommunicationStatistics;


//  ---------------------- Doxygen info ----------------------
//! \struct FRICommandData
//!
//! \brief
//! Data structure containing robot command data that is used
//! as input values for the selected controller (joint position,
//! joint impedance, Cartesian impedance)
//!
//! /sa ControlModeSet
//  ----------------------------------------------------------
typedef struct
{
	//! Flags to select the commanded values (depends on the control mode)
	uint32_t	FRIRobotCommandDataFlags;
	//! Commanded joint position vector in radians (MASK_CMD_JNTPOS)
	float		FRICommandedJointPositionVectorInRad				[NUMBER_OF_JOINTS			];
	//! Commanded Cartesian pose frame in meters (MASK_CMD_CARTPOS)
	float		FRICommandedCartesianFrame							[NUMBER_OF_FRAME_ELEMENTS	];
	//! Commanded additional joint torque vector in Nm (MASK_CMD_JNTTRQ)
	float		FRICommandedAdditionalJointTorqueVectorInNm			[NUMBER_OF_JOINTS			];
	//! Commanded additional Cartesian force/torque vector in N/Nm (MASK_CMD_TCPFT)
	float		FRICommandedAdditionalCartesianForceTorqueVector	[NUMBER_OF_CART_DOFS		];
	//! Commanded joint stiffness vector in Nm/rad (MASK_CMD_JNTSTIFF)
	float		FRICommandedJointStiffnessVectorInNmPerRad			[NUMBER_OF_JOINTS			];
	//! Commanded joint damping vector (normalized, MASK_CMD_JNTDAMP)
	float		FRICommandedNormalizedJointDampingVector			[NUMBER_OF_JOINTS			];
	//! Commanded Cartesian stiffness vector in N/m or Nm/rad, respectively (MASK_CMD_CARTSTIFF)
	float		FRICommandedCartesianStiffnessVector				[NUMBER_OF_CART_DOFS		];
	//! Commanded Cartesian damping vector (normalized, MASK_CMD_CARTDAMP)
	float		FRICommandedNormalizedCartesianDampingVector		[NUMBER_OF_CART_DOFS		];
} FRICommandData;


//  ---------------------- Doxygen info ----------------------
//! \struct FRIKRLData
//!
//! \brief
//! Data structure containing shared data between the remote PC and the
//! KRC. This data is available in both environments (FRI and KRL).
//  ----------------------------------------------------------
typedef struct
{
	//! KRC floating point values ($FRI_TO_REA[] and $FRI_FRM_REA[] in KRL)
	float		FRIFloatingPointValuesInKRC							[SIZE_USER_DATA				];
	//! KRC integer values ($FRI_TO_INT[] and $FRI_FRM_INT[] in KRL)
	int32_t		FRIIntegerValuesInKRC								[SIZE_USER_DATA				];
	//! KRC Boolean values ($FRI_TO_BOOL[] and $FRI_FRM_BOOL[] in KRL)
	uint16_t	FRIBoolValuesInKRC																;
	//! To achieve a multiple of four bytes
	uint16_t FRIFillData																		;
} FRIKRLData;

 
//  ---------------------- Doxygen info ----------------------
//! \struct FRIHeader
//!
//! \brief
//! Data structure containing the values of header of UDP
//! packages
//  ----------------------------------------------------------
typedef struct
{
	//! Sequence counter for UDP packages (incremented with each sent package by the sender)
	uint16_t	FRISequenceCounterForUDPPackages												;
	//! Reflected sequence counter for UDP packages (mirrored by this library)
	uint16_t	FRIReflectedSequenceCounterForUDPPackages										;
	//! Package size in bytes (cf. SIZE_DATA_SENT_TO_KRC)
	uint16_t	FRIPackageSizeInBytes															;
	//! Unique datagram ID (cf. FRI_DATAGRAM_ID_CMD)
	uint16_t	FRIDatagramID																	;
} FRIHeader;


//  ---------------------- Doxygen info ----------------------
//! \struct FRIStateData
//!
//! \brief
//! Data structure containing the values to describe the current state
//! of the Fast Research Interface
//  ----------------------------------------------------------
typedef struct
{
	//! Current system time in seconds (KRC side)
	float		FRITimeStamp																	;
	//! Current state of Fast Research Interface (monitor mode or command mode, cf. FRI_STATE)
	uint16_t	FRIState																		;
	//! Current communication quality (cf. FRI_COMMUNICATION_QUALITY)
	uint16_t	FRIQuality																		;
	//! Time period for sending packages from the KRC to the remote PC (set in KRL)
	float		FRISampleTimePeriodForDataSentFromKRC											;
	//! Time period for sending packages from the remote PC to the KRC unit
	float		FRISampleTimePeriodForDataSentToKRC												;
	//! Safety time limits
	float		FRICommunicationTimeLimitsForSafety												;
	//! Current FRI communication statistics
	FRICommunicationStatistics FRIStatistics													;
} FRIStateData;


//  ---------------------- Doxygen info ----------------------
//! \struct FRIRobotStateData
//!
//! \brief
//! Data structure containing values of the robot arm state
//  ----------------------------------------------------------
typedef struct
{
	//! Robot power status
	uint16_t	FRIRobotPower																	;
	//! Current control mode (cf. ControlModeSet)
	uint16_t	FRIRobotControl																	;
	//! Drive error status
	uint16_t	FRIDriveError																	;
	//! Drive warning status
	uint16_t	FRIDriveWarning																	;
	//! Drive temperature vector
	float		FRIDriveTemperature									[NUMBER_OF_JOINTS			];
} FRIRobotStateData;


//  ---------------------- Doxygen info ----------------------
//! \struct FRIMeasuredRobotData
//!
//! \brief
//! Data structure containing all measured and estimated values
//! from the KRC unit
//  ----------------------------------------------------------
typedef struct
{
	//! Measured joint position vector in radians
	float		FRIMeasuredJointPositionVectorInRad					[NUMBER_OF_JOINTS			];
	//! Measured Cartesian frame in m
	float		FRIMeasuredCartesianFrame							[NUMBER_OF_FRAME_ELEMENTS	];
	//! Commanded joint position vector in radians (provided by the KRC)
	float		FRICommandedJointPostionVectorFromKRC				[NUMBER_OF_JOINTS			];
	//! Commanded joint position offset vector in radians (provided by the KRC)
	float		FRICommandedJointPostionOffsetVectorFromKRC			[NUMBER_OF_JOINTS			];
	 //! Commanded Cartesian frame in m (provided by the KRC)_
	float		FRICommandedCartesianFrameFromKRC					[NUMBER_OF_FRAME_ELEMENTS	];
	//! Commanded Cartesian offset frame in m (provided by the KRC)_
	float		FRICommandedCartesianFrameOffsetFromKRC				[NUMBER_OF_FRAME_ELEMENTS	];
	//! Measured joint torque vector in Nm
	float		FRIMeasuredJointTorqueVectorInNm					[NUMBER_OF_JOINTS			];
	//! Estimated joint torque vector in Nm caused by external forces
	float		FRIEstimatedExternalJointTorqueVectorInNm			[NUMBER_OF_JOINTS			];
	//! Estimated Cartesian force/torque vector caused by external forces (in N or Nm, respectively)
	float		FRIEstimatedCartesianForcesAndTorques				[NUMBER_OF_CART_DOFS		];
	//! Jacobian matrix  
	float		FRIJacobianMatrix						[NUMBER_OF_CART_DOFS * NUMBER_OF_JOINTS	];
	//! Mass matrix
	float		FRIMassMatrix							[NUMBER_OF_JOINTS * NUMBER_OF_JOINTS	];
	//! Contribution of the gravitational forces to the commanded joint torque vector
	float		FRIGravityVectorInJointSpace						[NUMBER_OF_JOINTS			];
} FRIMeasuredRobotData;


//  ---------------------- Doxygen info ----------------------
//! \struct FRIDataReceivedFromKRC
//!
//! \brief
//! Data structure containing all data structures that are contained in one
//! data package received from the KRC unit
//  ----------------------------------------------------------
typedef struct
{
	//! FRI communication header
	FRIHeader				Header;
	//! Variables received from KRL
	FRIKRLData				SharedKRLVariables;
	//! FRI state
	FRIStateData			InterfaceState;
	//! Robot state
	FRIRobotStateData		Robot;	
	//! Measured data
	FRIMeasuredRobotData	MeasuredData;
} FRIDataReceivedFromKRC;


//  ---------------------- Doxygen info ----------------------
//! \struct FRIDataSendToKRC
//!
//! \brief
//! Data structure containing all data structures that are contained in one
//! data package sent to the KRC unit
//  ----------------------------------------------------------
typedef struct
{
	//! FRI communication header
	FRIHeader				Header;
	//! Variables sent to KRL
	FRIKRLData				SharedKRLVariables;
	//! Robot command values
	FRICommandData			CommandValues;
} FRIDataSendToKRC;


//  ---------------------- Doxygen info ----------------------
//! \def ALL_DATA_SIZES_SENT_TO_KRC_ARE_OK
//!
//! \brief
//! Macro to check, whether all data size are correct
//  ----------------------------------------------------------
#define ALL_DATA_SIZES_SENT_TO_KRC_ARE_OK	((sizeof(FRIRobotStateData			)	==	SIZE_ROBOT_STATE_DATA			)	&& \
											(sizeof(FRIHeader					)	==	SIZE_HEADER						)	&& \
											(sizeof(FRIKRLData					)	==	SIZE_KRL_DATA					)	&& \
											(sizeof(FRIDataReceivedFromKRC		)	==	SIZE_DATA_RECEIVED_FROM_KRC		)	&& \
											(sizeof(FRIDataSendToKRC			)	==	SIZE_DATA_SENT_TO_KRC			)	&& \
											(sizeof(FRICommunicationStatistics	)	==	SIZE_COMMUNICATION_STATISTICS	)	&& \
											(sizeof(FRIStateData				)	==	SIZE_FRI_STATE_DATA				)	&& \
											(sizeof(FRICommandData				)	==	SIZE_COMMAND_DATA				)	&& \
											(sizeof(FRIMeasuredRobotData		)	==	SIZE_MEASURED_ROBOT_DATA		))



#endif
