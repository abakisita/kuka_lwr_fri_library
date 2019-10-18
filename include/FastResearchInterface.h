//  ---------------------- Doxygen info ----------------------
//! \file FastResearchInterface.h
//!
//! \brief
//! Header file for the class FastResearchInterface
//!
//! \details
//! The class FastResearchInterface provides a basic low-level interface
//! to the KUKA Light-Weight Robot IV.
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


#ifndef __FastResearchInterface__
#define __FastResearchInterface__


#include <Console.h>
#include <DataLogging.h>
#include <FRICommunication.h>
#include <pthread.h>


//  ---------------------- Doxygen info ----------------------
//! \class FastResearchInterface
//!
//! \brief
//! Provides easy access to \em all functionalities of
//! Fast Research Interface of the KUKA Light-Weight Robot IV
//!
//! \details
//! This is the main class of the Fast Research Interface Library for the KUKA Light-Weight Robot IV.
//! An object of this class provides the following functionalities:\n
//! <ul>
//! <li>Setting basic attributes of the interface by using the
//! initialization file of the Fast Research Interface Library (cf. \ref sec_InitFile).
//! <li>Creating a communication thread that is dedicated for all UDP communication procedures to exchange data between the
//! KRC unit and the remote host.
//! <li>Creating a timer thread that periodically send signal through a condition variable
//! <li>As soon as the communication thread cyclically receives data telegrams from the KRC unit:
//! <ul>
//! <li>Providing robot status information (e.g., FRIDriveTemperature, arm power state, errors, warnings, etc.)
//! <li>Providing robot control data (e.g., measured joint position vector, measured joint torque vector, etc.)
//! <li>Providing shared KRL variables
//! <li>Providing communication status and quality data (e.g., jitter, latency, etc.; measured by the KRC host)
//! <li>Setting robot control data (e.g., commanded joint positions, commanded joint torques, etc.)
//! <li>Setting shared KRL variables
//! <li>Performing the start-up procedure of the robot
//! <li>Performing the shutdown procedure of the robot
//! <li>Enabling low-level real-time data logging methods
//! <li>Providing two different timer signals (KRC timer and local timer)
//! <li>A real-time capable method for console output
//! </ul>
//! </ul>
//! Users may use this class as the interface to the KRC unit if the complete functionality is required for the desired application.
//! In order to provide a simple and easy-to-start-with interface, this class is used by (but not derived to) the classes\n
//! <ul>
//! <li>LWRJointPositionController for the joint position controller,
//! <li>LWRCartImpedanceController for the Cartesian impedance controller,
//! <li>LWRJointImpedanceController for the joint impedance controller, and
//! <li>LWRJointTorqueController for the joint torque controller,\n
//! </ul>
//! which are all derived from the class LWRBaseControllerInterface.
//!
//! \attention
//! Only \em one object of this class is supposed to be created on a remote host. Otherwise
//! \em indeterministic behavior may occur.
//!
//! \remark
//! For a general description of the Fast Research Interface Library, please refer to the \ref sec_Introduction.
//!
//! \sa LWRBaseControllerInterface
//! \sa LWRJointPositionController
//! \sa LWRCartImpedanceController
//! \sa LWRJointImpedanceController
//! \sa LWRJointTorqueController
//!
//  ----------------------------------------------------------
class FastResearchInterface
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn FastResearchInterface(const char *InitFileName)
//!
//! \brief
//! Constructor
//!
//! \details
//! This constructor performs several procedures. It\n
//! <ul>
//! <li> initializes all class attributes,
//! <li> reads all parameters of the initialization file specified by \c InitFileName using the method
//! FastResearchInterface::ReadInitFile(), which further on uses an object of the class InitializationFileEntry,
//! <li> sets the priority, which is specified in the initialization file to the calling thread,
//! <li> creates the timer thread FastResearchInterface::TimerThreadMain() running at the
//! priority, which is specified in the initialization file,
//! <li> creates the communication thread FastResearchInterface::KRCCommunicationThreadMain() running at the
//! priority, which is specified in the initialization file,
//! <li> creates the Console object FastResearchInterface::OutputConsole, which provides the possibility of using
//! Console::printf() in real-time parts of the object, and
//! <li> creates the DataLogging object FastResearchInterface::DataLogger, which provides the possibility of logging
//! low-level control data under real-time conditions.\n\n\n
//!	</ul>
//! In case,
//! <ul>
//! <li> the initialization file specified by \c InitFileName cannot be opened,
//! <li> the initialization file specified by \c InitFileName does not contain all required parameters (cf. \ref sec_InitFile), or
//! <li> one of the threads FastResearchInterface::KRCCommunicationThreadMain() or FastResearchInterface::TimerThreadMain()
//! cannot be created successfully
//!	</ul>
//! the constructor lets the calling \em process terminate and exits with a value of \c EXIT_FAILURE.
//!
//! \param InitFileName
//! A pointer to an array of \c char values containing the path and filename of the
//! initialization file (e.g., <tt>\"/home/lwrcontrol/etc/980039-FRI-Driver.init\"</tt>).
//! For details about this file, please refer to \ref sec_InitFile.
//!
//! \note All threads of this class, that is,\n\n
//! <ul>
//! <li> the timer thread FastResearchInterface::TimerThreadMain(),
//! <li> the communication thread FastResearchInterface::KRCCommunicationThreadMain(),
//! <li> the output thread of the class Console, Console::ConsoleThreadMain(), and
//! <li> the calling thread\n\n
//! </ul>
//! use the Fifo scheduling policy (<tt>SCHED_FIFO</tt>).
//!
//! \attention
//! The call of the constructor does \b not fulfill any real-time requirements.
//  ----------------------------------------------------------
	FastResearchInterface(const char *InitFileName);


//  ---------------------- Doxygen info ----------------------
//! \fn ~FastResearchInterface(void)
//!
//! \brief
//! Destructor
//!
//! \details
//! This destructor performs a set of procedures to cleanly shutdown the robot:\n
//! <ul>
//! <li> The communication thread FastResearchInterface::KRCCommunicationThreadMain()
//! is terminated and joins the calling thread. If it still receives data packages from
//! the KRC unit, it will be checked whether the
//! robot has to be shutdown, too (cf. FastResearchInterface::StopRobot()).
//! If the thread does not receive any UDP data telegrams anymore, a
//! signal (<tt>SIGTERM</tt>) will be sent to the thread FastResearchInterface::KRCCommunicationThread.
//! <li> The timer thread FastResearchInterface::TimerThreadMain() is terminated and joins the calling thread.
//! <li>If the data logger still has data in its memory that has not been written to a log file yet,
//! this file will be written.
//! <li> The DataLogging object FastResearchInterface::DataLogger is deleted.
//! <li> The Console object FastResearchInterface::OutputConsole is deleted.\n\n
//! </ul>
//!
//! \attention
//! The call of the destructor does \b not fulfill any real-time requirements.
//  ----------------------------------------------------------
	~FastResearchInterface(void);


//  ---------------------- Doxygen info ----------------------
//! \enum LWRControlModes
//!
//! \brief Available control schemes for the KUKA Light-Weight Robot IV
//!
//! \details
//! Depending the parameter \c ControlMode of the method FastResearchInterface::StartRobot(), the KRL program
//! (cf.\ref sec_KRLFile1) sets the global variable <tt>\$stiffness.strategy</tt> and calls the KRL function
//! <tt>friStart()</tt>.
//  ----------------------------------------------------------
	enum LWRControlModes
	{
		//! \brief Joint position control
		JOINT_POSITION_CONTROL	=	10,
		//! \brief Cartesian impedance control
		CART_IMPEDANCE_CONTROL	=	20,
		//! \brief Joint impedance control
		JOINT_IMPEDANCE_CONTROL	=	30,
		//! \brief Joint torque control
		JOINT_TORQUE_CONTROL	=	90
	};


//  ---------------------- Doxygen info ----------------------
//! \fn int StartRobot(const unsigned int &ControlMode, const float &TimeOutValueInSeconds = 120.0)
//!
//! \brief
//! Starts the robot
//!
//! \details
//! This method performs a complete start-up procedure of the robot arm. The following steps are executed:\n\n
//! <ol>
//! <li> Before the KRL program \c FRIControl (cf.\ref sec_KRLFile1) is started by the user, the constructor
//! FastResearchInterface::FastResearchInterface::() should be called in order to prevent any loss of UDP packages.
//! <li> If the KRC unit is not already in <em>Monitor Mode</em> (i.e., no communication between the remote
//! host and the KRC unit is performed), the method waits for the call of the KRL function <tt>friOpen()</tt>
//! to receive UDP messages in order to let the KRC unit switch to <em>Monitor Mode</em>.
//! <li> Once, the KRC unit runs in <em>Monitor Mode</em>, the method sets the KRL variable <tt>$FRI_FRM_INT[16]</tt>
//! to a value of \c 10, which lets the KRL program \c FRIControl call the function <tt>friStart()</tt> in order to
//! switch from <em>Monitor Mode</em> to <em>Command Mode</em>.
//! <li> In the transition phase, the following command variables are sent from the remote host to the KRC unit
//! (see also: FastResearchInterface::SetControlScheme()):\n\n
//! <ul>
//! <li> Joint position control
//! <ul>
//! <li> Desired joint position vector = Current joint position vector
//! </ul>\n
//! <li> Cartesian impedance control
//! <ul>
//! <li> Desired Cartesian pose frame = Last commanded Cartesian pose frame
//! <li> Desired Cartesian pose offset frame = Identity frame
//! <li> Desired additional force/torque vector = Zero vector
//! <li> The desired Cartesian stiffness and damping values remain unchanged and may be specified by the user
//! by using the methods FastResearchInterface::SetCommandedCartStiffness() and
//! FastResearchInterface::SetCommandedCartDamping(), if required.
//! </ul>\n
//! <li> Joint impedance control
//! <ul>
//! <li> Desired joint position vector = Current joint position vector
//! <li> Desired joint position offset vector = Zero vector
//! <li> Desired additional joint torque vector = Zero vector
//! <li> The desired joint stiffness and damping values remain unchanged and may be specified by the user
//! by using the methods FastResearchInterface::SetCommandedJointStiffness() and
//! FastResearchInterface::SetCommandedJointDamping(), if required.
//! </ul>\n
//! <li> Joint torque control:
//! <ul>
//! <li> Desired joint torque vector = Zero vector
//! </ul>
//! </ul>\n
//! <li> After the KRC unit switched to <em>Command Mode</em>, the method waits until the robot is ready for
//! operation, that is, until the result value of FastResearchInterface::IsMachineOK() is \c true.
//! </ol>\n
//! The complete start-up procedure must not take longer than \c TimeOutValueInSeconds seconds. If completed successfully,
//! the robot arm is ready for operation. This method should be used pairwise with the method
//! FastResearchInterface::StopRobot() in order to ensure a safe and error-free operation.
//!
//! \param ControlMode
//! Either value of the enumeration FastResearchInterface::LWRControlModes:\n
//! <ul>
//! <li> FastResearchInterface::JOINT_POSITION_CONTROL for joint position control
//! <li> FastResearchInterface::CART_IMPEDANCE_CONTROL for Cartesian impedance control
//! <li> FastResearchInterface::JOINT_IMPEDANCE_CONTROL	for joint impedance control
//! <li> FastResearchInterface::JOINT_TORQUE_CONTROL for joint torque control\n
//! </ul>
//! If this method is called by an object derived from the class LWRBaseControllerInterface, this parameter is
//! automatically set by their respective method for starting the robot.\n\n
//!
//! \param TimeOutValueInSeconds
//! Timeout value in seconds for the user to turn on the robot arm by using the KUKA Control Panel.
//! The default value of this optional parameter is 120 seconds.
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit could be established. Please check
//! <ul>
//! <li> whether the KRC unit is turned on and the Ethernet cable (e.g., a crossed cable) is plugged in on both nodes,
//! <li> whether the KRL program \c FRIControl (cf.\ref sec_KRLFile1) has been started, or
//! <li> whether the KRC node is configured correctly in particular w.r.t. the network settings (cf. \ref sec_Introduction).
//! </ul>
//! <li> \c EALREADY if the KRC unit is already in command mode, that is, the KRL function <tt>friStart()</tt> was already called
//! by a previous call of FastResearchInterface::StartRobot(). If the robot is not ready for operation (i.e., the result of
//! FastResearchInterface::IsMachineOK() is \c false), call FastResearchInterface::StopRobot() first. It might be
//! required to check whether the KRC unit is running correctly, and whether the KRL program
//! KRL program \c FRIControl (cf.\ref sec_KRLFile1) is being executed.
//! <li> \c ETIME if the start-up procedure could not be completed within the specified time interval of \c TimeOutValueInSeconds
//! <li> \c EOK if no error occurred.
//! </ul>
//!
//! \sa FastResearchInterface::StopRobot()
//  ----------------------------------------------------------
	int StartRobot(		const unsigned int &ControlMode
				   	,	const float &TimeOutValueInSeconds = 120.0);


//  ---------------------- Doxygen info ----------------------
//! \fn int StopRobot(void)
//!
//! \brief
//! Stops the robot
//!
//! \details
//! By calling this method, a complete shutdown procedure of the robot arm is performed.
//! No matter whether the KRC unit runs in <em>Monitor Mode</em> or <em>Command Mode</em>, the KRL variable
//! <tt>$FRI_FRM_INT[16]</tt> is set to a value of \c 20, which lets the KRL program \c FRIControl
//! (cf.\ref sec_KRLFile1) call the function <tt>friStop()</tt> in order to
//! switch to <em>Monitor Mode</em>.
//! After the KRC unit switched to <em>Monitor Mode</em>, the method returns.
//!
//! \return
//! <ul>
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit could be established. Please check
//! <ul>
//! <li> whether the KRC unit is turned on and the Ethernet cable (e.g., a crossed cable) is plugged in on both nodes,
//! <li> whether the KRL program \c FRIControl (cf.\ref sec_KRLFile1) has been started, or
//! <li> whether the KRC node is configured correctly in particular w.r.t. the network settings (cf. \ref sec_Introduction).
//! </ul>
//! <li> \c EOK if no error occurred.
//! </ul>
//!
//! \sa FastResearchInterface::StartRobot()
//!
//! \attention
//! <ul>
//! <li>The call of this method does \b not fulfill any real-time requirements.
//! <li><b>After calling the KRL function <tt>friStop()</tt>, the robot arm power may be still turned on. In the cases of
//! <ul>
//! <li>FastResearchInterface::CART_IMPEDANCE_CONTROL,
//! <li>FastResearchInterface::JOINT_IMPEDANCE_CONTROL, or
//! <li>FastResearchInterface::JOINT_TORQUE_CONTROL,
//! </ul>
//! this may lead to an uncontrolled and/or undesired behavior of the robot arm.</b>
//! </ul>
//  ----------------------------------------------------------
	int StopRobot(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetMeasuredJointPositions(float *MeasuredJointPositions)
//!
//! \brief
//! Reads the measured joint position vector from the latest data telegram of the KRC unit
//!
//! \param MeasuredJointPositions
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The measured joint position vector is written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetMeasuredJointPositions(float *MeasuredJointPositions);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetCommandedJointPositions(float *CommandedJointPositions)
//!
//! \brief
//! Reads the commanded joint position vector from the latest data telegram of the KRC unit
//!
//! \param CommandedJointPositions
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The commanded joint position vector is written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetCommandedJointPositions(float *CommandedJointPositions);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetCommandedJointPositionOffsets(float *CommandedJointPositionOffsets)
//!
//! \brief
//! Reads the commanded joint position offset vector from the latest data telegram of the KRC unit
//!
//! \param CommandedJointPositionOffsets
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The commanded joint position offset is written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetCommandedJointPositionOffsets(float *CommandedJointPositionOffsets);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetMeasuredJointTorques(float *MeasuredJointTorques)
//!
//! \brief
//! Reads the measured joint torque vector from the latest data telegram of the KRC unit
//!
//! \param MeasuredJointTorques
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The measured joint torque vector is written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetMeasuredJointTorques(float *MeasuredJointTorques);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetEstimatedExternalJointTorques(float *EstimatedExternalJointTorques)
//!
//! \brief
//! Reads the estimated external joint torque vector from the latest data telegram of the KRC unit
//!
//! \param EstimatedExternalJointTorques
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The estimated external joint torque vector is written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetEstimatedExternalJointTorques(float *EstimatedExternalJointTorques);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetMeasuredCartPose(float *MeasuredCartPose)
//!
//! \brief
//! Reads the measured Cartesian pose frame from the latest data telegram of the KRC unit
//!
//! \param MeasuredCartPose
//! A pointer to an array of \c float values; the array has to have at least a size of twelve elements.
//! The measured Cartesian pose frame is written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetMeasuredCartPose(float *MeasuredCartPose);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetCommandedCartPose(float *CommandedCartPose)
//!
//! \brief
//! Reads the commanded Cartesian pose frame from the latest data telegram of the KRC unit
//!
//! \param CommandedCartPose
//! A pointer to an array of \c float values; the array has to have at least a size of twelve elements.
//! The commanded Cartesian pose frame is written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetCommandedCartPose(float *CommandedCartPose);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetCommandedCartPoseOffsets(float *CommandedCartPoseOffsets)
//!
//! \brief
//! Reads the commanded Cartesian pose offset frame from the latest data telegram of the KRC unit
//!
//! \param CommandedCartPoseOffsets
//! A pointer to an array of \c float values; the array has to have at least a size of twelve elements.
//! The commanded Cartesian pose offset frame is written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetCommandedCartPoseOffsets(float *CommandedCartPoseOffsets);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques)
//!
//! \brief
//! Reads the estimated external force and torque values w.r.t. the tool frame from the latest data telegram of the KRC unit
//!
//! \param EstimatedExternalCartForcesAndTorques
//! A pointer to an array of \c float values; the array has to have at least a size of six elements.
//! The estimated external force and torque values are written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCommandedJointPositions(const float *CommandedJointPositions)
//!
//! \brief
//! Copies the desired joint position vector into the data telegram to be send to the KRC unit
//!
//! \param CommandedJointPositions
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The desired joint position vector (given in radians) is copied from this array.
//!
//! \note
//! This value will only be relevant if the joint position or the joint impedance controller is active.
//!
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
	void SetCommandedJointPositions(const float *CommandedJointPositions);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCommandedJointTorques(const float *CommandedJointTorques)
//!
//! \brief
//! Copies the desired joint torque vector into the data telegram to be send to the KRC unit
//!
//! \param CommandedJointTorques
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The desired joint torque vector (given in Nm) is copied from this array.
//!
//! \note
//! This value will only be relevant if the joint impedance or the joint torque controller is active.
//!
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
	void SetCommandedJointTorques(const float *CommandedJointTorques);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCommandedJointStiffness(const float *CommandedJointStiffness)
//!
//! \brief
//! Copies the desired joint stiffness vector into the data telegram to be send to the KRC unit
//!
//! \param CommandedJointStiffness
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The desired joint stiffness vector is copied from this array.
//!
//! \note
//! This value will only be relevant if the joint impedance controller is active.
//!
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
	void SetCommandedJointStiffness(const float *CommandedJointStiffness);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCommandedJointDamping(const float *CommandedJointDamping)
//!
//! \brief
//! Copies the desired joint damping vector into the data telegram to be send to the KRC unit
//!
//! \param CommandedJointDamping
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The desired joint damping vector is copied from this array.
//!
//! \note
//! This value will only be relevant if the joint impedance controller is active.
//!
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
	void SetCommandedJointDamping(const float *CommandedJointDamping);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCommandedCartPose(const float *CommandedCartPose)
//!
//! \brief
//! Copies the desired Cartesian pose frame into the data telegram to be send to the KRC unit
//!
//! \param CommandedCartPose
//! A pointer to an array of \c float values; the array has to have at least a size of twelve elements.
//! The desired Cartesian pose frame is copied from this array.
//!
//! \note
//! This value will only be relevant if the Cartesian impedance controller is active.
//!
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
	void SetCommandedCartPose(const float *CommandedCartPose);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques)
//!
//! \brief
//! Copies the desired Cartesian force/torque vector into the data telegram to be send to the KRC unit
//!
//! \param CartForcesAndTorques
//! A pointer to an array of \c float values; the array has to have at least a size of six elements.
//! The desired Cartesian force/torque vector is copied from this array.
//!
//! \note
//! This value will only be relevant if the Cartesian impedance controller is active.
//!
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
	void SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCommandedCartStiffness(const float *CommandedCartStiffness)
//!
//! \brief
//! Copies the desired Cartesian stiffness vector into the data telegram to be send to the KRC unit
//!
//! \param CommandedCartStiffness
//! A pointer to an array of \c float values; the array has to have at least a size of six elements.
//! The desired Cartesian stiffness vector is copied from this array.
//!
//! \note
//! This value will only be relevant if the Cartesian impedance controller is active.
//!
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
	void SetCommandedCartStiffness(const float *CommandedCartStiffness);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetCommandedCartDamping(const float *CommandedCartDamping)
//!
//! \brief
//! Copies the desired Cartesian damping vector into the data telegram to be send to the KRC unit
//!
//! \param CommandedCartDamping
//! A pointer to an array of \c float values; the array has to have at least a size of six elements.
//! The desired Cartesian damping vector is copied from this array.
//!
//! \note
//! This value will only be relevant if the Cartesian impedance controller is active.
//!
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
	void SetCommandedCartDamping(const float *CommandedCartDamping);


//  ---------------------- Doxygen info ----------------------
//! \fn unsigned int GetFRIMode(void)
//! (read from the latest data telegram of the KRC unit)
//!
//! \brief
//! Returns the current mode of the Fast Research Interface Running on the KRC unit
//!
//! \return
//!  - <tt>FRI_STATE_OFF</tt> <em>off</em> (i.e., no UDP connection has been established yet)
//!  - <tt>FRI_STATE_MON</tt> <em>monitor mode</em>
//!  - <tt>FRI_STATE_CMD</tt> <em>command mode</em>
//!
//! \sa FRICommunication.h
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	unsigned int GetFRIMode(void);


//  ---------------------- Doxygen info ----------------------
//! \fn unsigned int GetCurrentControlScheme(void)
//! (read from the latest data telegram of the KRC unit)
//!
//! \brief
//! Returns the current control scheme of the Fast Research Interface Running on the KRC unit
//!
//! \return
//!  - <tt>FastResearchInterface::JOINT_POSITION_CONTROL</tt> if the joint position controller is currently active
//!  - <tt>FastResearchInterface::CART_IMPEDANCE_CONTROL</tt> if the Cartesian impedance controller is currently active
//!  - <tt>FastResearchInterface::JOINT_IMPEDANCE_CONTROL</tt> if the joint impedance controller is currently active
//!  - <tt>FastResearchInterface::JOINT_TORQUE_CONTROL</tt> if the joint torque controller is currently active
//!
//! \sa FRICommunication.h
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	unsigned int GetCurrentControlScheme(void);


//  ---------------------- Doxygen info ----------------------
//! \fn bool IsRobotArmPowerOn(void)
//!
//! \brief
//! Returns a Boolean value indicating whether the arm power is turned on and the brakes are released
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//!  - \c true if the arm power is on and the brakes are released
//!  - \c false if the arm power is off and the brakes are engaged
//!
//! \sa FRICommunication.h
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	bool IsRobotArmPowerOn(void);


//  ---------------------- Doxygen info ----------------------
//! \fn bool DoesAnyDriveSignalAnError(void)
//!
//! \brief
//! Returns a Boolean value indicating whether \em any drive signals an error
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//!  - \c true if one are more drives signal an error
//!  - \c false otherwise
//!
//! \sa FRICommunication.h
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	bool DoesAnyDriveSignalAnError(void);


//  ---------------------- Doxygen info ----------------------
//! \fn bool DoesAnyDriveSignalAWarning(void)
//!
//! \brief
//! Returns a Boolean value indicating whether \em any drive signals a warning
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//!  - \c true if one are more drives signal a warning
//!  - \c false otherwise
//!
//! \sa FRICommunication.h
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	bool DoesAnyDriveSignalAWarning(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetDriveTemperatures(float *Temperatures)
//!
//! \brief
//! Reads the measured drive FRIDriveTemperatures for each drive from the latest data telegram of the KRC unit
//!
//! \param Temperatures
//! A pointer to an array of \c float values; the array has to have at least a size of seven elements.
//! The measured drive FRIDriveTemperatures for each drive is written into this array.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetDriveTemperatures(float *Temperatures);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetCurrentJacobianMatrix(float **JacobianMatrix)
//!
//! \brief
//! Reads current Jacobian from the latest data telegram of the KRC unit
//!
//! \param JacobianMatrix
//! A pointer to an two-dimensional array of \c float values the array has to have at least a size of 42 elements.
//! The current Jacobian matrix is written into this array.
//!
//! \warning
//! This method has never been tested.
//!
//! \todo
//! Test this method!
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetCurrentJacobianMatrix(float **JacobianMatrix);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetCurrentMassMatrix(float **MassMatrix)
//!
//! \brief
//! Reads current mass matrix from the latest data telegram of the KRC unit
//!
//! \param MassMatrix
//! A pointer to an two-dimensional array of \c float values the array has to have at least a size of 49 elements.
//! The current mass matrix is written into this array.
//!
//! \warning
//! This method has never been tested.
//!
//! \todo
//! Test this method!
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetCurrentMassMatrix(float **MassMatrix);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetCurrentGravityVector(float *GravityVector)
//!
//! \brief
//! Reads current gravity vector (transformed into joint space) from the latest data telegram of the KRC unit
//!
//! \param GravityVector
//! A pointer to an array of \c float values. The current gravity vector 
//! in joint space is written into this array.
//!
//! \warning
//! This method has never been tested.
//!
//! \todo
//! Test this method!
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	void GetCurrentGravityVector(float *GravityVector);


//  ---------------------- Doxygen info ----------------------
//! \fn float GetFRICycleTime(void)
//!
//! \brief
//! Returns the communication cycle time between the KRC unit and the remote host
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//! Communication cycle time in seconds.
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	float GetFRICycleTime(void);


//  ---------------------- Doxygen info ----------------------
//! \fn int GetCommunicationTimingQuality(void)
//!
//! \brief
//! Returns the current communication quality measured by the KRC unit
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//!  - <tt>FRI_COMMUNICATION_QUALITY_UNACCEPTABLE</tt>
//!  - <tt>FRI_COMMUNICATION_QUALITY_BAD</tt>
//!  - <tt>FRI_COMMUNICATION_QUALITY_OK</tt>
//!  - <tt>FRI_COMMUNICATION_QUALITY_PERFECT</tt>
//!
//! \sa FRICommunication.h
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	int GetCommunicationTimingQuality(void);


//  ---------------------- Doxygen info ----------------------
//! \fn float GetUDPAverageRateOfAnsweredPackages(void)
//!
//! \brief
//! Returns the current answer rate of the remote host measured by the KRC unit
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//! Answer rate
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	float GetUDPAverageRateOfAnsweredPackages(void);


//  ---------------------- Doxygen info ----------------------
//! \fn float GetUDPAverageLatencyInSeconds(void)
//!
//! \brief
//! Returns the current communication latency measured by the KRC unit
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//! Current communication latency in seconds
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	float GetUDPAverageLatencyInSeconds(void);


//  ---------------------- Doxygen info ----------------------
//! \fn float GetUDPAverageJitterInSeconds(void)
//!
//! \brief
//! Returns the current communication jitter measured by the KRC unit
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//! Current communication jitter in seconds
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	float GetUDPAverageJitterInSeconds(void);


//  ---------------------- Doxygen info ----------------------
//! \fn float GetUDPPackageLossRate(void)
//!
//! \brief
//! Returns the current data package loss rate measured by the KRC unit
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//! Current data package loss rate
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	float GetUDPPackageLossRate(void);


//  ---------------------- Doxygen info ----------------------
//! \fn unsigned int GetNumberOfMissedUDPPackages(void)
//!
//! \brief
//! Returns the number of lost data packages measured by the KRC unit
//! (read from the latest data telegram of the KRC unit)
//!
//! \return
//! Number of lost data packages
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	unsigned int GetNumberOfMissedUDPPackages(void);


//  ---------------------- Doxygen info ----------------------
//! \fn unsigned int GetValueOfKRCSequenceCounter(void)
//!
//! \brief
//! Returns the current value of the data telegram sequence counter of the KRC unit
//! (read from the latest data telegram of the KRC unit)
//!
//! \note
//! This value may be affected by overflows, if the system runs for more than 49 days without interruption.
//!
//! \return
//! Current value of the data telegram sequence counter
//!
//! \sa FRIDataReceivedFromKRC
//  ----------------------------------------------------------
	unsigned int GetValueOfKRCSequenceCounter(void);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetKRLBoolValues(bool *KRLBoolValues)
//!
//! \brief
//! Gets the current value of <tt>\$FRI_TO_BOOL[]</tt> as set by the KRL program
//! (read from the latest data telegram of the KRC unit)
//!
//! \param KRLBoolValues
//! A pointer to an array of \c bool values; the array has to have at least a size of 16 elements.
//! The Boolean values set by the KRL program running on the KRC unit are copied into this array.
//!
//! \warning
//! This method has never been tested.
//!
//! \todo
//! Test this method!
//!
//! \sa FRIDataReceivedFromKRC
//! \sa FastResearchInterface::GetKRLBoolValue()
//! \sa FastResearchInterface::SetKRLBoolValues()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	void GetKRLBoolValues(bool *KRLBoolValues);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetKRLIntValues(int *KRLIntValues)
//!
//! \brief
//! Gets the current value of <tt>\$FRI_TO_INT[]</tt> as set by the KRL program
//! (read from the latest data telegram of the KRC unit)
//!
//! \param KRLIntValues
//! A pointer to an array of \c int values; the array has to have at least a size of 16 elements.
//! The integer values set by the KRL program running on the KRC unit are copied into this array.
//!
//! \sa FRIDataReceivedFromKRC
//! \sa FastResearchInterface::GetKRLIntValue()
//! \sa FastResearchInterface::SetKRLIntValues()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	void GetKRLIntValues(int *KRLIntValues);


//  ---------------------- Doxygen info ----------------------
//! \fn void GetKRLFloatValues(float *KRLFloatValues)
//!
//! \brief
//! Gets the current value of <tt>\$FRI_TO_REAL[]</tt> as set by the KRL program
//! (read from the latest data telegram of the KRC unit)
//!
//! \param KRLFloatValues
//! A pointer to an array of \c float values; the array has to have at least a size of 16 elements.
//! The floating point values set by the KRL program running on the KRC unit are copied into this array.
//!
//! \sa FRIDataReceivedFromKRC
//! \sa FastResearchInterface::GetKRLFloatValue()
//! \sa FastResearchInterface::SetKRLFloatValues()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	void GetKRLFloatValues(float *KRLFloatValues);


//  ---------------------- Doxygen info ----------------------
//! \fn bool GetKRLBoolValue(const unsigned int &Index)
//!
//! \brief
//! Returns one single element of the KRL array <tt>\$FRI_TO_BOOL[]</tt>
//! (read from the latest data telegram of the KRC unit)
//!
//! \param Index
//! The index of the desired Boolean value; this value has to be in range 0f <tt>0,...,15</tt>
//!
//! \return
//! The desired \c bool value at \c Index
//!
//! \warning
//! This method has never been tested.
//!
//! \todo
//! Test this method!
//!
//! \sa FRIDataReceivedFromKRC
//! \sa FastResearchInterface::GetKRLBoolValues()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	bool GetKRLBoolValue(const unsigned int &Index);


//  ---------------------- Doxygen info ----------------------
//! \fn bool GetKRLIntValue(const unsigned int &Index)
//!
//! \brief
//! Returns one single element of the KRL array <tt>\$FRI_TO_INT[]</tt>
//! (read from the latest data telegram of the KRC unit)
//!
//! \param Index
//! The index of the desired integer value; this value has to be in range 0f <tt>0,...,15</tt>
//!
//! \return
//! The desired \c int value at \c Index
//!
//! \sa FRIDataReceivedFromKRC
//! \sa FastResearchInterface::GetKRLIntValues()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	int GetKRLIntValue(const unsigned int &Index);


//  ---------------------- Doxygen info ----------------------
//! \fn bool GetKRLFloatValue(const unsigned int &Index)
//!
//! \brief
//! Returns one single element of the KRL array <tt>\$FRI_TO_REAL[]</tt>
//! (read from the latest data telegram of the KRC unit)
//!
//! \param Index
//! The index of the desired floating point value; this value has to be in range 0f <tt>0,...,15</tt>
//!
//! \return
//! The desired \c float value at \c Index
//!
//! \sa FRIDataReceivedFromKRC
//! \sa FastResearchInterface::GetKRLFloatValues()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	float GetKRLFloatValue(const unsigned int &Index);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetKRLBoolValues(const bool *KRLBoolValues)
//!
//! \brief
//! Copies data into the data telegram to be send to the KRC unit and read by the KRL program
//! from the array <tt>\$FRI_FRM_BOOL[]</tt>
//!
//! \param KRLBoolValues
//! A pointer to an array of \c bool values; the array has to have at least a size of 16 elements.
//! The Boolean values are copied into the data telegram.
//!
//! \warning
//! This method has never been tested.
//!
//! \todo
//! Test this method!
//!
//! \sa FRIDataSendToKRC
//! \sa FastResearchInterface::SetKRLBoolValue()
//! \sa FastResearchInterface::GetKRLBoolValues()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	void SetKRLBoolValues(const bool *KRLBoolValues);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetKRLIntValues(const int *KRLIntValues)
//!
//! \brief
//! Copies data into the data telegram to be send to the KRC unit and read by the KRL program
//! from the array <tt>\$FRI_FRM_INT[]</tt>
//!
//! \param KRLIntValues
//! A pointer to an array of \c int values; the array has to have at least a size of 16 elements.
//! The integer values are copied into the data telegram.
//!
//! \sa FRIDataSendToKRC
//! \sa FastResearchInterface::SetKRLIntValue()
//! \sa FastResearchInterface::GetKRLIntValues()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	void SetKRLIntValues(const int *KRLIntValues);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetKRLFloatValues(const float *KRLFloatValues)
//!
//! \brief
//! Copies data into the data telegram to be send to the KRC unit and read by the KRL program
//! from the array <tt>\$FRI_FRM_REAL[]</tt>
//!
//! \param KRLFloatValues
//! A pointer to an array of \c float values; the array has to have at least a size of 16 elements.
//! The floating point values are copied into the data telegram.
//!
//! \sa FRIDataSendToKRC
//! \sa FastResearchInterface::SetKRLFloatValue()
//! \sa FastResearchInterface::GetKRLFloatValues()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	void SetKRLFloatValues(const float *KRLFloatValues);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetKRLBoolValue(const unsigned int &Index, const bool &Value )
//!
//! \brief
//! Copies one single Boolean value into the data telegram to be send to the KRC unit and read by the KRL program
//! from the <tt>\$FRI_FRM_BOOL[<b>Index</b>]</tt>
//!
//! \param Index
//! The index of the commanded Boolean value in the KRL array <tt>\$FRI_FRM_BOOL[]</tt>;
//! this value has to be in range 0f <tt>0,...,15</tt>
//!
//! \param Value
//! The value to be copied into the data telegram and to read by the KRL program as <tt>\$FRI_FRM_BOOL[<b>Index</b>]</tt>.
//!
//! \warning
//! This method has never been tested.
//!
//! \todo
//! Test this method!
//!
//! \sa FRIDataSendToKRC
//! \sa FastResearchInterface::SetKRLBoolValues()
//! \sa FastResearchInterface::GetKRLBoolValue()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	void SetKRLBoolValue(		const unsigned int	&Index
						 	,	const bool			&Value	);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetKRLIntValue(const unsigned int &Index, const int &Value )
//!
//! \brief
//! Copies one single integer value into the data telegram to be send to the KRC unit and read by the KRL program
//! from the <tt>\$FRI_FRM_INT[<b>Index</b>]</tt>
//!
//! \param Index
//! The index of the commanded integer value in the KRL array <tt>\$FRI_FRM_INT[]</tt>;
//! this value has to be in range 0f <tt>0,...,15</tt>
//!
//! \param Value
//! The value to be copied into the data telegram and to read by the KRL program as <tt>\$FRI_FRM_INT[<b>Index</b>]</tt>.
//!
//! \sa FRIDataSendToKRC
//! \sa FastResearchInterface::SetKRLIntValues()
//! \sa FastResearchInterface::GetKRLIntValue()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	void SetKRLIntValue(		const unsigned int	&Index
							,	const int			&Value	);


//  ---------------------- Doxygen info ----------------------
//! \fn void SetKRLFloatValue(const unsigned int &Index, const float &Value )
//!
//! \brief
//! Copies one single floating point value into the data telegram to be send to the KRC unit and read by the KRL program
//! from the <tt>\$FRI_FRM_REAL[<b>Index</b>]</tt>
//!
//! \param Index
//! The index of the commanded floating point value in the KRL array <tt>\$FRI_FRM_REAL[]</tt>;
//! this value has to be in range 0f <tt>0,...,15</tt>
//!
//! \param Value
//! The value to be copied into the data telegram and to read by the KRL program as <tt>\$FRI_FRM_REAL[<b>Index</b>]</tt>.
//!
//! \sa FRIDataSendToKRC
//! \sa FastResearchInterface::SetKRLFloatValues()
//! \sa FastResearchInterface::GetKRLFloatValue()
//! \sa \ref sec_KRLFiles
//  ----------------------------------------------------------
	void SetKRLFloatValue(		const unsigned int	&Index
						  	,	const float			&Value	);


//  ---------------------- Doxygen info ----------------------
//! \fn int WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
//!
//! \brief
//! Blocks the calling thread until a message from the KRC unit has been received
//!
//! \details
//! <ul>
//! <li>If set, this method resets the attribute FastResearchInterface::NewDataFromKRCReceived and returns (i.e.,
//! if a message has already been received, the method returns immediately).
//! <li>If the attribute FastResearchInterface::NewDataFromKRCReceived is \em not set, this method blocks
//! on the condition variable FastResearchInterface::CondVarForDataReceptionFromKRC. The corresponding
//! signal for this condition variable will be generated by the thread
//! FastResearchInterface::KRCCommunicationThreadMain() immediately after the complete reception of a new
//! datagram from the KRC unit. The timeout parameter \c TimeoutValueInMicroSeconds is only supported
//! for the <a href="http://www.qnx.com" target="_blank" >QNX Neutrino RTOS</a>. If this value is set,
//! the calling thread will be waked up after the expiration of this timeout value given in \em microseconds.
//! If this value of \c TimeoutValueInMicroSeconds equals zero, no timeout functionality will be applied, and
//! the calling thread blocks until a message from the KRC unit is received (or until
//! FastResearchInterface::~FastResearchInterface() sends a termination signal).
//! </ul>
//!
//! \param TimeoutValueInMicroSeconds
//! Number of microseconds used for the timeout functionality
//!
//! \note
//! The timeout functionality using the value of \c TimeoutValueInMicroSeconds is only implemented for
//! <a href="http://www.qnx.com" target="_blank" >QNX Neutrino RTOS</a>.
//!
//! \return
//! <ul>
//! <li> \c EOK if a KRC message has been correctly received.
//! <li> \c EAGAIN if there are insufficient system resources are available.
//! <li> \c ETIMEDOUT if the specified timeout expired.
//! <li> \c EINVAL if an internal error occurred (cf.
//! <a href="http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/pthread_cond_wait.html"
//! target="_blank" >http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/pthread_cond_wait.html</a> or
//! <a href="http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/pthread_cond_timedwait.html"
//! target="_blank" >http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/pthread_cond_timedwait.html</a>,
//! respectively).
//! </ul>
//!
//! \sa FastResearchInterface::WaitForTimerTick()
//  ----------------------------------------------------------
	int WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds = 0);


//  ---------------------- Doxygen info ----------------------
//! \fn int WaitForTimerTick(const unsigned int &TimeoutValueInMicroSeconds = 0)
//!
//! \brief
//! Blocks the calling thread until the timer thread sends a signal
//!
//! \details
//! <ul>
//! <li>If set, this method resets the attribute FastResearchInterface::TimerFlag and returns (i.e.,
//! if a message has already been received, the method returns immediately).
//! <li>If the attribute FastResearchInterface::TimerFlag is \em not set, this method blocks
//! on the condition variable FastResearchInterface::CondVarForTimer. The corresponding
//! signal for this condition variable will be generated by the thread
//! FastResearchInterface::TimerThreadMain() immediately after its timer expired
//! The timeout parameter \c TimeoutValueInMicroSeconds is only supported
//! for the <a href="http://www.qnx.com" target="_blank" >QNX Neutrino RTOS</a>. If this value is set,
//! the calling thread will be waked up after the expiration of this timeout value given in \em microseconds.
//! If this value of \c TimeoutValueInMicroSeconds equals zero, no timeout functionality will be applied, and
//! the calling thread blocks until a message from the KRC unit is received.
//! </ul>
//!
//! \param TimeoutValueInMicroSeconds
//! Number of microseconds used for the timeout functionality
//!
//! \note
//! The timeout functionality using the value of \c TimeoutValueInMicroSeconds is only implemented for
//! <a href="http://www.qnx.com" target="_blank" >QNX Neutrino RTOS</a>.
//!
//! \return
//! <ul>
//! <li> \c EOK if a KRC message has been correctly received.
//! <li> \c EAGAIN if there are insufficient system resources are available.
//! <li> \c ETIMEDOUT if the specified timeout expired.
//! <li> \c EINVAL if an internal error occurred (cf.
//! <a href="http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/pthread_cond_wait.html"
//! target="_blank" >http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/pthread_cond_wait.html</a> or
//! <a href="http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/pthread_cond_timedwait.html"
//! target="_blank" >http://www.qnx.com/developers/docs/6.3.2/neutrino/lib_ref/p/pthread_cond_timedwait.html</a>,
//! respectively).
//! </ul>
//!
//! \sa FastResearchInterface::WaitForKRCTick()
//  ----------------------------------------------------------
	int WaitForTimerTick(const unsigned int &TimeoutValueInMicroSeconds = 0);


//  ---------------------- Doxygen info ----------------------
//! \fn bool IsMachineOK(void)
//!
//! \brief
//! Returns a Boolean value indicating whether the robot is ready for operation
//!
//! \details
//! This method will return true if the following three conditions are fulfilled:
//! <ul>
//! <li>the KRC unit is <em>Command Mode</em>,
//! <li>the robot arm is powered on and the brakes are released, and
//! <li>none of the drives signals an error.
//! </ul>
//!
//! \return
//!  - \c true if the machine is ready for operation
//!  - \c false otherwise
//!
//! \sa FastResearchInterface::GetFRIMode()
//! \sa FastResearchInterface::IsRobotArmPowerOn()
//! \sa FastResearchInterface::DoesAnyDriveSignalAnError()
//  ----------------------------------------------------------
	bool IsMachineOK(void);


//  ---------------------- Doxygen info ----------------------
//! \fn const char* GetCompleteRobotStateAndInformation(void)
//!
//! \brief
//! Returns a pointer to an array of \c char values describing the \em complete state of the robot
//!
//! \details
//! This method summarizes the all available robot information in a simple C-string.
//!
//! \return
//! A pointer to an array of \c char values describing the \em complete state of the robot
//!
//! \sa FRIDataReceivedFromKRC
//! \sa GetCompleteRobotStateAndInformation.cpp
//  ----------------------------------------------------------
	const char* GetCompleteRobotStateAndInformation(void);


//  ---------------------- Doxygen info ----------------------
//! \fn int printf(const char* Format, ...)
//!
//! \brief
//! A real-time wrapper for printf
//!
//! \copydetails Console::printf()
//  ----------------------------------------------------------
	int printf(const char* Format, ...);


//  ---------------------- Doxygen info ----------------------
//! \fn int PrepareLogging(const char *FileIdentifier = NULL)
//!
//! \brief
//! Creates and prepares an output file for logging
//!
//! \details
//! Depending on the controller (cf. FastResearchInterface::CurrentControlScheme) that has been selected
//! during the call of FastResearchInterface::StartRobot(), the method DataLogging::PrepareLogging()
//! is called by this function.
//!
//! Within the class FastResearchInterface, the data logging methods are supposed to be used in the following order:\n
//! <ol>
//! <li> This method (\b not real-time capable)
//! <li> FastResearchInterface::StartLogging() (real-time capable)
//! <li> FastResearchInterface::StopLogging() (real-time capable)
//! <li> FastResearchInterface::WriteLoggingDataFile() (\b not real-time capable)
//! </ol>
//! After this method has been successfully called, the real-time capable
//! method FastResearchInterface::StartLogging() may be called at any time to start the actual data logging.
//!
//! \param FileIdentifier
//! A pointer to an array of \c char values containing a string to identify the
//! written log file. This parameter is \em optional.
//!
//! \return
//! <ul>
//! <li> \c EINVAL if a former logging file has not been closed (i.e., if FastResearchInterface::PrepareLogging()
//!	  was called without a succeeding call of FastResearchInterface::WriteLoggingDataFile().
//! <li> \c EBADF if the file could not be created.
//! <li> \c EOK if no error occurred.
//! </ul>
//!
//! \attention
//! The call of this method does \b not fulfill any real-time requirements.
//!
//! \remark
//! Please refer to the class DataLogging for more details on real-time data logging for the KUKA Light-Weight-Robot.
//!
//! \sa DataLogging::PrepareLogging()
//  ----------------------------------------------------------
	int PrepareLogging(const char *FileIdentifier = NULL);


//  ---------------------- Doxygen info ----------------------
//! \fn int StartLogging(void)
//!
//! \brief
//! Starts real-time data logging
//!
//! \details
//! A call of this method sets the flag FastResearchInterface::LoggingIsActive, which lets the communication
//! thread FastResearchInterface::KRCCommunicationThreadMain() write all relevant control data into heap memory.
//! After calling FastResearchInterface::StopLogging(), FastResearchInterface::WriteLoggingDataFile() can be called,
//! which finally writes all logged data into a file.
//!
//! This method can only be called if FastResearchInterface::PrepareLogging() was called beforehand.
//!
//! Within the class FastResearchInterface, the data logging methods are supposed to be used in the following order:\n
//! <ol>
//! <li> FastResearchInterface::PrepareLogging() (\b not real-time capable)
//! <li> This method (real-time capable)
//! <li> FastResearchInterface::StopLogging() (real-time capable)
//! <li> FastResearchInterface::WriteLoggingDataFile() (\b not real-time capable)
//! </ol>
//!
//! \return
//! <ul>
//! <li> \c EINVAL if the method FastResearchInterface::PrepareLogging() has not been called beforehand.
//! <li> \c EOK otherwise.
//! </ul>
//!
//! \remark
//! Please refer to the class DataLogging for more details on real-time data logging for the KUKA Light-Weight-Robot.
//!
//! \sa FastResearchInterface::StopLogging()
//  ----------------------------------------------------------
	int StartLogging(void);


//  ---------------------- Doxygen info ----------------------
//! \fn int StopLogging(void)
//!
//! \brief
//! Stops real-time data logging
//!
//! \details
//! A call of this method resets the flag FastResearchInterface::LoggingIsActive, which stops the communication
//! thread FastResearchInterface::KRCCommunicationThreadMain() from writing data into heap memory.
//! After calling this method, FastResearchInterface::WriteLoggingDataFile() can be called,
//! which finally writes all logged data into a file.
//!
//! This method can only be called if FastResearchInterface::StartLogging() was called beforehand.
//!
//! Within the class FastResearchInterface, the data logging methods are supposed to be used in the following order:\n
//! <ol>
//! <li> FastResearchInterface::PrepareLogging() (\b not real-time capable)
//! <li> FastResearchInterface::StartLogging() (real-time capable)
//! <li> This method (real-time capable)
//! <li> FastResearchInterface::WriteLoggingDataFile() (\b not real-time capable)
//! </ol>
//!
//! \return
//! <ul>
//! <li> \c EINVAL if the method FastResearchInterface::StartLogging() has not been called beforehand.
//! <li> \c EOK otherwise.
//! </ul>
//!
//! \attention
//! The call of this method does \b not fulfill any real-time requirements.
//!
//! \remark
//! Please refer to the class DataLogging for more details on real-time data logging for the KUKA Light-Weight-Robot.
//!
//! \sa FastResearchInterface::StartLogging()
//  ----------------------------------------------------------
	int StopLogging(void);


//  ---------------------- Doxygen info ----------------------
//! \fn int WriteLoggingDataFile(void)
//!
//! \brief
//! Writes logged data to the output file and closes the file
//!
//! \details
//! The data that has been sent \em and received between the calls of FastResearchInterface::StartLogging() and
//! FastResearchInterface::StopLogging() is written into heap memory. This method writes this data into the file
//! that was prepared by the method FastResearchInterface::PrepareLogging() and closes the file.
//!
//! This method can only be called if FastResearchInterface::PrepareLogging() was called beforehand.
//!
//! Within the class FastResearchInterface, the data logging methods are supposed to be used in the following order:\n
//! <ol>
//! <li> FastResearchInterface::PrepareLogging() (\b not real-time capable)
//! <li> FastResearchInterface::StartLogging() (real-time capable)
//! <li> FastResearchInterface::StopLogging() (real-time capable)
//! <li> This method (\b not real-time capable)
//! </ol>
//!
//! \return
//! <ul>
//! <li> \c EINVAL if the method FastResearchInterface::PrepareLogging() has not been called beforehand.
//! <li> \c EOF if the file could not be closed.
//! <li> \c EOK otherwise.
//! </ul>
//!
//! \attention
//! The call of this method does \b not fulfill any real-time requirements.
//!
//! \remark
//! Please refer to the class DataLogging for more details on real-time data logging for the KUKA Light-Weight-Robot.
//!
//! \sa DataLogging::WriteToFile()
//  ----------------------------------------------------------
	int WriteLoggingDataFile(void);


protected:


//  ---------------------- Doxygen info ----------------------
//! \fn int ReadInitFile(const char *InitFileName)
//!
//! \brief
//! Reads the initialization file
//!
//! \param InitFileName
//! A pointer to an array of \c char containing the name of the file that
//! provides the desired initialization values/parameters.
//!
//! \return
//! <ul>
//! <li> The number of read parameter
//! <li> -1 if the file specified by \c InitFileName could not be opened
//! </ul>
//!
//! \sa InitializationFileEntry
//  ----------------------------------------------------------
	int ReadInitFile(const char *InitFileName);


//  ---------------------- Doxygen info ----------------------
//! \fn int SetControlScheme(const unsigned int &ControlScheme)
//!
//! \brief
//! Prepares a part of the robot start-up procedure
//!
//! \details
//! This method encapsulates a part of the method FastResearchInterface::StartRobot().
//! It sets controller flags of FRIDataSendToKRC and sets up the shared KRL variables
//! used for intercommunication of the Fast Research Library and the KRL program \c FRIControl (cf.\ref sec_KRLFile1).
//!
//! \param ControlScheme
//! Either value of the enumeration FastResearchInterface::LWRControlModes:\n
//! <ul>
//! <li> FastResearchInterface::JOINT_POSITION_CONTROL for joint position control
//! <li> FastResearchInterface::CART_IMPEDANCE_CONTROL for Cartesian impedance control
//! <li> FastResearchInterface::JOINT_IMPEDANCE_CONTROL	for joint impedance control
//! <li> FastResearchInterface::JOINT_TORQUE_CONTROL for joint torque control\n
//! </ul>\n
//!
//! \return
//! <ul>
//! <li> \c EINVAL if the value is not an element of LWRControlModes.
//! <li> \c ENOTCONN if no connection between the remote host and the KRC unit is available,
//! <li>	or if the KRC unit does not answer anymore (e.g., due to a manual call of <tt>firClose()</tt>.
//! <li> \c EBUSY if the KRC unit is already in <em>Command Mode</em>.
//! <li> \c EOK if no error occurred.
//! </ul>
//!
//! \sa FastResearchInterface::StartRobot()
//! \sa FRIDataSendToKRC
//  ----------------------------------------------------------
	int SetControlScheme(const unsigned int &ControlScheme);


//  ---------------------- Doxygen info ----------------------
//! \fn static void *TimerThreadMain(void *ThreadArgument)
//!
//! \brief
//! Thread function for the timer thread
//!
//! \details
//! This function provides a basic timer functionality by periodically broadcasting a signal via a
//! condition via a condition variable (FastResearchInterface::CondVarForTimer).
//! This signal may be used by user applications calling FastResearchInterface::WaitForTimerTick().
//!
//! \param ThreadArgument
//! A pointer the FastResearchInterface object of the calling thread.
//!
//! \note
//! This method is currently only implemented for the <a href="http://www.qnx.com" target="_blank" >QNX Neutrino RTOS</a>.
//!
//! \todo
//! Implement this method for other operating systems.
//!
//! \sa FastResearchInterface::WaitForTimerTick()
//  ----------------------------------------------------------
	static void *TimerThreadMain(void *ThreadArgument);


//  ---------------------- Doxygen info ----------------------
//! \fn static void *TimerThreadMain(void *ThreadArgument)
//!
//! \brief
//! Thread function for the KRC communication thread
//!
//! \details
//! This function is the gateway to the KRC unit; it sends and receives data telegrams using the class
//! friUDP. Right after the reception of one FRIDataReceivedFromKRC package, a FRIDataSendToKRC is sent back to the
//! KRC unit. Furthermore, a signal to all other threads is broadcasted through the condition variable
//! FastResearchInterface::CondVarForDataReceptionFromKRC. Other threads may use the method
//! FastResearchInterface::WaitForKRCTick() to wait for this signal.
//!
//! \param ThreadArgument
//! A pointer the FastResearchInterface object of the calling thread.
//!
//! \sa FastResearchInterface::WaitForKRCTick()
//  ----------------------------------------------------------
	static void *KRCCommunicationThreadMain(void *ThreadArgument);


//  ---------------------- Doxygen info ----------------------
//! \enum CalledLoggingMethod
//!
//! \brief
//! Describes the state of the data logger
//!
//! \sa FastResearchInterface::PrepareLogging() (\b not real-time capable)
//! \sa FastResearchInterface::StartLogging() (real-time capable)
//! \sa FastResearchInterface::StopLogging() (real-time capable)
//! \sa FastResearchInterface::WriteLoggingDataFile() (\b not real-time capable)
//  ----------------------------------------------------------
	enum CalledLoggingMethod
	{
		PrepareLoggingCalled		=	1,
		StartLoggingCalled			=	2,
		StopLoggingCalled			=	3,
		WriteLoggingDataFileCalled	=	4
	};


//  ---------------------- Doxygen info ----------------------
//! \var bool TimerThreadIsRunning
//!
//! \brief
//! Used as indication for the timer thread to terminate itself
//  ----------------------------------------------------------
	bool					TimerThreadIsRunning;


//  ---------------------- Doxygen info ----------------------
//! \var bool TimerFlag
//!
//! \brief
//! Set by the timer thread each time its timer fires
//  ----------------------------------------------------------
	bool					TimerFlag;


//  ---------------------- Doxygen info ----------------------
//! \var bool KRCCommunicationThreadIsRunning
//!
//! \brief
//! Set by the KRC communication thread each time a message from the KRC unit is received
//  ----------------------------------------------------------
	bool					KRCCommunicationThreadIsRunning;


//  ---------------------- Doxygen info ----------------------
//! \var bool NewDataFromKRCReceived
//!
//! \brief
//! Used as indication for the KRC communication thread to terminate itself
//  ----------------------------------------------------------
	bool					NewDataFromKRCReceived;


//  ---------------------- Doxygen info ----------------------
//! \var bool LoggingIsActive
//!
//! \brief
//! Used by FastResearchInterface::StartLogging() and FastResearchInterface::StopLogging()
//  ----------------------------------------------------------
	bool					LoggingIsActive;
	

//  ---------------------- Doxygen info ----------------------
//! \var bool ThreadCreated
//!
//! \brief
//! Used during the creation of new threads to acknowledge the creating
//! thread that the new thread is running.
//!
//! \sa MutexForThreadCreation
//! \sa CondVarForThreadCreation
//  ----------------------------------------------------------
	bool					ThreadCreated;	


//  ---------------------- Doxygen info ----------------------
//! \var CalledLoggingMethod LoggingState
//!
//! \brief
//! Stores which logging methods has been called lately
//  ----------------------------------------------------------
	CalledLoggingMethod		LoggingState;


//  ---------------------- Doxygen info ----------------------
//! \var char *RobotName
//!
//! \brief
//! A pointer to an array of \c char values containing the name of the robot (specified in the initialization file)
//!
//! \sa sec_InitFile
//  ----------------------------------------------------------
	char					*RobotName;


//  ---------------------- Doxygen info ----------------------
//! \var char *LoggingPath
//!
//! \brief
//! A pointer to an array of \c char values containing the path of the log file (specified in the initialization file)
//!
//! \sa sec_InitFile
//  ----------------------------------------------------------
	char					*LoggingPath;


//  ---------------------- Doxygen info ----------------------
//! \var char *LoggingFileName
//!
//! \brief
//! A pointer to an array of \c char values containing the name of the log file (specified in the initialization file)
//!
//! \sa sec_InitFile
//  ----------------------------------------------------------
	char					*LoggingFileName;


//  ---------------------- Doxygen info ----------------------
//! \var char *RobotStateString
//!
//! \brief
//! A pointer to an array of \c char values containing the description of the complete robot state
//!
//! \sa FastResearchInterface::GetCompleteRobotStateAndInformation()
//  ----------------------------------------------------------
	char					*RobotStateString;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int CurrentControlScheme
//!
//! \brief
//! Contains the current control scheme
//!
//! \sa FastResearchInterface::LWRControlModes
//  ----------------------------------------------------------
	unsigned int			CurrentControlScheme;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int NumberOfLoggingFileEntries
//!
//! \brief
//! Contains the maximum number of logging file entries (specified in the initialization file)
//! \sa sec_InitFile
//  ----------------------------------------------------------
	unsigned int			NumberOfLoggingFileEntries;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int PriorityKRCCommunicationThread
//!
//! \brief
//! Contains the priority of the KRC communication thread (specified in the initialization file)
//!
//! \sa FastResearchInterface::KRCCommunicationThreadMain()
//! \sa sec_InitFile
//  ----------------------------------------------------------
	unsigned int			PriorityKRCCommunicationThread;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int PriorityTimerThread
//!
//! \brief
//! Contains the priority of the timer thread (specified in the initialization file)
//!
//! \sa FastResearchInterface::TimerThreadMain()
//! \sa sec_InitFile
//  ----------------------------------------------------------
	unsigned int			PriorityTimerThread;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int PriorityMainThread
//!
//! \brief
//! Contains the priority of the thread that called the constructor (specified in the initialization file)
//! \sa sec_InitFile
//  ----------------------------------------------------------
	unsigned int			PriorityMainThread;


//  ---------------------- Doxygen info ----------------------
//! \var unsigned int PriorityOutputConsoleThread
//!
//! \brief
//! Contains the priority of the output thread (specified in the initialization file)
//!
//! \sa Console::Console()
//! \sa sec_InitFile
//  ----------------------------------------------------------
	unsigned int			PriorityOutputConsoleThread;


//  ---------------------- Doxygen info ----------------------
//! \var double CycleTime
//!
//! \brief
//! Contains the cycle time in seconds (specified in the initialization file)
//!
//! \sa sec_InitFile
//  ----------------------------------------------------------
	double					CycleTime;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_mutex_t MutexForControlData
//!
//! \brief
//! Mutex to protect FastResearchInterface::ReadData and FastResearchInterface::CommandData
//  ----------------------------------------------------------
	pthread_mutex_t			MutexForControlData;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_mutex_t MutexForCondVarForTimer
//!
//! \brief
//! Mutex to protect FastResearchInterface::TimerFlag
//  ----------------------------------------------------------
	pthread_mutex_t			MutexForCondVarForTimer;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_mutex_t MutexForLogging
//!
//! \brief
//! Mutex to protect FastResearchInterface::LoggingIsActive
//  ----------------------------------------------------------
	pthread_mutex_t			MutexForLogging;

	
//  ---------------------- Doxygen info ----------------------
//! \var pthread_mutex_t MutexForThreadCreation
//!
//! \brief
//! Mutex used during the creation of new threads
//!
//! \sa ThreadCreated
//! \sa CondVarForThreadCreation
//  ----------------------------------------------------------
	pthread_mutex_t			MutexForThreadCreation;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_cond_t CondVarForTimer
//!
//! \brief
//! Condition variable used by the timer thread FastResearchInterface::TimerThreadMain() for broadcasting
//! after the firing of its timer
//  ----------------------------------------------------------
	pthread_cond_t			CondVarForTimer;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_cond_t CondVarForDataReceptionFromKRC
//!
//! \brief
//! Condition variable used by the KRC communication thread FastResearchInterface::KRCCommunicationThreadMain() for broadcasting
//! after the reception of a message from the KRC unit
//  ----------------------------------------------------------
	pthread_cond_t			CondVarForDataReceptionFromKRC;

	
//  ---------------------- Doxygen info ----------------------
//! \var pthread_mutex_t CondVarForThreadCreation
//!
//! \brief
//! Condition variable used during the creation of new threads
//!
//! \sa ThreadCreated
//! \sa MutexForThreadCreation
//  ----------------------------------------------------------
	pthread_cond_t			CondVarForThreadCreation;		


//  ---------------------- Doxygen info ----------------------
//! \var pthread_t KRCCommunicationThread
//!
//! \brief
//! POSIX thread identifier for the KRC communication thread FastResearchInterface::KRCCommunicationThreadMain()
//  ----------------------------------------------------------
	pthread_t				KRCCommunicationThread;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_t TimerThread
//!
//! \brief
//! POSIX thread identifier for the timer thread FastResearchInterface::TimerThreadMain()
//  ----------------------------------------------------------
	pthread_t				TimerThread;


//  ---------------------- Doxygen info ----------------------
//! \var pthread_t MainThread
//!
//! \brief
//! POSIX thread identifier for the thread calling the constructor FastResearchInterface::FastResearchInterface()
//  ----------------------------------------------------------
	pthread_t				MainThread;


//  ---------------------- Doxygen info ----------------------
//! \var Console *OutputConsole
//!
//! \brief
//! Pointer to a Console object used for message output through a low-priority thread
//  ----------------------------------------------------------
	Console					*OutputConsole;


//  ---------------------- Doxygen info ----------------------
//! \var DataLogging *DataLogger
//!
//! \brief
//! Pointer to a DataLogging object used for real-time logging of low-level control data
//  ----------------------------------------------------------
	DataLogging				*DataLogger;


//  ---------------------- Doxygen info ----------------------
//! \var FRIDataReceivedFromKRC ReadData
//!
//! \brief
//! Data structure object containing a copy of a complete received data telegram from the KRC unit
//  ----------------------------------------------------------
	FRIDataReceivedFromKRC 			ReadData;


//  ---------------------- Doxygen info ----------------------
//! \var FRIDataSendToKRC CommandData
//!
//! \brief
//! Data structure object containing a copy of a complete data telegram to be sent to the KRC unit
//  ----------------------------------------------------------
	FRIDataSendToKRC				CommandData;

};	// class FastResearchInterface

#endif
