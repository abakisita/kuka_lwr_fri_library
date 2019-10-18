//  ---------------------- Doxygen info ----------------------
//! \file UDPSocket.h
//!
//! \brief
//! Header file for UDP sockets
//!
//! \details
//! The communication between the remote PC and the KRC is done through 
//! UDP sockets
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



#ifndef __UDPSocket__
#define __UDPSocket__

#include "FRICommunication.h"
#include <stdio.h>
#include <stdlib.h>


#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
#include <winsock2.h>
#else
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif


//  ---------------------- Doxygen info ----------------------
//! \class UDPSocket
//!
//! \brief
//! Simple implementation of UDP sockets to intercommunicate between the
//! the remote PC and the KRC unit
//!
//! \sa FRICommunication.h
//  ----------------------------------------------------------
class UDPSocket
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn UDPSocket(void)
//!
//! \brief
//! Constructor
//!
//! \details
//! This constructor initializes the socket.
//!
//! \attention
//! Calling of the constructor does \b not fulfill any real-time requirements.
//  ----------------------------------------------------------
	UDPSocket(void);
	
	
//  ---------------------- Doxygen info ----------------------
//! \fn ~UDPSocket(void)
//!
//! \brief
//! Destructor
//!
//! \details
//! This destructor closes the socket again.
//!
//! \attention
//! Calling of the destructor does \b not fulfill any real-time requirements.
//  ----------------------------------------------------------
	~UDPSocket(void);
	
	
//  ---------------------- Doxygen info ----------------------
//! \fn int SendFRIDataToKRC(const FRIDataSendToKRC *DataPackageToBeSentToKRC)
//!
//! \brief
//! Sends data from the remote PC the KRC unit
//!
//! \param DataPackageToBeSentToKRC
//! Pointer to a FRIDataSendToKRC object that contains the data to be sent
//!
//! \return
//! <ul>
//! <li>\c EOK if successful</li>
//! <li>\c ENOTCONN if no connection is established or if a wrong amount
//!        of data has been sent</li>
//! </ul>
//  ----------------------------------------------------------	
	int SendFRIDataToKRC(const FRIDataSendToKRC *DataPackageToBeSentToKRC);


//  ---------------------- Doxygen info ----------------------
//! \fn int ReceiveFRIDataFromKRC(FRIDataReceivedFromKRC *DataPackageFromKRC) const
//!
//! \brief
//! Receives data from the KRC unit (blocking)
//!
//! \param DataPackageFromKRC
//! Pointer to a FRIDataReceivedFromKRC object that will contain the
//! received data from the KRC unit.
//!
//! \return
//! <ul>
//! <li>\c EOK if successful</li>
//! <li>\c ENOTCONN if no connection is established or if a wrong amount
//!        of data has been received</li>
//! </ul>
//  ----------------------------------------------------------		
	int ReceiveFRIDataFromKRC(FRIDataReceivedFromKRC *DataPackageFromKRC) const;


protected:

//  ---------------------- Doxygen info ----------------------
//! \fn void Init(void)
//!
//! \brief
//! Initializes the socket (only called by the constructor)
//  ----------------------------------------------------------
	void Init(void);
	
	
//  ---------------------- Doxygen info ----------------------
//! \fn void void Close(void)
//!
//! \brief
//! Closes the socket (only called by the destructor)
//  ----------------------------------------------------------	
	void Close(void);


#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
//  ---------------------- Doxygen info ----------------------
//! \fn int StartWindowsSocket(void)
//!
//! \brief
//! Microsoft Windows only: Initializes the socket (only called by the constructor)
//!
//! \return
//! <ul>
//! <li> Zero if successful</li>
//! <li> Non-zero if Winsock DLL cannot be initialized</li>
//! </ul>
//  ----------------------------------------------------------
	int StartWindowsSocket(void);
#endif


private:

//  ---------------------- Doxygen info ----------------------
//! \fn int ReceiveUDPPackage(const int UDPSocketNumber, FRIDataReceivedFromKRC *ReceivedData) const
//!
//! \brief
//! Sends data from the remote PC to the KRC unit using \c recvfrom() (only called by the method ReceiveFRIDataFromKRC())
//!
//! \param UDPSocketNumber
//! Socket number
//!
//! \param ReceivedData
//! Pointer to a FRIDataReceivedFromKRC object that will contain the
//! received data from the KRC unit.
//!
//! \return
//! <ul>
//! <li>\c EOK if successful</li>
//! <li>\c ENOTCONN if no connection is established or if a wrong amount
//!        of data has been received</li>
//! </ul>
//  ----------------------------------------------------------
	int ReceiveUDPPackage(const int UDPSocketNumber, FRIDataReceivedFromKRC *ReceivedData) const;
	
	
//  ---------------------- Doxygen info ----------------------
//! \var UDPSocketNumber
//!
//! \brief
//! Contains the integer socket number
//  ----------------------------------------------------------
	int UDPSocketNumber ;
	
	
//  ---------------------- Doxygen info ----------------------
//! \var ServerPortNumber
//!
//! \brief
//! Contains the integer server port number
//  ----------------------------------------------------------	
	int ServerPortNumber;
	
//  ---------------------- Doxygen info ----------------------
//! \var IPAddressOfKRCUnit
//!
//! \brief
//! Contains the IP address of the KRC unit
//  ----------------------------------------------------------		
	struct sockaddr_in IPAddressOfKRCUnit;
};



#endif
