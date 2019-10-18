//  ---------------------- Doxygen info ----------------------
//! \file UDPSocket.cpp
//!
//! \brief
//! Implementation file for UDP sockets
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


#include "UDPSocket.h"
 #include <OSAbstraction.h>

#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib") 
#endif

//  ---------------------- Doxygen info ----------------------
//! \def SERVER_PORT
//!
//! \brief
//! Default UDP server port
//  ----------------------------------------------------------
#define SERVER_PORT 49938


// ****************************************************************
// Constructor
//
UDPSocket::UDPSocket(void)
{
	this->ServerPortNumber	=	SERVER_PORT;

	// --------------------------------------------
	//! \todo Remove this.
	if (!ALL_DATA_SIZES_SENT_TO_KRC_ARE_OK)
	{
		printf("data structure size error!\n");
		exit(1);
	}
	// --------------------------------------------

#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
	StartWindowsSocket();
#endif
	this->Init();
}


// ****************************************************************
// Destructor
//
UDPSocket::~UDPSocket()
{
  this->Close();
}


// ****************************************************************
// StartWindowsSocket()
//
#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
int UDPSocket::StartWindowsSocket(void)
{
	WSADATA WSAData;
	return (WSAStartup(MAKEWORD(2,0), &WSAData));
}
#endif


// ****************************************************************
// Init()
//
void UDPSocket::Init(void)
{
	struct sockaddr_in		KRCAddress;
	
	memset(&KRCAddress					, 0,	sizeof(KRCAddress						));
	memset(&this->IPAddressOfKRCUnit	, 0,	sizeof(this->IPAddressOfKRCUnit		));
	
	UDPSocketNumber = socket(PF_INET, SOCK_DGRAM, 0);
	
	if (UDPSocketNumber < 0)
	{
		fprintf(stderr, "ERROR: Cannot open socket.\n");
		fflush(stderr);
		exit(EXIT_FAILURE);
	}

	KRCAddress.sin_family		=	AF_INET;
	KRCAddress.sin_addr.s_addr	=	htonl(INADDR_ANY);
	KRCAddress.sin_port			=	htons(ServerPortNumber);

	if (bind(UDPSocketNumber, (struct sockaddr *)&KRCAddress, sizeof(KRCAddress)) < 0)
	{
		fprintf(stderr, "ERROR: Cannot bind port %d.\n", ServerPortNumber);
		fflush(stderr);
		this->Close();
		exit(EXIT_FAILURE);
	}
}


// ****************************************************************
// ReceiveFRIDataFromKRC()
//
int UDPSocket::ReceiveFRIDataFromKRC(FRIDataReceivedFromKRC *DataPackageFromKRC) const
{
	int					DataReceived	=	0;
	
	if (UDPSocketNumber >= 0)
	{		
		DataReceived = ReceiveUDPPackage(		UDPSocketNumber
											,	DataPackageFromKRC	);

		if (DataReceived == sizeof(FRIDataReceivedFromKRC))
		{
			return (EOK);
		}
		else
		{
			fprintf(stderr, "ERROR: Data size incorrect (received: %d, expected %d)\n", DataReceived, sizeof(FRIDataReceivedFromKRC));
			fflush(stderr);
		}
	}
	memset(DataPackageFromKRC, 0, sizeof(FRIDataReceivedFromKRC));
	
	return (ENOTCONN);
}


// ****************************************************************
// SendFRIDataToKRC()
//
int UDPSocket::SendFRIDataToKRC(const FRIDataSendToKRC *DataPackageToBeSentToKRC)
{
	int NumberOfSentBytes	=	0;

	this->IPAddressOfKRCUnit.sin_family = AF_INET;

	if	(	(UDPSocketNumber					>=	0	)
		&&	(ntohs(IPAddressOfKRCUnit.sin_port)	!=	0	)	)
	{
		NumberOfSentBytes	=	sendto(		UDPSocketNumber
										,	(char *) DataPackageToBeSentToKRC
										,	sizeof(FRIDataSendToKRC)
										,	0
										,	(struct sockaddr *)&IPAddressOfKRCUnit
										,	sizeof(IPAddressOfKRCUnit));
										
		if (NumberOfSentBytes == sizeof(FRIDataSendToKRC))
		{
			return (EOK);
		}
	}
	
	return (ENOTCONN);
}


// ****************************************************************
// Close()
//
void UDPSocket::Close(void)
{
	if (UDPSocketNumber >= 0)
	{
#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
		closesocket(UDPSocketNumber); 
		WSACleanup(); 
#else
		close(UDPSocketNumber);
#endif
	}
	UDPSocketNumber	=	-1;
	
	return;
}


// ****************************************************************
// ReceiveUDPPackage()
//
int UDPSocket::ReceiveUDPPackage(const int UDPSocketNumber, FRIDataReceivedFromKRC *ReceivedData) const 
{
	int NumberOfReceivedBytes	=	0;

	if (UDPSocketNumber	>=	0)
	{
#ifdef _NTO_
		socklen_t SizeOfSocketAddressInBytes	=	sizeof(struct sockaddr_in);
#else
#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
		int SizeOfSocketAddressInBytes			=	sizeof(struct sockaddr_in);
#else
		unsigned int SizeOfSocketAddressInBytes	=	sizeof(struct sockaddr_in);
#endif
#endif

		NumberOfReceivedBytes	=	recvfrom(		UDPSocketNumber
												,	(char *)ReceivedData
												,	sizeof(FRIDataReceivedFromKRC)
												,	0
												,	(struct sockaddr *)&IPAddressOfKRCUnit
												,	&SizeOfSocketAddressInBytes);

		return (NumberOfReceivedBytes);
	}
	
	return(-1);
}
