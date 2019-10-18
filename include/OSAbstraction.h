//  ---------------------- Doxygen info ----------------------
//! \file OSAbstraction.h
//!
//! \brief
//! Header file for simple OS-specific functions for abstraction
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


#ifndef __OSAbstraction__
#define __OSAbstraction__

#include <stdlib.h>



//  ---------------------- Doxygen info ----------------------
//! \def OS_FOLDER_SEPARATOR
//!
//! \brief
//! Slash for non-Microsoft operating system or backslash for Microsoft operating systems
//  ----------------------------------------------------------
#if defined(WIN32) || defined(WIN64) || defined(_WIN64)
#define OS_FOLDER_SEPARATOR	("\\")
#else
#define OS_FOLDER_SEPARATOR	("/")
#endif


//  ---------------------- Doxygen info ----------------------
//! \fn unsigned char WaitForKBCharacter(bool *Abort = NULL)
//!
//! \brief Waits for one single keyboard stroke
//!
//! \details
//! The function returns after the stroke or if \c Abort becomes set.
//! If \c Abort is set at the call of the function, the function only checks once, if a key has
//! been pressed.
//!
//! \param Abort
//! Pointer to a boolean values, which lets the function terminate
//!
//! \return
//!  - Value of the pressed key
//!  - 255 if no key was pressed
//!
//! \sa CheckForKBCharacter()
//  ----------------------------------------------------------
unsigned char 	WaitForKBCharacter(bool *Abort = NULL);


//  ---------------------- Doxygen info ----------------------
//! \fn unsigned char char CheckForKBCharacter(void)
//!
//! \brief Checks for one single keyboard stroke
//!
//! \details
//! The function returns immediately.
//!
//! \return
//!  - Value of the pressed key
//!  - 255 if no key was pressed
//!
//! \sa WaitForKBCharacter()
//  ----------------------------------------------------------
unsigned char CheckForKBCharacter(void);


//  ---------------------- Doxygen info ----------------------
//! \fn float GetSystemTimeInSeconds(const bool &Reset = false)
//!
//! \brief Returns a time value in seconds
//!
//! \details
//! Operating system independent function to return the value of the system time
//! in seconds.
//!
//! \param Reset
//! If this flag is set, the value of \em this system time will be
//! set to zero.
//!
//! \return
//! The value of the system time in seconds.
//  ----------------------------------------------------------
float GetSystemTimeInSeconds(const bool &Reset = false);



// ##############  L I N U X  #######################################################
#include <LinuxAbstraction.h>

// ##################################################################################
// ##################################################################################
// ##################################################################################

#endif
