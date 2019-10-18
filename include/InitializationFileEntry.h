//  ---------------------- Doxygen info ----------------------
//! \file InitializationFileEntry.h
//!
//! \brief
//! Header file for the class InitializationFileEntry
//!
//! \details
//! The class InitializationFileEntry provides a simple interface to read
//! entries from an initialization file and to provide in an
//! application.
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


#ifndef __InitializationFileEntry__
#define __InitializationFileEntry__


#include <stdlib.h>


//  ---------------------- Doxygen info ----------------------
//! \def VALUELEN
//!
//! \brief
//! The maximum length of one single entry of the initialization
//! file in bytes (including the terminating \c \\0 character)
//  ----------------------------------------------------------
#define VALUELEN	((unsigned int)512)


//  ---------------------- Doxygen info ----------------------
//! \def NAMELEN
//!
//! \brief
//! The maximum length of one entry or section name of the initialization
//! file in bytes (including the terminating \c \\0 character)
//  ----------------------------------------------------------
#define NAMELEN		((unsigned int)512)


//  ---------------------- Doxygen info ----------------------
//! \class InitializationFileEntry
//!
//! \brief
//! A simple implementation of a class for reading an initialization file.
//!
//! \attention
//! The implementation of this class does \b not feature real-time behavior.
//!
//! \sa \ref sec_InitFile
//! \sa InitializationFileEntry::InitializationFileEntry()
//  ----------------------------------------------------------
class InitializationFileEntry
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn InitializationFileEntry(const char *FileName = NULL)
//!
//! \brief
//! Constructor
//!
//! \details
//! The constructor reads all entry from the initialization file \c Filename and
//! creates a double-linked list of InitializationFileEntry objects.
//!
//! \param  FileName
//! A pointer to an array of \c char containing the name of the file that
//! provides the desired initialization values/parameters
//!
//! \attention
//! The implementation of the constructor does \b not feature real-time behavior.
//!
//! \sa \ref sec_InitFile
//  ----------------------------------------------------------
	InitializationFileEntry(const char *FileName = NULL);


//  ---------------------- Doxygen info ----------------------
//! \fn ~InitializationFileEntry(void)
//!
//! \brief
//! Destructor
//!
//! \details
//! The destructor deletes all list entries.
//!
//! \attention
//! The implementation of the constructor does \b not feature real-time behavior.
//  ----------------------------------------------------------
	~InitializationFileEntry(void);


//  ---------------------- Doxygen info ----------------------
//! \fn char *GetAll(void)
//!
//! \brief
//! Returns a complete entry
//!
//! \return
//! A pointer to an array of \c char values containing the current entry
//  ----------------------------------------------------------
	char *GetAll(void);


//  ---------------------- Doxygen info ----------------------
//! \fn bool FindEntry(const char *Name)
//!
//! \brief
//! Looks for an entry that matches Name
//!
//! \details
//! If one could be found it sets the internal current entry pointer to this
//! entry and returns \c true. Otherwise, the current entry pointer will remain
//! unchanged and the return value is false.
//!
//! \param Name
//! A pointer to an array of \c char values containing the name of the entry to be looked for
//!
//! \return
//!  - \c true if the entry \c Name was found
//!  - \c false otherwise
//  ----------------------------------------------------------
	bool FindEntry(const char *Name);


//  ---------------------- Doxygen info ----------------------
//! \fn char *GetValue(void)
//!
//! \brief
//! Returns the value of the current entry (may be empty)
//!
//! \return
//! A pointer to an array of \c char values containing the value
//! of the current entry
//  ----------------------------------------------------------
	char *GetValue(void);


//  ---------------------- Doxygen info ----------------------
//! \fn char *GetName (void)
//!
//! \brief
//! Returns the name of the current entry
//!
//! \note
//! The pointer will remain valid until the next call of GetName().
//!
//! \return
//! A pointer to an array of \c char values containing the name
//! of the current entry
//  ----------------------------------------------------------
	char *GetName (void);


//  ---------------------- Doxygen info ----------------------
//! \fn char *GetSection(void)
//!
//! \brief
//! Returns the name of the current section (may be empty)
//!
//! \return
//! A pointer to an array of \c char values containing the name
//! of the current entry section.
//  ----------------------------------------------------------
	char *GetSection(void);


//  ---------------------- Doxygen info ----------------------
//! \fn bool NextEntry(void)
//!
//! \brief
//! Set the internal current entry pointer to the successor of the current entry
//!
//! \return
//! - \c true if another entry exists
//! - \c false otherwise
//  ----------------------------------------------------------
	bool NextEntry(void);


protected:


//  ---------------------- Doxygen info ----------------------
//! \fn bool FindNextSection(void)
//!
//! \brief
//! Set the internal current entry pointer to the beginning of the next section
//!
//! \return
//! - \c true if another section exists
//! - \c false otherwise
//  ----------------------------------------------------------
	bool FindNextSection(void);


//  ---------------------- Doxygen info ----------------------
//! \fn bool Add(const char* Line)
//!
//! \brief
//! Add a line to the list
//!
//! \details
//! - A line containing [name] introduces a new section.
//! - A '=' may be used to separate an entry name and it's value.
//! - A general line (without [,] and =) will be used as entry name
//!
//! \param Line
//! A pointer to an array of \c char values containing the line to be added.
//!
//! \return
//! - \c true if the line could be added
//! - \c false otherwise
//  ----------------------------------------------------------
	bool Add(const char* Line);


//  ---------------------- Doxygen info ----------------------
//! \fn void ReadFile(const char *FileName)
//!
//! \brief
//! Reads the initialization file
//!
//! \details
//! This method reads the contents of the file \c FileName and stores
//! it in the \c InitializationFileEntry list.
//!
//! \param FileName
//! A pointer to an array of \c char values containing the file name
//  ----------------------------------------------------------
	void ReadFile(const char *FileName);


//  ---------------------- Doxygen info ----------------------
//! \fn InitializationFileEntry *CurrentEntry
//!
//! \brief
//! A pointer to the current entry of the parsed initialization file
//  ----------------------------------------------------------
	InitializationFileEntry		*CurrentEntry;


//  ---------------------- Doxygen info ----------------------
//! \fn InitializationFileEntry *Prev
//!
//! \brief
//! A pointer to the previous entry of the parsed initialization file
//  ----------------------------------------------------------
	InitializationFileEntry		*Prev;


//  ---------------------- Doxygen info ----------------------
//! \fn InitializationFileEntry *Next
//!
//! \brief
//! A pointer to the next entry of the parsed initialization file
//  ----------------------------------------------------------
	InitializationFileEntry		*Next;


//  ---------------------- Doxygen info ----------------------
//! \fn char Name[NAMELEN]
//!
//! \brief
//! An array of \c char values used by the method InitializationFileEntry::GetName()
//! to return the name of the current entry
//  ----------------------------------------------------------
	char 						Name[NAMELEN];


//  ---------------------- Doxygen info ----------------------
//! \fn char Value[NAMELEN]
//!
//! \brief
//! An array of \c char values containing the value of the current entry
//  ----------------------------------------------------------
	char 						Value[VALUELEN];


//  ---------------------- Doxygen info ----------------------
//! \fn char EntryName[NAMELEN]
//!
//! \brief
//! An array of \c char values containing the name of the current entry
//  ----------------------------------------------------------
	char 						EntryName[NAMELEN];


//  ---------------------- Doxygen info ----------------------
//! \fn char SectionName[NAMELEN]
//!
//! \brief
//! An array of \c char values containing the name of the section of the current entry
//  ----------------------------------------------------------
	char 						SectionName[NAMELEN];


//  ---------------------- Doxygen info ----------------------
//! \fn char CurrentSectionName[NAMELEN]
//!
//! \brief
//! An array of \c char values containing the name of the section
//! of the current entry used by InitializationFileEntry::Add()
//! if a new section was found
//  ----------------------------------------------------------
	char 						CurrentSectionName[NAMELEN];

};	// class InitializationFileEntry


#endif
