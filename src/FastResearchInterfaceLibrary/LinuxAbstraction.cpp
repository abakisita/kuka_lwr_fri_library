//  ---------------------- Doxygen info ----------------------
//! \file LinuxAbstraction.cpp
//!
//! \brief
//! Implementation file containing OS-specific functions (Linux)
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


#include <OSAbstraction.h>
#include <time.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>




// ****************************************************************
// Global variables
//

static struct termios 			termattr
							,	save_termattr;

static int 						ttysavefd								=	-1;

static bool						GetSystemTimeInSecondsCalledFirstTime	=	true;

struct timespec					StoredSystemTimeInSeconds;




// ****************************************************************
// Data structure declarations
//

static enum
{
  RESET, RAW
} ttystate = RESET;

int DisableSingleCharacterInput(void);
int EnableSingleCharacterInput(void);


void delay(const int &TimeInMilliseconds)
{
	usleep(TimeInMilliseconds * 1000);
	return;
}


int stricmp(const char *s1, const char *s2)
{
	return(strcasecmp(s1, s2));
}


// ****************************************************************
// EnableSingleCharacterInput()
// Put the user's TTY in one-character-at-a-time mode.
// returns 0 on success, -1 on failure.

int EnableSingleCharacterInput(void)
{
	int 	i;

	i = tcgetattr (STDIN_FILENO, &termattr);

	if (i < 0)
	{
		printf("tcgetattr() returned %d for fildes = %d ", i, STDIN_FILENO);
		perror ("");
		return -1;
	}

	save_termattr = termattr;

	termattr.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
	termattr.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
	termattr.c_cflag &= ~(CSIZE | PARENB);
	termattr.c_cflag |= CS8;
	termattr.c_oflag &= ~(OPOST);

	termattr.c_cc[VMIN] = 0;
	termattr.c_cc[VTIME] = 0;

	i = tcsetattr (STDIN_FILENO, TCSANOW, &termattr);
	if (i < 0)
	{
		printf("tcsetattr() returned %d for fildes=%d",i,STDIN_FILENO);
		perror("");
		return -1;
	}

	ttystate = RAW;
	ttysavefd = STDIN_FILENO;
	return 0;
}



// ****************************************************************
// DisableSingleCharacterInput()
// Restore normal TTY mode. Very important to call
// the function before exiting else the TTY won't be too usable.
// returns 0 on success, -1 on failure.

int DisableSingleCharacterInput()
{
	int i;
	if (ttystate != RAW)
	{
		return 0;
	}
	i = tcsetattr (STDIN_FILENO, TCSAFLUSH, &save_termattr);
	if (i < 0)
	{
		return -1;
	}
	ttystate = RESET;
	return 0;
}


// ****************************************************************
// WaitForKBCharacter()
//
unsigned char WaitForKBCharacter(bool *Abort)
{
	unsigned char	ch		=	0;
	size_t			size;

	fflush(stdout);
	fflush(stdin);

	if (Abort == NULL)
	{
		delay(100);
	}

	if (ttystate != RAW)
	{
		EnableSingleCharacterInput();
	}

	if (Abort == NULL)
	{
		while (1)
		{
			usleep(20000);

			size = read (STDIN_FILENO, &ch, 1);
			if (size > 0)
			{
				break;
			}
		}
	}
	else
	{
		size = read (STDIN_FILENO, &ch, 1);
		if (size == 0)
		{
			ch = 255;
			while (!(*Abort))
			{
				usleep(20000);

				size = read (STDIN_FILENO, &ch, 1);
				if (size > 0)
				{
					break;
				}
			}
		}
	}
	DisableSingleCharacterInput();
	fflush(stdout);
	fflush(stdin);
	if (Abort == NULL)
	{
		delay(100);
	}
	return(ch);
}


unsigned char CheckForKBCharacter(void)
{
	unsigned char	ch		=	0;
	size_t			size;

	fflush(stdout);
	fflush(stdin);

	if (ttystate != RAW)
	{
		EnableSingleCharacterInput();
	}

	size = read (STDIN_FILENO, &ch, 1);

	if (size == 0)
	{
		ch = 255;
	}

	DisableSingleCharacterInput();

	return(ch);
}


float GetSystemTimeInSeconds(const bool &Reset)
{
	struct timespec			CurrentLocalMachineTime;

	clock_gettime( CLOCK_REALTIME, &CurrentLocalMachineTime);

	if ( (GetSystemTimeInSecondsCalledFirstTime) || (Reset) )
	{
		clock_gettime( CLOCK_REALTIME, &StoredSystemTimeInSeconds);
		GetSystemTimeInSecondsCalledFirstTime = false;
	}

	return((float)(((double)(CurrentLocalMachineTime.tv_sec
					-	StoredSystemTimeInSeconds.tv_sec))
					+	(double)(CurrentLocalMachineTime.tv_nsec
					-	StoredSystemTimeInSeconds.tv_nsec)
					*	(double)1e-9));
}