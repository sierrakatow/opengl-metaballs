//////////////////////////////////////////////////////////////////////////////////////////
//	FPS_COUNTER.cpp
//	functions to calculate frames per second
//	Downloaded from: www.paulsprojects.net
//	Created:	20th July 2002
//
//	Copyright (c) 2006, Paul Baker
//	Distributed under the New BSD Licence. (See accompanying file License.txt or copy at
//	http://www.paulsprojects.net/NewBSDLicense.txt)
//////////////////////////////////////////////////////////////////////////////////////////	
//#include <windows.h>
//#include "LOG.h"
#include "fpscounter.h"

void FpsCounter::Update(void)
{
	//keep track of time lapse and frame count
	time = timeGetTime()*0.001;							//get current time in seconds
	++frames;												//increase frame count
		
	if(time-lastTime>1.0)									//if it has been 1 second
	{
		fps		= frames/(time-lastTime);					//update fps number
		lastTime= time;										//set beginning count
		frames	= 0;										//reset frames this second
	}
}
