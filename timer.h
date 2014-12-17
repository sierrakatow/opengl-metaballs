//////////////////////////////////////////////////////////////////////////////////////////
//	TIMER.h
//	timer class
//	Downloaded from: www.paulsprojects.net
//	Created:	20th July 2002
//
//	Copyright (c) 2006, Paul Baker
//	Distributed under the New BSD Licence. (See accompanying file License.txt or copy at
//	http://www.paulsprojects.net/NewBSDLicense.txt)
//////////////////////////////////////////////////////////////////////////////////////////	

#ifndef TIMER_H
#define TIMER_H

// #include <windows.h>
// #include <mmsystem.h>

class Timer
{
public:
	Timer() : isPaused(false)
	{	Reset();	}
	~Timer()	{}

	void Reset();
	double GetTime();
	void Pause();
	void Unpause();

protected:
	bool isPaused;
	double pauseTime;
	double startTime;
};

#endif