//////////////////////////////////////////////////////////////////////////////////////////
//	FPS_COUNTER.h
//	class to calculate frames per second
//	Downloaded from: www.paulsprojects.net
//	Created:	20th July 2002
//
//	Copyright (c) 2006, Paul Baker
//	Distributed under the New BSD Licence. (See accompanying file License.txt or copy at
//	http://www.paulsprojects.net/NewBSDLicense.txt)
//////////////////////////////////////////////////////////////////////////////////////////	

#ifndef FPSCOUNTER_H
#define FPSCOUNTER_H

class FpsCounter
{
public:
	void Update(void);									//updates counter - call once per frame
	void Shutdown(void);								//send max, min, average to log
	double GetFps(void)	{		return fps;		}
		
	FpsCounter() : fps(0.0), lastTime(0.0), frames(0), time(0.0) 
	{}
	~FpsCounter()	{}
	
protected:
	double fps;

	double lastTime;
	int frames;
	double time;
};

#endif	