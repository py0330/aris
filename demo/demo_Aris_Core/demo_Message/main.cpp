#include "Platform.h"

#include "Aris_Thread.h"
#include "Aris_Message.h"

#include <stdio.h>

#ifdef PLATFORM_IS_LINUX
#include <unistd.h>
#endif
#ifdef PLATFORM_IS_WINDOWS
#include<windows.h>
#endif


#include <iostream>
#include <cstring>
using namespace std;

void* Thread1(void *);
void* Thread2(void *);
void* Thread3(void *);
void* Thread4(void *);
int CallBack1(Aris::Core::MSG &);
int CallBack2(Aris::Core::MSG &);
int CallBack3(Aris::Core::MSG &);
int CallBack4(Aris::Core::MSG &);
int CallBackDefault(Aris::Core::MSG &);

using namespace Aris::Core;

Aris::Core::EVENT stopEvent;

int main()
{
	Aris::Core::THREAD T1, T2, T3,T4;

	T1.SetFunction(Thread1);
	T2.SetFunction(Thread2);
	T3.SetFunction(Thread3);
	T4.SetFunction(Thread4);

	T1.Start(0);
	T2.Start(0);
	T3.Start(0);
	T4.Start(0);

	Aris::Core::MSG Msg(0,0);
	
	Aris::Core::RegisterMsgCallback(0, nullptr);
	Aris::Core::RegisterMsgCallback(1, CallBack1);
	Aris::Core::RegisterMsgCallback(2, CallBack2);
	Aris::Core::RegisterMsgCallback(3, nullptr);
	Aris::Core::RegisterMsgCallback(3, CallBack3);
	Aris::Core::RegisterDefaultCallback(CallBackDefault);

	Aris::Core::RunMsgLoop();

	int i;
	cin >> i;

	return 0;
}

void* Thread1(void *in)
{
	static int i = 0;
	
	while (1)
	{
#ifdef PLATFORM_IS_WINDOWS
		Sleep(1000);
#endif
#ifdef PLATFORM_IS_LINUX
		usleep(1000000);
#endif
		Aris::Core::MSG Msg(i,10);

		sprintf(Msg.GetDataAddress(), "%d is good", i);

		Aris::Core::PostMsg(Msg);
		i++;
		if (i > 4)
			i -= 4;
	}

	return 0;
}

void* Thread2(void *in)
{
	static int i = 0;

	while (1)
	{
#ifdef PLATFORM_IS_WINDOWS
		Sleep(5000);
#endif
#ifdef PLATFORM_IS_LINUX
		usleep(5000000);
#endif
		Aris::Core::RegisterMsgCallback(3, CallBack4);
	}

	return 0;
}

void* Thread3(void *in)
{
	while (1)
	{
#ifdef PLATFORM_IS_WINDOWS
		Sleep(15000);
#endif
#ifdef PLATFORM_IS_LINUX
		usleep(15000000);
#endif
		
		stopEvent.SetEvent();
	}

	return 0;
}

void* Thread4(void *in)
{
	while (1)
	{
		stopEvent.WaitForEvent();
		stopEvent.ResetEvent();

		cout<<"Now stop MSG loop"<<endl;
		StopMsgLoop();

	}

	return 0;
}

int CallBack1(Aris::Core::MSG &in)
{
	cout << "Message:" << in.GetMsgID() << "   Callback:1   Data:" << in.GetDataAddress() << endl;
	return 0;
}

int CallBack2(Aris::Core::MSG &in)
{
	cout << "Message:" << in.GetMsgID() << "   Callback:2   Data:" << in.GetDataAddress() << endl;
	return 0;
}

int CallBack3(Aris::Core::MSG &in)
{
	cout << "Message:" << in.GetMsgID() << "   Callback:3   Data:" << in.GetDataAddress() << endl;
	return 0;
}

int CallBack4(Aris::Core::MSG &in)
{
#ifdef PLATFORM_IS_WINDOWS
	Sleep(5000);
#endif
#ifdef PLATFORM_IS_LINUX
	usleep(5000000);
#endif
	cout << "Message:" << in.GetMsgID() << "   Callback:4   Data:" << in.GetDataAddress() << endl;
	return 0;
}

int CallBackDefault(Aris::Core::MSG &in)
{
	cout << "Message:" << in.GetMsgID() << "   Callback:0   Data:" << in.GetDataAddress() << endl;
	return 0;
}
