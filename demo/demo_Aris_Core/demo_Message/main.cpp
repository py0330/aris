#include "Platform.h"

#include <thread>

#include "Aris_Message.h"

#include <stdio.h>


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

//Aris::Core::EVENT stopEvent;

int main()
{

	return 0;
}

