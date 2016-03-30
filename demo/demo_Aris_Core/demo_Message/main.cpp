#include <thread>

#include "aris_core_msg_loop.h"

#include <stdio.h>


#include <iostream>
#include <cstring>
using namespace std;

void* Thread1(void *);
void* Thread2(void *);
void* Thread3(void *);
void* Thread4(void *);
int CallBack1(aris::core::Msg &);
int CallBack2(aris::core::Msg &);
int CallBack3(aris::core::Msg &);
int CallBack4(aris::core::Msg &);
int CallBackDefault(aris::core::Msg &);

using namespace aris::core;

//aris::core::EVENT stopEvent;

int main()
{

	return 0;
}

