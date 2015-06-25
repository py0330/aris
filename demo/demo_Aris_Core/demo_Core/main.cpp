#include "Platform.h"

#include "Aris_Core.h"

#ifdef PLATFORM_IS_LINUX
#include <unistd.h>
#endif

#include <iostream>
#include <cstring>
using namespace std;

int show(const Aris::Core::MSG &msg)
{
	cout << "Msg Length:" << msg.GetLength()<<endl;
	cout << "Msg MsgID :" << msg.GetMsgID()<<endl;
	//cout << "Msg Type  :" << msg.GetType()<<endl;
	cout << "Msg Data  :" << msg.GetDataAddress()<<endl<<endl;

	return 0;
}


int main()
{
	Aris::Core::MSG m1, m2;

	m1.SetLength(10);
	
	memcpy(m1.GetDataAddress(), "12345679", 10);

	m1.SetLength(12);

	


	m1.SetMsgID(101);
	//m1.SetType(1233);

	m2 = m1;

	show(m1);
	
	Aris::Core::MSG m3(m1);

	show(m2);

	m3.SetLength(15);

	show(m3);

	Aris::Core::RT_MSG::instance[0].CopyMore("rt msg123", 10);
	Aris::Core::RT_MSG::instance[0].CopyMore("rt msg123", 10);
	Aris::Core::RT_MSG::instance[1].CopyMore("98765", 6);
	cout << (char *)Aris::Core::RT_MSG::instance[0].GetDataAddress() << endl;
	cout << (char *)Aris::Core::RT_MSG::instance[1].GetDataAddress() << endl;
	cout << Aris::Core::RT_MSG::instance[0].GetLength()<<endl;
	cout << Aris::Core::RT_MSG::instance[1].GetLength() << endl;

	int i;
	cin >> i;
	
	return 0;
}
