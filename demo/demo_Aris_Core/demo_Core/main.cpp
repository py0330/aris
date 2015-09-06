#include "Platform.h"

#ifdef PLATFORM_IS_WINDOWS
#define CRTDBG_MAP_ALLOC    
#include <stdlib.h>    
#include <crtdbg.h>    
#endif

#include "Aris_Core.h"
#include "Aris_XML.h"

#ifdef PLATFORM_IS_LINUX
#include <unistd.h>
#endif

#include <iostream>
#include <cstring>
using namespace std;

struct AAA
{
	int a;
	int b;
	double c;
};


int show(const Aris::Core::MSG &msg)
{
	cout << "Msg Length:" << msg.GetLength()<<endl;
	cout << "Msg MsgID :" << msg.GetMsgID()<<endl;
	//cout << "Msg Type  :" << msg.GetType()<<endl;
	cout << "Msg Data  :" << msg.GetDataAddress()<<endl<<endl;

	return 0;
}

using namespace Aris::Core;

int main()
{
	{
		Aris::Core::MSG m1, m2;

		cout << log("first log") << endl;
		MSG_BASE *p = new MSG;
		delete p;

		m1.SetLength(10);
		

		memcpy(m1.GetDataAddress(), "123456789", 10);

		m1.SetLength(12);

		m1.SetMsgID(101);
		//m1.SetType(1233);

		MSG m4(m1);

		m2 = m1;
		show(m1);

		Aris::Core::MSG m3(m1);

		show(m2);

		m3.SetLength(15);

		show(m3);

		Aris::Core::RT_MSG::instance[0].CopyMore("rt msg123", 10);

		cout << "1" << endl;

		//Aris::Core::RT_MSG::instance[0].CopyMore("rt msg123", 10);
		Aris::Core::RT_MSG::instance[1].CopyMore("98765", 6);
		cout << (char *)Aris::Core::RT_MSG::instance[0].GetDataAddress() << endl;
		cout << (char *)Aris::Core::RT_MSG::instance[1].GetDataAddress() << endl;
		cout << Aris::Core::RT_MSG::instance[0].GetLength() << endl;
		cout << Aris::Core::RT_MSG::instance[1].GetLength() << endl;


		AAA aaa = { 0, 0, 1 };
		AAA bbb = { 0, 0, 3.5 };

		AAA ccc;
		AAA ddd;

		MSG m;

		m.CopyStruct(aaa, bbb, aaa);
		m.CopyStruct(aaa, bbb, aaa);
		m.PasteStruct(ccc, ccc, ddd);

		cout << "ccc:" << endl << "    " << ccc.a << endl << "    " << ccc.b << endl << "    " << ccc.c << endl;
		cout << "ddd:" << endl << "    " << ddd.a << endl << "    " << ddd.b << endl << "    " << ddd.c << endl;

		cout << (char*)m.GetDataAddress() << endl;








		int i;
		cin >> i;
	}


	Aris::Core::DOCUMENT doc;

	Aris::Core::ELEMENT *ele=doc.NewElement("test");



	ele->SetAttribute("boolVar", false);


	


	doc.InsertFirstChild(ele);

	doc.SaveFile("test.xml");




#ifdef PLATFORM_IS_WINDOWS
	_CrtDumpMemoryLeaks();
#endif

	return 0;
}
