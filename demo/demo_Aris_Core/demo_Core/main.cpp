

#ifdef WIN32
#define CRTDBG_MAP_ALLOC    
#include <stdlib.h>    
#include <crtdbg.h>    
#endif

#include "aris_core.h"
#include "aris_core_xml.h"

#ifdef UNIX
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


int show(const aris::core::Msg &msg)
{
	cout << "Msg Length:" << msg.size()<<endl;
	cout << "Msg MsgID :" << msg.msgID()<<endl;
	//cout << "Msg Type  :" << msg.GetType()<<endl;
	cout << "Msg Data  :" << msg.data()<<endl<<endl;

	return 0;
}

using namespace aris::core;

template<class... Args>
void test(Args... args);
void test();

template<class FirstArg, class... Args>
class test_class
{
public:
	static void test(FirstArg firstArg, Args... args)
	{
		std::cout << "test::" << firstArg << std::endl;
		::test(args...);
	};
};

template<class FirstArg, class... Args>
class test_class<const FirstArg *,Args...>
{
public:
	static void test(const FirstArg *firstArg, Args... args)
	{
		std::cout << "test::" << *firstArg << std::endl;
		::test(args...);
	};
};

template<class... Args>
class test_class<char [], Args...>
{
public:
	static void test(char firstArg[], Args... args)
	{
		std::cout << "test::char::" << firstArg << std::endl;
		::test(args...);
	};
};

template<class... Args>
void test(Args... args)
{
	test_class<Args...>::test(args...);
}

void test()
{
	std::cout << "test end" << std::endl;
}


int main()
{
	/*{
		aris::core::Msg m1, m2;

		cout << log("first log") << endl;
		MsgBase *p = new Msg;
		delete p;

		m1.SetLength(10);
		

		memcpy(m1.data(), "123456789", 10);

		m1.SetLength(12);

		m1.setMsgID(101);
		//m1.setType(1233);

		Msg m4(m1);

		m2 = m1;
		show(m1);

		aris::core::Msg m3(m1);

		show(m2);

		m3.SetLength(15);

		show(m3);

		aris::core::MsgRT::instance[0].copyMore("rt msg123", 10);

		cout << "1" << endl;

		//aris::core::MsgRT::instance[0].copyMore("rt msg123", 10);
		aris::core::MsgRT::instance[1].copyMore("98765", 6);
		cout << (char *)aris::core::MsgRT::instance[0].data() << endl;
		cout << (char *)aris::core::MsgRT::instance[1].data() << endl;
		cout << aris::core::MsgRT::instance[0].GetLength() << endl;
		cout << aris::core::MsgRT::instance[1].GetLength() << endl;


		AAA aaa = { 0, 0, 1 };
		AAA bbb = { 0, 0, 3.5 };

		AAA ccc;
		AAA ddd;

		Msg m;

		m.copyStruct(aaa, bbb, aaa);
		m.copyStruct(aaa, bbb, aaa);
		m.pasteStruct(ccc, ccc, ddd);

		cout << "ccc:" << endl << "    " << ccc.a << endl << "    " << ccc.b << endl << "    " << ccc.c << endl;
		cout << "ddd:" << endl << "    " << ddd.a << endl << "    " << ddd.b << endl << "    " << ddd.c << endl;

		cout << (char*)m.data() << endl;








		int i;
		cin >> i;
	}*/

	double i = 0.11;
	const double *p = &i;

	char b[4] = "222";
	//char *c = b;
	test("222");
	//test_class<int, int, int>::test(1, 1, 1);
	//test(1, 1.0, 2.11, &i, "end", "end?");
	//test(1, 0);

#ifdef WIN32
	_CrtDumpMemoryLeaks();
#endif

	char a;
	std::cin >> a;

	return 0;
}
