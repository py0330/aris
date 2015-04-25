#include<iostream>


using namespace std;

#include "tinyxml2.h"
#include "Aris_XML.h"

using namespace tinyxml2;


const char * f()
{
	const char * a = "12345asdv";
	char *b = "12345";
	b[0] = 1;
	return a;
}

int main()
{
	static const char* xml =
		"<?xml version=\"1.0\" encoding=\"UTF - 8\" ?>"
		"<!--DOCTYPE PLAY SYSTEM \"play.dtd\"-->"
		"<PLAY>"
		"<TITLE>  1.0 2.0 3.0  \n\n"
		"  4.00   5.01   6.002   </TITLE>"
		"</PLAY>";

	::XMLDocument doc,doc2;
	cout<<"num:"<<doc.Parse(xml)<<endl;

	doc.SaveFile("myxml.xml");

	doc2.LoadFile("myxml.xml");

	::XMLElement *ele=doc2.FirstChildElement();
	
	const char *ch= (ele->FirstChildElement()->GetText());
	cout << ch << endl;

	double a[10];
	//int length = strlen(ch),n=0;
	//for (int i = 0; i < length; ++i)
	//{
	//	if (((ch[i] == ' ') || (ch[i] == '\n') || (i == 0)) && ((ch[i + 1] != ' ') && (ch[i + 1] != '\n') && (ch[i + 1] != '\0')))
	//	{
	//		//sscanf_s(&ch[i], "%lf", &a[n]);
	//		a[n] = atof(&ch[i]);
	//		n++;
	//	}
	//}
	int n=Aris::Core::str2doubleArray(ch, a);

	for (int i = 0; i < n; ++i)
	{
		cout << a[i]<<endl;
	}


	ch = &ch[10];
	

	Aris::Core::DOCUMENT d1,d2;
	d1.LoadFile("myxml.xml");
	
	Aris::Core::ELEMENT* t1 = d1.NewElement("newElement");
	
	d1.RootElement()->InsertEndChild(t1);

	d1.SaveFile("myxml2.xml");

	char chh[100];

	sprintf(chh, "%-20.10e", 1.256478542321314e25);
	cout << chh << endl;
	sprintf(chh+20, "%-20.10e", -1.256478542321314e25);
	cout << chh << endl;
	sprintf(chh+40, "%-20.10e", 1.256478542321314e25);
	cout << chh << endl;
	cout << chh << endl;

	n = Aris::Core::str2doubleArray(chh, a);
	for (int i = 0; i < n; ++i)
	{
		cout << a[i] << endl;
	}

	char aaaa;
	cin >> aaaa;

	return 0;
}