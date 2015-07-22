#include "Aris_XML.h"
#include <string>
#include <iostream>
#include <functional>

using namespace std;

namespace Aris
{
	namespace Core
	{
		int ReplaceVariable(ELEMENT* pEle)
		{
			/*递归函数，用来访问pEle下所有元素*/
			function<void(ELEMENT*, function<void(ELEMENT*)>)>
				recursiveFunc = [&recursiveFunc](ELEMENT* pEle, function<void(ELEMENT*)> pFunc)->void
			{
				if (pFunc != nullptr)
				{
					pFunc(pEle);
				}

				for (ELEMENT* p = pEle->FirstChildElement();
					p != 0;
					p = p->NextSiblingElement())
				{
					recursiveFunc(p, pFunc);
				}
			};
			
			/*以下用来将内容替换成变量中内容*/
			for (ELEMENT* p = pEle->FirstChildElement("Variable")->FirstChildElement();
				p != 0;
				p=p->NextSiblingElement())
			{
				std::string vName, vValue;
				
				vName = string("$(") + string(p->Name()) + string(")");
				vValue = std::string(p->GetText());
				
				recursiveFunc(pEle, [vName,vValue](ELEMENT* p)->void
				{
					const char* text;
					if ((text=p->GetText()) != nullptr)
					{
						string str(text);
						int pos;
						while ((pos = str.find(vName)) != str.npos)
						{
							str.replace(pos, vName.length(), vValue);
						}
						p->SetText(str.c_str());
					}
				});
				
			}
			return 0;
		}
	}
}