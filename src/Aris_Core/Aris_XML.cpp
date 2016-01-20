#include <string>
#include <iostream>
#include <functional>

#include "aris_xml.h"

using namespace std;

namespace Aris
{
	namespace Core
	{
		void ReplaceVariable(XmlElement* pEle)
		{
			/*递归函数，用来访问pEle下所有元素*/
			function<void(XmlElement*, function<void(XmlElement*)>)>
				recursiveFunc = [&recursiveFunc](XmlElement* pEle, function<void(XmlElement*)> pFunc)->void
			{
				if (pFunc != nullptr)
				{
					pFunc(pEle);
				}

				for (XmlElement* p = pEle->FirstChildElement();
					p != 0;
					p = p->NextSiblingElement())
				{
					recursiveFunc(p, pFunc);
				}
			};
			
			/*以下用来将内容替换成变量中内容*/
			for (XmlElement* p = pEle->FirstChildElement("Variable")->FirstChildElement();
				p != 0;
				p=p->NextSiblingElement())
			{
				std::string vName, vValue;
				
				vName = string("$(") + string(p->Name()) + string(")");
				vValue = std::string(p->GetText());
				
				recursiveFunc(pEle, [vName,vValue](XmlElement* p)->void
				{
					const char* text;
					if ((text=p->GetText()) != nullptr)
					{
						string str(text);
						std::size_t pos;
						while ((pos = str.find(vName)) != str.npos)
						{
							str.replace(pos, vName.length(), vValue);
						}
						p->SetText(str.c_str());
					}
				});
				
			}
		}
	}
}