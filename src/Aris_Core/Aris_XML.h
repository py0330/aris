#ifndef ARIS_XML_H_
#define ARIS_XML_H_

#include "tinyxml2.h"
#include <cstring>

namespace Aris
{
	namespace Core
	{
		typedef tinyxml2::XMLDocument XmlDocument;
		typedef tinyxml2::XMLDeclaration XmlDeclaration;
		typedef tinyxml2::XMLNode XmlNode;
		typedef tinyxml2::XMLElement XmlElement;
		typedef tinyxml2::XMLAttribute XmlAttribute;

		void ReplaceVariable(XmlElement* pEle);
	}
}

#endif
