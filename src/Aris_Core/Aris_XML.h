#ifndef ARIS_XML_H_
#define ARIS_XML_H_

#include "tinyxml2.h"
#include <cstring>

namespace Aris
{
	namespace Core
	{
		typedef tinyxml2::XMLDocument DOCUMENT;
		typedef tinyxml2::XMLDeclaration DECLARATION;
		typedef tinyxml2::XMLNode NODE;
		typedef tinyxml2::XMLElement ELEMENT;
		typedef tinyxml2::XMLAttribute ATTRIBUTE;

		void ReplaceVariable(ELEMENT* pEle);
	}
}

#endif
