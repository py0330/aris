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

		int ReplaceVariable(ELEMENT* pEle);

		enum
		{
			DOUBLE_SIZE = 25,
		};

		int str2doubleArray(const char *str, double *doubleArray, unsigned int ArrayLength = 0);
		int doubleArray2str(char *str, const double *doubleArray, unsigned int ArrayLength = 0);
	}
}

#endif
