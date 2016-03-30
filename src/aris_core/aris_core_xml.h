#ifndef ARIS_CORE_XML_H_
#define ARIS_CORE_XML_H_

#include <tinyxml2.h>
#include <cstring>

namespace aris
{
	namespace core
	{
		typedef tinyxml2::XMLDocument XmlDocument;
		typedef tinyxml2::XMLDeclaration XmlDeclaration;
		typedef tinyxml2::XMLNode XmlNode;
		typedef tinyxml2::XMLElement XmlElement;
		typedef tinyxml2::XMLAttribute XmlAttribute;
	}
}

#endif
