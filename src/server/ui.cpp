#include "aris/server/ui.hpp"

namespace aris::server
{
	auto InterfaceRoot::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		auto ins = doc_.RootElement()->DeepClone(xml_ele.GetDocument());
		xml_ele.Parent()->InsertAfterChild(&xml_ele, ins);
		xml_ele.Parent()->DeleteChild(&xml_ele);
	}
	auto InterfaceRoot::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		doc_.Clear();
		auto root = xml_ele.DeepClone(&doc_);
		doc_.InsertEndChild(root);
	}

}
