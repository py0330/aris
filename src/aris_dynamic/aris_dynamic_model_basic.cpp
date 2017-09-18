#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>
#include <limits>
#include <type_traits>
#include <ios>

#include "aris_core.h"
#include "aris_dynamic_matrix.h"
#include "aris_dynamic_screw.h"
#include "aris_dynamic_model.h"

namespace aris
{
	namespace dynamic
	{
		auto Element::model()->Model& { return dynamic_cast<Model&>(root()); }
		auto Element::model()const->const Model&{ return dynamic_cast<const Model&>(root()); }
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->aris::core::Matrix
		{
			std::string error = "failed to get Matrix attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

			aris::core::Matrix mat;
			try
			{
				mat = this->model().calculator().calculateExpression(xml_ele.Attribute(attribute_name.c_str()));
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "failed to evaluate matrix:" + e.what());
			}

			return mat;
		}
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, const aris::core::Matrix& default_value)const->aris::core::Matrix
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeMatrix(xml_ele, attribute_name) : default_value;
		}
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n)const->aris::core::Matrix
		{
			std::string error = "failed to get Matrix attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

			aris::core::Matrix mat = attributeMatrix(xml_ele, attribute_name);

			if (mat.m() != m || mat.n() != n)
			{
				throw std::runtime_error(error + "matrix has wrong dimensions, it's dimentsion should be \"" + std::to_string(m) + "," + std::to_string(n)
					+ "\", while the real value is \"" + std::to_string(mat.m()) + "," + std::to_string(mat.n()) + "\"");
			}

			return mat;
		}
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n, const aris::core::Matrix& default_value)const->aris::core::Matrix
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeMatrix(xml_ele, attribute_name, m, n) : default_value;
		}

		auto DynEle::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);
			xml_ele.SetAttribute("active", active() ? "true" : "false");
		}
		auto DynEle::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			active_ = attributeBool(xml_ele, "active", true);
			Element::loadXml(xml_ele);
		}

		auto Environment::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);
			xml_ele.SetAttribute("gravity", core::Matrix(1, 6, gravity_).toString().c_str());
		}
		auto Environment::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			std::copy_n(attributeMatrix(xml_ele, "gravity", 1, 6).data(), 6, gravity_);
			Object::loadXml(xml_ele);
		}

		auto Variable::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);
			xml_ele.SetText(this->toString().c_str());
		}

		auto MatrixVariable::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			data_ = model().calculator().calculateExpression(xml_ele.GetText());
			Variable::loadXml(xml_ele);
			model().calculator().addVariable(name(), data_);
		}
		auto StringVariable::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			data_ = std::string(xml_ele.GetText());
			model().calculator().addVariable(name(), data_);
			Variable::loadXml(xml_ele);
		}
	}
}
