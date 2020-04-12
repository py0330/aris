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

#include "aris/core/core.hpp"
#include "aris/dynamic/matrix.hpp"
#include "aris/dynamic/screw.hpp"
#include "aris/dynamic/model.hpp"

namespace aris::dynamic
{
	auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->aris::core::Matrix
	{
		std::string error = "failed to get Matrix attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		aris::core::Matrix mat;
		try
		{
			static aris::core::Calculator c;
			static int i = 0;
			if (i == 0)
			{
				c.addVariable("PI", "Number", double(aris::PI));
				i = 1;
			}
			//auto mat = c.calculateExpression(xml_ele.GetText() + "})").second;
			auto ret = c.calculateExpression(std::string("Matrix(") + xml_ele.Attribute(attribute_name.c_str()) + ")");
			mat = std::any_cast<aris::core::Matrix&>(ret.second);
		}
		catch (std::exception &e)
		{
			std::cout << xml_ele.Attribute(attribute_name.c_str()) << std::endl;
			
			
			static aris::core::Calculator c;
			static int i = 0;
			if (i == 0)
			{
				c.addVariable("PI", "Number", double(aris::PI));
				i = 1;
			}
			auto ret = c.calculateExpression(std::string("Matrix(") + xml_ele.Attribute(attribute_name.c_str()) + ")");
			
			
			THROW_FILE_LINE(error + "failed to evaluate matrix:" + e.what());
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
			THROW_FILE_LINE(error + "matrix has wrong dimensions, it's dimentsion should be \"" + std::to_string(m) + "," + std::to_string(n)
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
		Object::loadXml(xml_ele);
		std::copy_n(attributeMatrix(xml_ele, "gravity", 1, 6).data(), 6, gravity_);
	}

	auto Variable::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		Element::saveXml(xml_ele);
		xml_ele.SetText(this->toString().c_str());
	}

	auto MatrixVariable::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Variable::loadXml(xml_ele);
		static aris::core::Calculator c;
		static int i = 0;
		if (i == 0)
		{
			c.addVariable("PI", "Number", double(3.141592653));
			i = 1;
		}

		auto mat = c.calculateExpression(std::string("Matrix({") + xml_ele.GetText() + "})").second;
		data() = std::any_cast<const aris::core::Matrix&>(mat);



		//model()->calculator().addVariable(name(), "Matrix", data());
	}
	auto StringVariable::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Variable::loadXml(xml_ele);
		data() = std::string(xml_ele.GetText());
		//model()->calculator().addVariable(name(), "String", data());
	}
}
