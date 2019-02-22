#ifndef ARIS_DYNAMIC_MODEL_BASIC_
#define ARIS_DYNAMIC_MODEL_BASIC_

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>
#include <deque>
#include <type_traits>

#include <aris/core/core.hpp>
#include <aris/dynamic/matrix.hpp>
#include <aris/dynamic/screw.hpp>



namespace aris::dynamic
{
	using double6x6 = double[6][6];
	using double4x4 = double[4][4];
	using double3 = double[3];
	using double6 = double[6];
	using double7 = double[7];
	using double10 = double[10];

	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	class Model;

	class Element :public aris::core::Object
	{
	public:
		auto model()noexcept->Model& { return *ancestor<Model>(); }
		auto model()const noexcept->const Model& { return const_cast<std::decay_t<decltype(*this)> *>(this)->model(); }
		auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->aris::core::Matrix;
		auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, const aris::core::Matrix& default_value)const->aris::core::Matrix;
		auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n)const->aris::core::Matrix;
		auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n, const aris::core::Matrix& default_value)const->aris::core::Matrix;

		~Element() = default;
		explicit Element(const std::string &name = "element") :Object(name) {}
		ARIS_REGISTER_TYPE("Element");
		ARIS_DEFINE_BIG_FOUR(Element);
	};
	class DynEle : public Element
	{
	public:
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto active() const noexcept->bool { return active_; }
		auto activate(bool active = true)noexcept->void { active_ = active; }

		virtual ~DynEle() = default;
		explicit DynEle(const std::string &name, bool active = true) : Element(name), active_(active) {};
		ARIS_REGISTER_TYPE("DynEle");
		ARIS_DEFINE_BIG_FOUR(DynEle);

	private:
		bool active_;
	};

	class Environment final :public Element
	{
	public:
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto gravity()const noexcept->const double6& { return gravity_; }
		auto setGravity(const double *gravity)noexcept->void { s_vc(6, gravity, gravity_); }

		virtual ~Environment() = default;
		explicit Environment(const std::string &name = "dyn_ele") :Element(name) {}
		ARIS_REGISTER_TYPE("Environment");
		ARIS_DEFINE_BIG_FOUR(Environment);

	private:
		double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };
	};

	class Variable :public Element
	{
	public:
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual toString() const->std::string { return ""; }

		virtual ~Variable() = default;
		explicit Variable(const std::string &name = "variable") : Element(name) {}
		ARIS_REGISTER_TYPE("Variable");
		ARIS_DEFINE_BIG_FOUR(Variable);
	};
	template<typename VariableType> class VariableTemplate : public Variable
	{
	public:
		auto data()->VariableType& { return data_; }
		auto data()const->const VariableType& { return data_; }

		virtual ~VariableTemplate() = default;
		explicit VariableTemplate(const std::string &name = "variable_template", const VariableType &data = VariableType(), bool active = true) : Variable(name), data_(data) {}
		VariableTemplate(const VariableTemplate &other) = default;
		VariableTemplate(VariableTemplate &&other) = default;
		VariableTemplate& operator=(const VariableTemplate &other) = default;
		VariableTemplate& operator=(VariableTemplate &&other) = default;

	private:
		VariableType data_;
	};
	class MatrixVariable final : public VariableTemplate<aris::core::Matrix>
	{
	public:
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto virtual toString() const->std::string override { return data().toString(); }

		virtual ~MatrixVariable() = default;
		explicit MatrixVariable(const std::string &name = "matrix_variable", const aris::core::Matrix &data = aris::core::Matrix()) : VariableTemplate(name, data) {}
		ARIS_REGISTER_TYPE("MatrixVariable");
		ARIS_DEFINE_BIG_FOUR(MatrixVariable);
	};
	class StringVariable final : public VariableTemplate<std::string>
	{
	public:
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto virtual toString() const->std::string override { return data(); }

		virtual ~StringVariable() = default;
		explicit StringVariable(const std::string &name = "string_variable", const std::string &data = "") : VariableTemplate(name, data) {}
		ARIS_REGISTER_TYPE("StringVariable");
		ARIS_DEFINE_BIG_FOUR(StringVariable);
	};

	/// @}
}

#endif
