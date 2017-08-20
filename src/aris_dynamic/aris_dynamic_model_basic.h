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

#include <aris_core.h>
#include <aris_dynamic_matrix.h>
#include <aris_dynamic_block_matrix.h>
#include <aris_dynamic_screw.h>

namespace aris
{
	namespace dynamic
	{
		using double6x6 = double[6][6];
		using double4x4 = double[4][4];
		using double3 = double[3];
		using double6 = double[6];
		using double7 = double[7];

		class Model;

		class Element :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Element" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto model()->Model&;
			auto model()const->const Model&;

			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->aris::core::Matrix;
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, const aris::core::Matrix& default_value)const->aris::core::Matrix;
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n)const->aris::core::Matrix;
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n, const aris::core::Matrix& default_value)const->aris::core::Matrix;

		protected:
			~Element() = default;
			explicit Element(const std::string &name) :Object(name) {}
			explicit Element(Object &father, const aris::core::XmlElement &xml_ele) :Object(father, xml_ele) {}
			Element(const Element&) = default;
			Element(Element&&) = default;
			Element& operator=(const Element&) = default;
			Element& operator=(Element&&) = default;
		};
		class DynEle : public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "DynEle" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto active() const->bool { return active_; }
			auto activate(bool active = true)->void { active_ = active; }

		protected:
			virtual ~DynEle() = default;
			explicit DynEle(const std::string &name, bool active = true) : Element(name), active_(active) {};
			explicit DynEle(Object &father, const aris::core::XmlElement &xml_ele);
			DynEle(const DynEle &) = default;
			DynEle(DynEle &&) = default;
			DynEle& operator=(const DynEle &) = default;
			DynEle& operator=(DynEle &&) = default;

		private:
			bool active_;
		};
		
		class Environment final :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Environment" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto gravity()const ->const double6&{ return gravity_; }

		private:
			auto virtual operator=(const Object &other)->Object& { return dynamic_cast<Environment&>(*this) = dynamic_cast<const Environment&>(other); }
			auto virtual operator=(Object &&other)->Object& { return dynamic_cast<Environment&>(*this) = dynamic_cast<Environment&&>(other); }

			virtual ~Environment() = default;
			explicit Environment(Object &father, const aris::core::XmlElement &xml_ele);
			explicit Environment(const std::string &name) :Element(name) {}
			Environment(const Environment &) = default;
			Environment(Environment &&) = default;
			Environment &operator=(const Environment &) = default;
			Environment &operator=(Environment &&) = default;

		private:
			double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};

		class Variable :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Variable" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual toString() const->std::string { return ""; }

		protected:
			virtual ~Variable() = default;
			explicit Variable(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele) {}
			explicit Variable(const std::string &name) : Element(name) {}
			Variable(const Variable&) = default;
			Variable(Variable&&) = default;
			Variable& operator=(const Variable&) = default;
			Variable& operator=(Variable&&) = default;

			friend class Model;
			friend class aris::core::Root;
		};
		template<typename VariableType> class VariableTemplate : public Variable
		{
		public:
			auto data()->VariableType& { return data_; }
			auto data()const->const VariableType&{ return data_; }

		protected:
			explicit VariableTemplate(const std::string &name, const VariableType &data, bool active = true) : Variable(name), data_(data) {}
			explicit VariableTemplate(Object &father, const aris::core::XmlElement &xml_ele) : Variable(father, xml_ele) {}
			VariableTemplate(const VariableTemplate &other) = default;
			VariableTemplate(VariableTemplate &&other) = default;
			VariableTemplate& operator=(const VariableTemplate &other) = default;
			VariableTemplate& operator=(VariableTemplate &&other) = default;

			VariableType data_;
			friend class Model;
		};
		class MatrixVariable final : public VariableTemplate<aris::core::Matrix>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "MatrixVariable" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual toString() const->std::string override { return data_.toString(); }

		private:
			virtual ~MatrixVariable() = default;
			explicit MatrixVariable(const std::string &name, const aris::core::Matrix &data) : VariableTemplate(name, data) {}
			explicit MatrixVariable(Object &father, const aris::core::XmlElement &xml_ele);
			MatrixVariable(const MatrixVariable &other) = default;
			MatrixVariable(MatrixVariable &&other) = default;
			MatrixVariable& operator=(const MatrixVariable &other) = default;
			MatrixVariable& operator=(MatrixVariable &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class StringVariable final : public VariableTemplate<std::string>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "StringVariable" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual toString() const->std::string override { return data_; }

		private:
			virtual ~StringVariable() = default;
			explicit StringVariable(const std::string &name, const std::string &data) : VariableTemplate(name, data) {}
			explicit StringVariable(Object &father, const aris::core::XmlElement &xml_ele);
			StringVariable(const StringVariable &other) = default;
			StringVariable(StringVariable &&other) = default;
			StringVariable& operator=(const StringVariable &other) = default;
			StringVariable& operator=(StringVariable &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
	}
}

#endif
