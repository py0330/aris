#ifndef ARIS_DYNAMIC_MODEL_BASIC_H_
#define ARIS_DYNAMIC_MODEL_BASIC_H_

#include <aris/core/object.hpp>
#include <aris/core/expression_calculator.hpp>
#include <aris/dynamic/screw.hpp>
#include <aris/dynamic/model_base.hpp>

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

	class ARIS_API Element{
	public:
		auto model()noexcept->Model* { return model_; }
		auto model()const noexcept->const Model* { return const_cast<std::decay_t<decltype(*this)>*>(this)->model(); }
		auto id()const->Size { return id_; }
		auto resetModel(Model* m) { model_ = m; }

		~Element() = default;
		Element() = default;
		ARIS_DEFINE_BIG_FOUR(Element);

	private:
		Model* model_{ nullptr };
		Size id_{ 0 };
		friend class Model;
	};
	class ARIS_API DynEle : public Element{
	public:
		auto active() const noexcept->bool { return active_; }
		auto activate(bool active = true)noexcept->void;
		auto name()const->std::string { return name_; }
		auto setName(const std::string &name)->void { name_ = name; }

		virtual ~DynEle() = default;
		explicit DynEle(const std::string &name = "dyn_ele", bool active = true) : active_(active), name_(name) { };
		ARIS_DEFINE_BIG_FOUR(DynEle);

	private:
		std::string name_;
		bool active_;
	};

	class ARIS_API Environment final{
	public:
		auto gravity()const noexcept->const double6& { return gravity_; }
		auto setGravity(const double *gravity)noexcept->void { s_vc(6, gravity, gravity_); }

		virtual ~Environment() = default;
		explicit Environment(){}
		ARIS_DEFINE_BIG_FOUR(Environment);

	private:
		double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };
	};
	class ARIS_API Variable :public Element{
	public:
		auto virtual toString() const->std::string { return ""; }
		auto virtual fromString(std::string_view str)->void {}
		auto name()const->std::string { return name_; }
		auto setName(const std::string &name)->void { name_ = name; }

		virtual ~Variable() = default;
		explicit Variable(const std::string &name = "variable"){ setName(name); }
		ARIS_DEFINE_BIG_FOUR(Variable);
	private:
		std::string name_;
	};
	
	template<typename VariableType> class VariableTemplate : public Variable
	{
	public:
		auto data()->VariableType& { return data_; }
		auto data()const->const VariableType& { return data_; }

		virtual ~VariableTemplate() = default;
		explicit VariableTemplate(const std::string &name = "variable_template", const VariableType &data = VariableType(), bool active = true) : Variable(name), data_(data) {}
		VariableTemplate(const VariableTemplate &other)noexcept = default;
		VariableTemplate(VariableTemplate &&other)noexcept = default;
		VariableTemplate& operator=(const VariableTemplate &other)noexcept = default;
		VariableTemplate& operator=(VariableTemplate &&other)noexcept = default;

	private:
		VariableType data_;
	};
	class ARIS_API MatrixVariable final : public VariableTemplate<aris::core::Matrix> {
	public:
		auto virtual toString() const->std::string override { return data().toString(); }
		auto virtual fromString(std::string_view str)->void override;

		virtual ~MatrixVariable() = default;
		explicit MatrixVariable(const std::string &name = "matrix_variable", const aris::core::Matrix &data = aris::core::Matrix()) : VariableTemplate(name, data) {}
		ARIS_DEFINE_BIG_FOUR(MatrixVariable);
	};
	class ARIS_API StringVariable final : public VariableTemplate<std::string> {
	public:
		auto virtual toString() const->std::string override { return data(); }

		virtual ~StringVariable() = default;
		explicit StringVariable(const std::string &name = "string_variable", const std::string &data = "") : VariableTemplate(name, data) {}
		ARIS_DEFINE_BIG_FOUR(StringVariable);
	};

	/// @}
}

#endif
