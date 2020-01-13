#ifndef ARIS_CORE_EXPRESSION_CALCULATOR_H_
#define ARIS_CORE_EXPRESSION_CALCULATOR_H_

#include <map>
#include <functional>
#include <string>
#include <iostream>
#include <list>
#include <any>

#include "aris/core/basic_type.hpp"
#include "aris/core/log.hpp"
#include "aris/core/object.hpp"

namespace aris::core
{
	class Matrix
	{
	public:
		auto swap(Matrix &other)->Matrix&;
		auto empty() const->bool { return data_vec_.empty(); }
		auto size() const->Size { return m()*n(); }
		auto data()->double * { return data_vec_.data(); }
		auto data() const->const double * { return data_vec_.data(); }
		auto begin() ->double * { return data(); }
		auto begin() const ->const double * { return data(); }
		auto end() ->double * { return data() + size(); }
		auto end()  const ->const double * { return data() + size(); }
		auto m() const->Size { return m_; }
		auto n() const->Size { return n_; }
		auto resize(Size m, Size n)->Matrix &;
		auto transpose()->Matrix &;

		auto copySubMatrixTo(const Matrix &subMat, Size beginRow, Size beginCol, Size rowNum, Size colNum)->void;
		auto copySubMatrixTo(const Matrix &subMat, Size beginRow, Size beginCol)->void { copySubMatrixTo(subMat, beginRow, beginCol, subMat.m(), subMat.n()); }

		auto toString() const->std::string;
		auto toDouble() const->double { return data()[0]; }
		auto dsp() const ->void { std::cout << this->toString(); }

		auto operator()(Size i, Size j)->double & { return is_row_major_ ? data()[i*n() + j] : data()[j*m() + i]; }
		auto operator()(Size i, Size j) const->const double & { return is_row_major_ ? data()[i*n() + j] : data()[j*m() + i]; }

		friend auto operator + (const Matrix &m1, const Matrix &m2)->Matrix;
		friend auto operator - (const Matrix &m1, const Matrix &m2)->Matrix;
		friend auto operator * (const Matrix &m1, const Matrix &m2)->Matrix;
		friend auto operator / (const Matrix &m1, const Matrix &m2)->Matrix;
		friend auto operator - (const Matrix &m1)->Matrix;
		friend auto operator + (const Matrix &m1)->Matrix;

		template <typename MATRIX_LIST>
		friend auto combineColMatrices(const MATRIX_LIST &matrices)->Matrix;
		template <typename MATRIX_LIST>
		friend auto combineRowMatrices(const MATRIX_LIST &matrices)->Matrix;
		template <typename MATRIX_LISTLIST>
		friend auto combineMatrices(const MATRIX_LISTLIST &matrices)->Matrix;

		~Matrix() {}
		Matrix(double value);
		Matrix(Size m, Size n, double value = 0);
		Matrix(Size m, Size n, const double *data);
		Matrix(const std::initializer_list<Matrix> &data);
		Matrix() :m_(0), n_(0), is_row_major_(true) {}
		Matrix(const Matrix &other) = default;
		Matrix(Matrix &&other) { this->swap(other); }
		Matrix &operator=(Matrix other) { this->swap(other); return *this; }

	private:
		Size m_, n_;
		bool is_row_major_;
		std::vector<double> data_vec_;
	};

	template <typename MATRIX_LIST>
	Matrix combineColMatrices(const MATRIX_LIST &matrices)
	{
		Matrix ret;

		// 获取全部矩阵的行数和列数 //
		for (const auto &mat : matrices)
		{
			if (mat.size() == 0)continue;
			if ((ret.n() != 0) && (ret.n() != mat.n()))THROW_FILE_LINE("input do not have valid size");

			ret.m_ += mat.m();
			ret.n_ = mat.n();
		}

		ret.resize(ret.m(), ret.n());

		// 赋值 //
		Size beginRow = 0;
		for (const auto &mat : matrices)
		{
			for (Size i = 0; i < mat.m(); i++)
			{
				for (Size j = 0; j < mat.n(); j++)
				{
					ret(i + beginRow, j) = mat(i, j);
				}
			}
			beginRow += mat.m();
		}

		return ret;
	}
	template <typename MATRIX_LIST>
	Matrix combineRowMatrices(const MATRIX_LIST &matrices)
	{
		Matrix ret;

		// 获取全部矩阵的行数和列数 //
		for (const auto &mat : matrices)
		{
			if (mat.size() == 0)continue;
			if ((ret.m() != 0) && (ret.m() != mat.m()))THROW_FILE_LINE("input do not have valid size");

			ret.m_ = mat.m();
			ret.n_ += mat.n();
		}

		ret.resize(ret.m(), ret.n());

		// 赋值 //
		Size beginCol = 0;
		for (const auto &mat : matrices)
		{
			for (Size i = 0; i < mat.m(); i++)
			{
				for (Size j = 0; j < mat.n(); j++)
				{
					ret(i, j + beginCol) = mat(i, j);
				}
			}
			beginCol += mat.n();
		}

		return ret;
	}
	template <typename MATRIX_LISTLIST>
	Matrix combineMatrices(const MATRIX_LISTLIST &matrices)
	{
		std::list<Matrix> mat_col_list;
		for (const auto &mat_list : matrices)mat_col_list.push_back(combineRowMatrices(mat_list));
		return combineColMatrices(mat_col_list);
	}

	class Calculator
	{
	public:
		using BuiltInFunction = std::function<std::any(std::vector<std::any>&)>;
		using BinaryOperatorFunction = std::function<std::any(std::any&, std::any&)>;
		using UnaryOperatorFunction = std::function<std::any(std::any&)>;

		auto calculateExpression(std::string_view expression) const->std::pair<std::string, std::any>;
		
		auto addTypename(std::string_view tpn)->void;
		auto addOperator(std::string_view opr, int ul_priority, int ur_priority, int b_priority)->void;
		auto addVariable(std::string_view var, std::string_view type, const std::any &value)->void;
		auto addFunction(std::string_view fun, const std::vector<std::string> &param_type, std::string_view ret_type, BuiltInFunction f)->void;
		
		auto addUnaryLeftOperatorFunction(std::string_view opr, std::string_view p_type, std::string_view ret_type, UnaryOperatorFunction f)->void;
		auto addUnaryRightOperatorFunction(std::string_view opr, std::string_view p_type, std::string_view ret_type, UnaryOperatorFunction f)->void;
		auto addBinaryOperatorFunction(std::string_view opr, std::string_view p1_type, std::string_view p2_type, std::string_view ret_type, BinaryOperatorFunction f)->void;
		auto clearVariables()->void;

		//auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		//auto virtual saveXml(aris::core::XmlElement &xml_ele)const->void override;
		virtual ~Calculator();
		explicit Calculator(const std::string &name = "calculator");
		ARIS_DECLARE_BIG_FOUR(Calculator);
		//ARIS_REGISTER_TYPE(Calculator)
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class LanguageParser : public aris::core::Object
	{
	public:
		auto setProgram(std::string_view program)->void;
		auto parseLanguage()->void;
		auto varPool()->const std::vector<std::string>&;
		auto gotoMain()->void;
		auto gotoLine(int line)->void;
		auto forward(bool is_this_cmd_successful = true)->void;
		auto currentCmd()const->const std::string&;
		auto currentLine()const->int;
		auto isCurrentLineKeyWord()const->bool;
		auto isCurrentLineFunction()const->bool;
		auto isEnd()const->bool;

		virtual ~LanguageParser();
		explicit LanguageParser(const std::string &name = "language_parser");
		ARIS_REGISTER_TYPE(LanguageParser);
		ARIS_DECLARE_BIG_FOUR(LanguageParser);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}


#endif
