#ifndef ARIS_CORE_EXPRESSION_CALCULATOR_H_
#define ARIS_CORE_EXPRESSION_CALCULATOR_H_

#include <map>
#include <functional>
#include <string>
#include <iostream>
#include <list>
#include <any>

#include <aris_lib_export.h>
#include "aris/core/basic_type.hpp"
#include "aris/core/log.hpp"
#include "aris/core/object.hpp"

namespace aris::core
{
	class ARIS_API Matrix
	{
	public:
		auto swap(Matrix &other)->Matrix&;
		auto empty() const->bool;
		auto size() const->Size;
		auto data()->double *;
		auto data() const->const double *;
		auto begin() ->double *;
		auto begin() const ->const double *;
		auto end() ->double *;
		auto end()  const ->const double *;
		auto m() const->Size;
		auto n() const->Size;
		auto resize(Size m, Size n)->Matrix &;
		auto transpose()->Matrix &;

		auto copySubMatrixTo(const Matrix &subMat, Size beginRow, Size beginCol, Size rowNum, Size colNum)->void;
		auto copySubMatrixTo(const Matrix &subMat, Size beginRow, Size beginCol)->void;

		auto toString() const->std::string;
		auto toDouble() const->double;
		auto dsp() const ->void;

		auto operator()(Size i, Size j)->double &;
		auto operator()(Size i, Size j) const->const double &;

		friend auto ARIS_API operator + (const Matrix &m1, const Matrix &m2)->Matrix;
		friend auto ARIS_API operator - (const Matrix &m1, const Matrix &m2)->Matrix;
		friend auto ARIS_API operator * (const Matrix &m1, const Matrix &m2)->Matrix;
		friend auto ARIS_API operator / (const Matrix &m1, const Matrix &m2)->Matrix;
		friend auto ARIS_API operator - (const Matrix &m1)->Matrix;
		friend auto ARIS_API operator + (const Matrix &m1)->Matrix;

		template <typename MATRIX_LIST>
		friend auto combineColMatrices(const MATRIX_LIST &matrices)->Matrix;
		template <typename MATRIX_LIST>
		friend auto combineRowMatrices(const MATRIX_LIST &matrices)->Matrix;
		template <typename MATRIX_LISTLIST>
		friend auto combineMatrices(const MATRIX_LISTLIST &matrices)->Matrix;

		~Matrix();
		Matrix(double value);
		Matrix(Size m, Size n, double value = 0);
		Matrix(Size m, Size n, const double *data);
		Matrix(const std::initializer_list<Matrix> &data);
		Matrix();
		Matrix(const Matrix &other);
		Matrix(Matrix &&other);
		Matrix &operator=(Matrix other);

	private:
		struct Imp;
		ImpPtr<Imp> imp_;
	};

	template <typename MATRIX_LIST>
	Matrix combineColMatrices(const MATRIX_LIST &matrices)
	{
		// 获取全部矩阵的行数和列数 //
		Size m = 0, n = 0;
		for (const auto &mat : matrices)
		{
			if (mat.size() == 0)continue;
			if ((n != 0) && (n != mat.n()))THROW_FILE_LINE("input do not have valid size");
			m += mat.m();
			n = mat.n();
		}
		Matrix ret(m, n);

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
		// 获取全部矩阵的行数和列数 //
		Size m = 0, n = 0;
		for (const auto &mat : matrices)
		{
			if (mat.size() == 0)continue;
			if ((m != 0) && (m != mat.m()))THROW_FILE_LINE("input do not have valid size");
			m = mat.m();
			n += mat.n();
		}
		Matrix ret(m,n);

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

	class ARIS_API Calculator
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
		auto setNumberType(std::string_view tpn, std::function<std::any(double)> construct)->void;
		auto setStringType(std::string_view tpn, std::function<std::any(std::string)> construct)->void;

		auto addUnaryLeftOperatorFunction(std::string_view opr, std::string_view p_type, std::string_view ret_type, UnaryOperatorFunction f)->void;
		auto addUnaryRightOperatorFunction(std::string_view opr, std::string_view p_type, std::string_view ret_type, UnaryOperatorFunction f)->void;
		auto addBinaryOperatorFunction(std::string_view opr, std::string_view p1_type, std::string_view p2_type, std::string_view ret_type, BinaryOperatorFunction f)->void;
		auto clearVariables()->void;
		auto clearAllRules()->void;

		virtual ~Calculator();
		explicit Calculator(const std::string &name = "calculator");
		ARIS_DECLARE_BIG_FOUR(Calculator);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/*class ARIS_API LanguageParser
	{
	public:
		auto setProgram(std::string_view program)->void;
		auto parseLanguage()->void;
		auto varPool()->const std::vector<std::string>&;
		auto gotoMain()->void;
		auto gotoLine(int line)->void;
		auto forward(bool is_this_cmd_successful = true)->void;
		// 返回整句话，同时 trim 两侧 //
		auto currentCmd()const->const std::string&;
		// 返回当前的行号 //
		auto currentLine()const->int;
		// 返回当前行的 word //
		auto currentWord()const->std::string_view;
		// 返回当前行出去 word 后的部分 //
		auto currentParamStr()const->std::string_view;
		auto isCurrentLineKeyWord()const->bool;
		auto isCurrentLineFunction()const->bool;
		auto isEnd()const->bool;

		virtual ~LanguageParser();
		explicit LanguageParser(const std::string &name = "language_parser");
		ARIS_DECLARE_BIG_FOUR(LanguageParser);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};*/
	class ARIS_API LanguageParser
	{
	public:
		auto setProgram(std::map<std::string, std::string> program)->void;
		auto parseLanguage()->void;
		auto varPool()->const std::vector<std::string>&;
		auto gotoMain()->void;
		auto gotoFileLine(std::string file, int line)->void;
		auto forward(bool is_this_cmd_successful = true)->void;
		// 返回整句话，同时 trim 两侧 //
		auto currentCmd()const->const std::string&;
		// 返回当前的行号 //
		auto currentLine()const->int;
		// 返回当前的文件 //
		auto currentFile()const->std::string;
		// 返回当前行的 word //
		auto currentWord()const->std::string_view;
		// 返回当前行出去 word 后的部分 //
		auto currentParamStr()const->std::string_view;
		auto isCurrentLineKeyWord()const->bool;
		auto isCurrentLineFunction()const->bool;
		auto isEnd()const->bool;

		virtual ~LanguageParser();
		explicit LanguageParser(const std::string &name = "language_parser");
		ARIS_DECLARE_BIG_FOUR(LanguageParser);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}


#endif
