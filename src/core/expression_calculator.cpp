#include <sstream>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <cstring>
#include <algorithm>
#include <regex>
#include <list>
#include <cmath>
#include <any>

#include "aris/core/expression_calculator.hpp"

namespace aris::core
{
	auto s_mm(Size m, Size n, Size k, const double* A, Size lda, const double* B, Size ldb, double *C, Size ldc) noexcept->void
	{
		for (Size i = 0; i < m; ++i)
		{
			Size row_idx = i * lda;
			for (Size j = 0; j < n; ++j)
			{
				Size idx = i * ldc + j;

				C[idx] = 0;
				for (Size u = 0; u < k; ++u)
				{
					C[idx] += A[row_idx + u] * B[j + u * ldb];
				}
			}
		}
	}
	auto s_mmTN(Size m, Size n, Size k, const double* A, Size lda, const double* B, Size ldb, double *C, Size ldc) noexcept->void
	{
		for (Size i = 0; i < m; ++i)
		{
			for (Size j = 0; j < n; ++j)
			{
				Size idx = i * ldc + j;

				C[idx] = 0;
				for (Size u = 0; u < k; ++u)
				{
					C[idx] += A[i + u * lda] * B[j + u * ldb];
				}
			}
		}
	}
	auto s_mmNT(Size m, Size n, Size k, const double* A, Size lda, const double* B, Size ldb, double *C, Size ldc) noexcept->void
	{
		for (Size i = 0; i < m; ++i)
		{
			Size row_idx = i * lda;
			for (Size j = 0; j < n; ++j)
			{
				Size col_idx = j * ldb;

				Size idx = i * ldc + j;

				C[idx] = 0;
				for (Size u = 0; u < k; ++u)
				{
					C[idx] += A[row_idx + u] * B[col_idx + u];
				}
			}
		}
	}
	auto s_mmTT(Size m, Size n, Size k, const double* A, Size lda, const double* B, Size ldb, double *C, Size ldc) noexcept->void
	{
		for (Size i = 0; i < m; ++i)
		{
			for (Size j = 0; j < n; ++j)
			{
				Size col_idx = j * ldb;
				Size idx = i * ldc + j;

				C[idx] = 0;
				for (Size u = 0; u < k; ++u)
				{
					C[idx] += A[i + u * lda] * B[col_idx + u];
				}
			}
		}
	}

	Matrix::Matrix(Size m, Size n, double value) : m_(m), n_(n), is_row_major_(true), data_vec_(m*n, value) { }
	Matrix::Matrix(Size m, Size n, const double *data) : Matrix(m, n) { std::copy(data, data + m * n, this->data()); }
	Matrix::Matrix(double value) : m_(1), n_(1), is_row_major_(true), data_vec_(1, value) {}
	Matrix::Matrix(const std::initializer_list<Matrix> &data) : Matrix()
	{
		std::list<std::list<Matrix> > mat_list_list;

		bool need_new_row = true;
		for (auto &i : data)
		{
			if (i.empty())
			{
				need_new_row = true;
			}
			else
			{
				if (need_new_row)
				{
					mat_list_list.push_back(std::list<Matrix>());
					need_new_row = false;
				}

				mat_list_list.back().push_back(i);
			}
		}


		(*this) = aris::core::combineMatrices(mat_list_list);
	}
	auto Matrix::swap(Matrix &other)->Matrix&
	{
		std::swap(this->m_, other.m_);
		std::swap(this->n_, other.n_);
		std::swap(this->is_row_major_, other.is_row_major_);
		std::swap(this->data_vec_, other.data_vec_);

		return *this;
	}
	auto Matrix::resize(Size m, Size n)->Matrix &
	{
		m_ = m;
		n_ = n;
		data_vec_.resize(m*n);
		return *this;
	}
	auto Matrix::transpose()->Matrix &
	{
		is_row_major_ = !is_row_major_;
		std::swap(m_, n_);
		return *this;
	}
	auto Matrix::copySubMatrixTo(const Matrix &subMat, Size beginRow, Size beginCol, Size rowNum, Size colNum)->void
	{
		if ((beginRow + subMat.m() > m()) || (beginCol + subMat.n() > n()))
		{
			THROW_FILE_LINE("Function CopySubMatrixTo must have subMat smaller than self matrix");
		}

		for (Size i = 0; i < subMat.m(); ++i)
		{
			for (Size j = 0; j < subMat.n(); ++j)
			{
				this->operator()(i + beginRow, j + beginCol) = subMat(i, j);
			}
		}

	}
	auto Matrix::toString() const->std::string
	{
		std::stringstream stream;

		stream.precision(15);
		stream << "{";
		for (Size i = 0; i < m(); ++i)
		{
			for (Size j = 0; j < n(); ++j)
			{
				stream << this->operator()(i, j);
				if (j<n() - 1)stream << ",";
			}
			if (i<m() - 1)
				stream << ";\n";
		}
		stream << "}";

		return stream.str();
	}

	Matrix operator + (const Matrix &m1, const Matrix &m2)
	{
		Matrix ret;

		if (m1.size() == 1)
		{
			ret = Matrix(m2);

			for (auto &d : ret)
			{
				d += *m1.data();
			}
		}
		else if (m2.size() == 1)
		{
			ret = Matrix(m1);

			for (auto &d : ret)
			{
				d += *m2.data();
			}
		}
		else if ((m1.m() == m2.m()) && (m1.n() == m2.n()))
		{
			ret.resize(m1.m(), m1.n());

			for (Size i = 0; i < ret.m(); ++i)
			{
				for (Size j = 0; j < ret.n(); ++j)
				{
					ret(i, j) = m1(i, j) + m2(i, j);
				}
			}
		}
		else
		{
			THROW_FILE_LINE("Can't plus matrices, the dimensions are not equal");
		}
		return ret;
	}
	Matrix operator - (const Matrix &m1, const Matrix &m2)
	{
		Matrix ret;

		if ((m1.m() == 1) && (m1.n() == 1))
		{
			ret = Matrix(m2);

			for (Size i = 0; i < ret.size(); ++i)
			{
				ret.data()[i] = m1(0, 0) - ret.data()[i];
			}
		}
		else if ((m2.m() == 1) && (m2.n() == 1))
		{
			ret = Matrix(m1);

			for (Size i = 0; i < ret.size(); ++i)
			{
				ret.data()[i] -= m2(0, 0);
			}
		}
		else if ((m1.m() == m2.m()) && (m1.n() == m2.n()))
		{
			ret.resize(m1.m(), m1.n());

			for (Size i = 0; i < ret.m(); i++)
			{
				for (Size j = 0; j < ret.n(); j++)
				{
					ret(i, j) = m1(i, j) - m2(i, j);
				}
			}
		}
		else
		{
			THROW_FILE_LINE("Can't minus matrices, the dimensions are not equal");
		}

		return ret;
	}
	Matrix operator * (const Matrix &m1, const Matrix &m2)
	{
		Matrix ret;

		if ((m1.m() == 1) && (m1.n() == 1))
		{
			ret = Matrix(m2);

			for (Size i = 0; i < ret.size(); ++i)
			{
				ret.data()[i] *= m1(0, 0);
			}
		}
		else if ((m2.m() == 1) && (m2.n() == 1))
		{
			ret = Matrix(m1);

			for (Size i = 0; i < ret.size(); ++i)
			{
				ret.data()[i] *= m2(0, 0);
			}
		}
		else if (m1.n() == m2.m())
		{
			ret.resize(m1.m(), m2.n());

			if (m1.is_row_major_)
			{
				if (m2.is_row_major_)
				{
					aris::core::s_mm(m1.m(), m2.n(), m1.n(), m1.data(), 1, m2.data(), 1, ret.data(), 1);
				}
				else
				{
					aris::core::s_mmNT(m1.m(), m2.n(), m1.n(), m1.data(), 1, m2.data(), 1, ret.data(), 1);
				}
			}
			else
			{
				if (m2.is_row_major_)
				{
					aris::core::s_mmTN(m1.m(), m2.n(), m1.n(), m1.data(), 1, m2.data(), 1, ret.data(), 1);
				}
				else
				{
					aris::core::s_mmTT(m1.m(), m2.n(), m1.n(), m1.data(), 1, m2.data(), 1, ret.data(), 1);
				}
			}


		}
		else
		{
			THROW_FILE_LINE("Can't multiply matrices, the dimensions are not equal");
		}

		return ret;
	}
	Matrix operator / (const Matrix &m1, const Matrix &m2)
	{
		Matrix ret;

		if ((m1.m() == 1) && (m1.n() == 1))
		{
			ret = Matrix(m2);

			for (Size i = 0; i < ret.size(); ++i)
			{
				ret.data()[i] = m1(0, 0) / ret.data()[i];
			}
		}
		else if ((m2.m() == 1) && (m2.n() == 1))
		{
			ret = Matrix(m1);

			for (Size i = 0; i < ret.size(); ++i)
			{
				ret.data()[i] /= m2(0, 0);
			}
		}
		else
		{
			THROW_FILE_LINE("Right now, divide operator of matrices is not added");
		}

		return ret;
	}
	Matrix operator - (const Matrix &m1)
	{
		Matrix m(m1.m(), m1.n());
		for (Size i = 0; i < m1.size(); ++i)m.data()[i] = -m1.data()[i];

		return m;
	}
	Matrix operator + (const Matrix &m1)
	{
		return m1;
	}

	auto getWord(std::string_view &input)->std::string_view
	{
		static const std::string seperateStr("()[]{},;");
		static const std::string operatorStr("*+-/\\^|<>=");
		static const std::string spaceStr(" \t\n\r\f\v");
		static const std::string numStr("0123456789.");
		static const std::string varStr("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_0123456789");

		// triml and check if empty //
		auto next_pos = input.find_first_not_of(spaceStr);
		input = next_pos == std::string::npos ? std::string_view() : std::string_view(input.data() + next_pos, input.size() - next_pos);
		if (next_pos == std::string_view::npos)return std::string_view();

		// get number //
		if (input[0] <= '9' && input[0] >= '0')
		{
			bool has_scientific = false;
			int i;
			for (i = 1; i < input.size(); ++i)
			{
				// 可能是科学计数法 //
				if (i < input.size() - 1
					&& (input[i] == 'e' || input[i] == 'E')
					&& ((input[i + 1] <= '9' && input[i + 1] >= '0') || input[i + 1] == '+' || input[i + 1] == '-'))
				{
					++i;
					continue;
				}

				if (numStr.find(input[i]) == std::string_view::npos) break;
			}

			auto ret = input.substr(0, i);
			input = i == input.size() ? std::string_view() : input.substr(i);
			return ret;
		}
		// get seperator //
		else if (seperateStr.find(input[0]) != seperateStr.npos)
		{
			auto ret = input.substr(0, 1);
			input = 1 == input.size() ? std::string_view() : input.substr(1);
			return ret;
		}
		// get operator //
		else if (operatorStr.find(input[0]) != operatorStr.npos)
		{
			int i;
			for (i = 1; i < input.size() && operatorStr.find(input[i]) != std::string_view::npos; ++i);

			auto ret = input.substr(0, i);
			input = i == input.size() ? std::string_view() : input.substr(i);
			return ret;
		}
		// get string //
		else
		{
			int i;
			for (i = 1; i < input.size() && varStr.find(input[i]) != std::string_view::npos; ++i);

			auto ret = input.substr(0, i);
			input = i == input.size() ? std::string_view() : input.substr(i);
			return ret;
		}
	}
	
	struct Calculator::Imp
	{
		class Operator;
		class Function;

		class Token
		{
		public:
			enum Type
			{
				NO,     //not determined
				COMMA,    //comma,which is ','
				SEMICOLON,    //SEMICOLONerator,which is ';'
				PARENTHESIS_L,  //pranthese begin(left parenthesis)
				PARENTHESIS_R,  //pranthese end(right parenthesis)
				BRACKET_L,  //bracket begin(left)
				BRACKET_R,  //bracket end(right)
				BRACE_L,
				BRACE_R,
				OPERATOR,    //operator
				NUMBER,    //const matrix
				VARIABLE,    //variable
				Function     //function
			};

			Type type;
			std::string_view word;

			double num;
			const Matrix *var;
			const Imp::Function *fun;
			const Imp::Operator *opr;
		};
		class Operator
		{
		public:
			std::string name;

			Size priority_ul;//unary left
			Size priority_ur;//unary right
			Size priority_b;//binary

			typedef std::function<Matrix(Matrix)> U_FUN;
			typedef std::function<Matrix(Matrix, Matrix)> B_FUN;

			U_FUN fun_ul;
			U_FUN fun_ur;
			B_FUN fun_b;

			Operator() :priority_ul(0), priority_ur(0), priority_b(0) {}

			void SetUnaryLeftOpr(Size priority, U_FUN fun) { priority_ul = priority; this->fun_ul = fun; }
			void SetUnaryRightOpr(Size priority, U_FUN fun) { priority_ur = priority; this->fun_ur = fun; }
			void SetBinaryOpr(Size priority, B_FUN fun) { priority_b = priority; this->fun_b = fun; }
		};
		class Function
		{
			typedef std::function<Matrix(std::vector<Matrix>)> FUN;
		public:
			std::string name;
			std::map<Size, FUN> funs;

			void AddOverloadFun(Size n, FUN fun) { funs.insert(make_pair(n, fun)); }
		};

		typedef std::vector<Token> TokenVec;
		TokenVec Expression2Tokens(std::string_view expression)const;
		Matrix CaculateTokens(TokenVec::iterator beginToken, TokenVec::iterator max_end_token) const;

		Matrix CaculateValueInParentheses(TokenVec::iterator &i, TokenVec::iterator max_end_token) const;
		Matrix CaculateValueInBraces(TokenVec::iterator &i, TokenVec::iterator max_end_token) const;
		Matrix CaculateValueInFunction(TokenVec::iterator &i, TokenVec::iterator max_end_token) const;
		Matrix CaculateValueInOperator(TokenVec::iterator &i, TokenVec::iterator max_end_token) const;

		TokenVec::iterator FindNextOutsideToken(TokenVec::iterator leftPar, TokenVec::iterator endToken, Token::Type type) const;
		TokenVec::iterator FindNextEqualLessPrecedenceBinaryOpr(TokenVec::iterator beginToken, TokenVec::iterator endToken, Size precedence)const;
		std::vector<std::vector<Matrix> > GetMatrices(TokenVec::iterator beginToken, TokenVec::iterator endToken)const;

		std::map<std::string, Operator, std::less<>> operator_map_;
		std::map<std::string, Function, std::less<>> function_map_;
		std::map<std::string, Matrix, std::less<>> variable_map_;
		std::map<std::string, std::string, std::less<>> string_map_;//string variable
	};
	auto Calculator::Imp::Expression2Tokens(std::string_view expression)const -> TokenVec
	{
		TokenVec tokens;
		Token token;

		for (token.word = getWord(expression); !token.word.empty(); token.word = getWord(expression))
		{
			token.type = Token::NO;

			switch (*token.word.data())
			{
			case ',':token.type = Token::COMMA; break;
			case ';':token.type = Token::SEMICOLON; break;
			case '(':token.type = Token::PARENTHESIS_L; break;
			case ')':token.type = Token::PARENTHESIS_R; break;
			case '[':token.type = Token::BRACKET_L; break;
			case ']':token.type = Token::BRACKET_R; break;
			case '{':token.type = Token::BRACE_L; break;
			case '}':token.type = Token::BRACE_R; break;
			default:
				// 数字
				if (std::stringstream(std::string(token.word)) >> token.num)
				{
					token.type = Token::NUMBER;
					break;
				}
				// 操作符
				if (operator_map_.find(token.word) != operator_map_.end())
				{
					token.type = Token::OPERATOR;
					token.opr = &operator_map_.find(token.word)->second;
					break;
				}
				// 变量
				if (variable_map_.find(token.word) != variable_map_.end())
				{
					token.type = Token::VARIABLE;
					token.var = &variable_map_.find(token.word)->second;
					break;
				}
				// 函数
				if (function_map_.find(token.word) != function_map_.end())
				{
					token.type = Token::Function;
					token.fun = &function_map_.find(token.word)->second;
					break;
				}

			}
			if (token.type == Token::NO) THROW_FILE_LINE("unrecognized symbol \"" + std::string(token.word) + "\"");
			tokens.push_back(token);
			continue;
		}

		return tokens;
	}
	auto Calculator::Imp::CaculateTokens(TokenVec::iterator beginToken, TokenVec::iterator endToken) const ->Matrix
	{
		if (beginToken >= endToken)THROW_FILE_LINE("invalid expression");

		auto i = beginToken;
		Matrix value;
		bool isBegin = true;
		while (i < endToken)
		{
			// 如果没有当前值,证明刚刚开始计算 //
			if (isBegin)
			{
				isBegin = false;
				switch (i->type)
				{
				case Token::PARENTHESIS_L:
					value = CaculateValueInParentheses(i, endToken);
					break;
				case Token::BRACE_L:
					value = CaculateValueInBraces(i, endToken);
					break;
				case Token::NUMBER:
					value = i->num;
					i++;
					break;
				case Token::OPERATOR:
					value = CaculateValueInOperator(i, endToken);
					break;
				case Token::VARIABLE:
					value = *i->var;
					i++;
					break;
				case Token::Function:
					value = CaculateValueInFunction(i, endToken);
					break;
				default:
					THROW_FILE_LINE("expression not valid");
				}
			}
			else//如果有当前值,但没有操作符
			{
				if (i->type == Token::OPERATOR)
				{
					if (i->opr->priority_ur)
					{
						value = i->opr->fun_ur(value);
						i++;
					}
					else if (i->opr->priority_b > 0)
					{
						auto e = FindNextEqualLessPrecedenceBinaryOpr(i + 1, endToken, i->opr->priority_b);
						value = i->opr->fun_b(value, CaculateTokens(i + 1, e));
						i = e;
					}
					else
					{
						THROW_FILE_LINE("expression not valid");
					}
				}
				else
				{
					THROW_FILE_LINE("expression not valid: lack operator");
				}
			}
		}

		return value;
	}
	auto Calculator::Imp::CaculateValueInParentheses(TokenVec::iterator &i, TokenVec::iterator max_end_token)const->Matrix
	{
		auto beginPar = i + 1;
		auto endPar = FindNextOutsideToken(i + 1, max_end_token, Token::PARENTHESIS_R);
		i = endPar + 1;

		return CaculateTokens(beginPar, endPar);
	}
	auto Calculator::Imp::CaculateValueInBraces(TokenVec::iterator &i, TokenVec::iterator max_end_token)const->Matrix
	{
		auto beginBce = i + 1;
		auto endBce = FindNextOutsideToken(i + 1, max_end_token, Token::BRACE_R);
		i = endBce + 1;

		return combineMatrices(GetMatrices(beginBce, endBce));
	}
	auto Calculator::Imp::CaculateValueInFunction(TokenVec::iterator &i, TokenVec::iterator max_end_token)const->Matrix
	{
		auto beginPar = i + 1;
		if (i + 1 >= max_end_token) THROW_FILE_LINE("invalid expression");
		if (beginPar->type != Token::PARENTHESIS_L)THROW_FILE_LINE("function must be followed by \"(\"");

		auto endPar = FindNextOutsideToken(beginPar + 1, max_end_token, Token::PARENTHESIS_R);
		auto matrices = GetMatrices(beginPar + 1, endPar);

		if (matrices.size() != 1)THROW_FILE_LINE("function \"" + std::string(i->word) + "\" + do not has invalid param type");

		auto params = matrices.front();
		auto f = i->fun->funs.find(params.size());
		if (f == i->fun->funs.end())THROW_FILE_LINE("function \"" + std::string(i->word) + "\" + do not has invalid param num");

		i = endPar + 1;
		return f->second(params);
	}
	auto Calculator::Imp::CaculateValueInOperator(TokenVec::iterator &i, TokenVec::iterator max_end_token)const->Matrix
	{
		auto opr = i;
		i = FindNextEqualLessPrecedenceBinaryOpr(opr + 1, max_end_token, opr->opr->priority_ul);
		return opr->opr->fun_ul(CaculateTokens(opr + 1, i));
	}
	auto Calculator::Imp::FindNextOutsideToken(TokenVec::iterator beginToken, TokenVec::iterator endToken, Token::Type type)const->TokenVec::iterator
	{
		Size parNum = 0, braNum = 0, bceNum = 0;
		auto nextPlace = beginToken;

		while (nextPlace < endToken)
		{
			if ((parNum == 0) && (braNum == 0) && (bceNum == 0) && (nextPlace->type == type))return nextPlace;
			switch (nextPlace->type)
			{
			case Token::PARENTHESIS_L:parNum++; break;
			case Token::PARENTHESIS_R:parNum--; break;
			case Token::BRACKET_L:braNum++; break;
			case Token::BRACKET_R:braNum--; break;
			case Token::BRACE_L:bceNum++; break;
			case Token::BRACE_R:bceNum--; break;
			default:break;
			}

			++nextPlace;
		}

		return nextPlace;
	}
	auto Calculator::Imp::FindNextEqualLessPrecedenceBinaryOpr(TokenVec::iterator beginToken, TokenVec::iterator endToken, Size precedence)const->TokenVec::iterator
	{
		auto nextOpr = beginToken;

		while (nextOpr < endToken)
		{
			nextOpr = FindNextOutsideToken(nextOpr, endToken, Token::OPERATOR);

			if ((nextOpr == endToken) || (nextOpr->opr->priority_b <= precedence))
			{
				break;
			}
			else
			{
				++nextOpr;
			}
		}

		return nextOpr;
	}
	auto Calculator::Imp::GetMatrices(TokenVec::iterator beginToken, TokenVec::iterator endToken)const->std::vector<std::vector<Matrix> >
	{
		std::vector<std::vector<Matrix> > ret;

		auto rowBegin = beginToken;
		while (rowBegin < endToken)
		{
			auto rowEnd = FindNextOutsideToken(rowBegin, endToken, Token::SEMICOLON);
			auto colBegin = rowBegin;

			ret.push_back(std::vector<Matrix>());

			while (colBegin < rowEnd)
			{
				auto colEnd = FindNextOutsideToken(colBegin, rowEnd, Token::COMMA);

				ret.back().push_back(CaculateTokens(colBegin, colEnd));

				if (colEnd == endToken)
					colBegin = colEnd;
				else
					colBegin = colEnd + 1;
			}


			if (rowEnd == endToken)
				rowBegin = rowEnd;
			else
				rowBegin = rowEnd + 1;
		}


		return ret;
	}
	auto Calculator::calculateExpression(std::string_view expression) const->Matrix
	{
		auto tokens = imp_->Expression2Tokens(std::string(expression));
		return imp_->CaculateTokens(tokens.begin(), tokens.end());
	}
	auto Calculator::evaluateExpression(const std::string &expression)const->std::string
	{
		auto ret = expression;

		for (auto &var : imp_->string_map_)
		{
			std::string exp{ "\\$\\{" + var.first + "\\}" };

			std::regex var_rex{ exp };
			ret = std::regex_replace(ret, var_rex, var.second);
		}


		return ret;
	}
	auto Calculator::addVariable(const std::string &name, const Matrix &value)->void
	{
		if (imp_->function_map_.find(name) != imp_->function_map_.end())
		{
			THROW_FILE_LINE("function \"" + name + "already exists, can't add variable");
		}

		if (imp_->variable_map_.find(name) != imp_->variable_map_.end())
		{
			THROW_FILE_LINE("variable \"" + name + "already exists, can't add variable");
		}
		imp_->variable_map_.insert(make_pair(name, value));
	}
	auto Calculator::addVariable(const std::string &name, const std::string &value)->void
	{
		if (imp_->string_map_.find(name) != imp_->string_map_.end())
		{
			THROW_FILE_LINE("variable \"" + name + "already exists, can't add string variable");
		}
		imp_->string_map_.insert(make_pair(name, value));
	}
	auto Calculator::addFunction(const std::string &name, std::function<Matrix(std::vector<Matrix>)> f, Size n)->void
	{
		if (imp_->variable_map_.find(name) != imp_->variable_map_.end())
		{
			THROW_FILE_LINE("variable \"" + name + "already exists, can't add function");
		}

		imp_->function_map_[name].AddOverloadFun(n, f);
	}
	auto Calculator::clearVariables()->void { imp_->variable_map_.clear(); imp_->string_map_.clear(); }
	Calculator::~Calculator() = default;
	Calculator::Calculator(const std::string &name) 
	{
		imp_->operator_map_["+"].SetBinaryOpr(1, [](Matrix m1, Matrix m2) {return m1 + m2; });
		imp_->operator_map_["+"].SetUnaryLeftOpr(1, [](Matrix m) {return m; });
		imp_->operator_map_["-"].SetBinaryOpr(1, [](Matrix m1, Matrix m2) {return m1 - m2; });
		imp_->operator_map_["-"].SetUnaryLeftOpr(1, [](Matrix m) {return -m; });
		imp_->operator_map_["*"].SetBinaryOpr(2, [](Matrix m1, Matrix m2) {return m1 * m2; });
		imp_->operator_map_["/"].SetBinaryOpr(2, [](Matrix m1, Matrix m2) {return m1 / m2; });

#define MATRIX_OPR(OPR) \
		Matrix ret;\
		if (m1.size() == 1)\
		{					\
			ret = Matrix(m2);\
			for (auto &d : ret)d = m1(0, 0) OPR d;\
		}\
		else if (m2.size() == 1)\
		{\
			ret = Matrix(m1);\
			for (auto &d : ret) d = d OPR m2(0, 0);\
		}\
		else if ((m1.m() == m2.m()) && (m1.n() == m2.n()))\
		{\
			ret = Matrix(m1);\
								\
			for (Size i = 0; i < ret.size(); ++i)\
			{\
				ret.data()[i] = m1.data()[i] OPR m2.data()[i];\
			}\
		}\
		else\
		{\
			THROW_FILE_LINE("dimensions are not equal");\
		}\
		return ret;


		imp_->operator_map_["<"].SetBinaryOpr(1, [](Matrix m1, Matrix m2) { MATRIX_OPR(<); });
		imp_->operator_map_["<="].SetBinaryOpr(1, [](Matrix m1, Matrix m2) { MATRIX_OPR(<= ); });
		imp_->operator_map_[">"].SetBinaryOpr(1, [](Matrix m1, Matrix m2) { MATRIX_OPR(>); });
		imp_->operator_map_[">="].SetBinaryOpr(1, [](Matrix m1, Matrix m2) { MATRIX_OPR(>= ); });
		imp_->operator_map_["=="].SetBinaryOpr(1, [](Matrix m1, Matrix m2) { MATRIX_OPR(== ); });

#undef ARIS_SET_TYPE

		addFunction("sqrt", [](std::vector<Matrix> v)
		{
			Matrix ret = v.front();

			for (auto &d : ret)	d = std::sqrt(d);

			return ret;
		}, 1);
	}
	Calculator::Calculator(const Calculator &) = default;
	Calculator::Calculator(Calculator &&) = default;
	Calculator& Calculator::operator=(const Calculator &) = default;
	Calculator& Calculator::operator=(Calculator &&) = default;

	struct Compiler::Imp
	{
		struct Type {};
		struct Value
		{
			std::string type_;
			std::any value_;

			auto val()->std::any& 
			{ 
				if (std::any_cast<void*>(&value_))
				{
					auto p = reinterpret_cast<std::any*>(std::any_cast<void*>(value_));
					return *p;
				}
				else
				{
					return value_;
				}
				
				
				return std::any_cast<std::any>(&value_) ? value_ : **std::any_cast<std::any*>(&value_);
			}
		};
		struct Variable
		{
			std::string type_, name_;
			std::any value_;
		};
		struct Operator
		{
			std::string name;

			Size priority_ul{ 0 };    //unary left
			Size priority_ur{ 0 };    //unary right
			Size priority_b{ 0 };     //binary

			std::map<std::string, std::map<std::string, std::pair<std::string, BinaryOperatorFunction>, std::less<>>, std::less<>> b_funs_;
			std::map<std::string, std::pair<std::string, UnaryOperatorFunction>, std::less<>> ul_funs_;
			std::map<std::string, std::pair<std::string, UnaryOperatorFunction>, std::less<>> ur_funs_;
		};
		struct Function
		{
			std::string name;
			std::vector<std::tuple<std::vector<std::string>, std::string, Compiler::BuiltInFunction>> funs_;
		};
		class Token
		{
		public:
			enum Type
			{
				NO,               // invalid
				COMMA,            // ,
				SEMICOLON,        // ;
				PARENTHESIS_L,    // (
				PARENTHESIS_R,    // )
				BRACKET_L,        // [
				BRACKET_R,        // ]
				BRACE_L,          // {
				BRACE_R,          // }
				OPERATOR,         // operator
				NUMBER,           // number
				VARIABLE,         // variable
				FUNCTION          // function
			};

			Type type;
			std::string_view word;

			double num;
			const Variable *var;
			const Function *fun;
			const Operator *opr;
		};

		typedef std::vector<Token> TokenVec;
		auto Expression2Tokens(std::string_view expression)const->TokenVec;
		auto CaculateTokens(TokenVec::iterator begin_token, TokenVec::iterator max_end_token) const->Value;

		auto CaculateValueInParentheses(TokenVec::iterator &begin_token, TokenVec::iterator max_end_token) const->Value;
		auto CaculateValueInBraces(TokenVec::iterator &begin_token, TokenVec::iterator max_end_token) const->Value;
		auto CaculateValueInFunction(TokenVec::iterator &begin_token, TokenVec::iterator max_end_token) const->Value;
		auto CaculateValueInOperator(TokenVec::iterator &begin_token, TokenVec::iterator max_end_token) const->Value;

		TokenVec::iterator FindNextOutsideToken(TokenVec::iterator left_par, TokenVec::iterator end_token, Token::Type type) const;
		TokenVec::iterator FindNextEqualLessPrecedenceBinaryOpr(TokenVec::iterator begin_token, TokenVec::iterator end_token, Size precedence)const;
		auto GetValues(TokenVec::iterator begin_token, TokenVec::iterator end_token)const->std::vector<std::vector<Value> >;

		std::map<std::string, Type, std::less<>> type_map_;
		std::map<std::string, Operator, std::less<>> operator_map_;
		std::map<std::string, Function, std::less<>> function_map_;
		std::map<std::string, Variable, std::less<>> variable_map_;
	};
	auto Compiler::Imp::Expression2Tokens(std::string_view expression)const -> TokenVec
	{
		TokenVec tokens;
		Token token;

		for (token.word = getWord(expression); !token.word.empty(); token.word = getWord(expression))
		{
			switch (*token.word.data())
			{
			case ',':token.type = Token::COMMA; break;
			case ';':token.type = Token::SEMICOLON; break;
			case '(':token.type = Token::PARENTHESIS_L; break;
			case ')':token.type = Token::PARENTHESIS_R; break;
			case '[':token.type = Token::BRACKET_L; break;
			case ']':token.type = Token::BRACKET_R; break;
			case '{':token.type = Token::BRACE_L; break;
			case '}':token.type = Token::BRACE_R; break;
			default:
				// 数字
				if (std::stringstream(std::string(token.word)) >> token.num)
				{
					token.type = Token::NUMBER;
					break;
				}
				// 操作符
				if (operator_map_.find(token.word) != operator_map_.end())
				{
					token.type = Token::OPERATOR;
					token.opr = &operator_map_.find(token.word)->second;
					break;
				}
				// 变量
				if (variable_map_.find(token.word) != variable_map_.end())
				{
					token.type = Token::VARIABLE;
					token.var = &variable_map_.find(token.word)->second;
					break;
				}
				// 函数
				if (function_map_.find(token.word) != function_map_.end())
				{
					token.type = Token::FUNCTION;
					token.fun = &function_map_.find(token.word)->second;
					break;
				}

				THROW_FILE_LINE("unrecognized symbol \"" + std::string(token.word) + "\"");
			}
			tokens.push_back(token);
		}

		return tokens;
	}
	auto Compiler::Imp::CaculateTokens(TokenVec::iterator b_tok, TokenVec::iterator e_tok) const ->Value
	{
		if (b_tok >= e_tok)THROW_FILE_LINE("invalid expression");

		Value value;
		for (auto i = b_tok; i < e_tok; )
		{
			// 是否刚开始 //
			if (i == b_tok)
			{
				switch (i->type)
				{
				case Token::PARENTHESIS_L:
					value = CaculateValueInParentheses(i, e_tok);
					break;
				case Token::BRACE_L:
					value = CaculateValueInBraces(i, e_tok);
					break;
				case Token::NUMBER:
					value = Value{ "Number", i->num };
					i++;
					break;
				case Token::OPERATOR:
					value = CaculateValueInOperator(i, e_tok);
					break;
				case Token::VARIABLE:
					value = Value{ i->var->type_, (void*)&i->var->value_ };
					i++;
					break;
				case Token::FUNCTION:
					value = CaculateValueInFunction(i, e_tok);
					break;
				default:
					THROW_FILE_LINE("expression not valid");
				}
			}
			else//如果有当前值,但没有操作符
			{
				if (i->type == Token::OPERATOR)
				{
					if (i->opr->priority_ur)
					{
						auto func = i->opr->ur_funs_.at(value.type_);
						value = Value{ func.first, func.second(value.val()) };
						i++;
					}
					else if (i->opr->priority_b > 0)
					{
						auto e = FindNextEqualLessPrecedenceBinaryOpr(i + 1, e_tok, i->opr->priority_b);
						auto right_value = CaculateTokens(i + 1, e);
						auto func = i->opr->b_funs_.at(value.type_).at(right_value.type_);
						value = Value{ func.first, func.second(value.val(), right_value.val()) };
						i = e;
					}
					else
					{
						THROW_FILE_LINE("expression not valid");
					}
				}
				else
				{
					THROW_FILE_LINE("expression not valid: lack operator");
				}
			}
		}

		return value;
	}
	auto Compiler::Imp::CaculateValueInParentheses(TokenVec::iterator &i, TokenVec::iterator max_end_token)const->Value
	{
		auto b_par = i + 1;
		auto e_par = FindNextOutsideToken(i + 1, max_end_token, Token::PARENTHESIS_R);
		i = e_par + 1;

		return CaculateTokens(b_par, e_par);
	}
	auto Compiler::Imp::CaculateValueInBraces(TokenVec::iterator &i, TokenVec::iterator max_end_token)const->Value
	{
		auto b_bra = i + 1;
		auto e_bra = FindNextOutsideToken(i + 1, max_end_token, Token::BRACE_R);
		i = e_bra + 1;

		auto values = GetValues(b_bra, e_bra);
		std::vector<std::vector<Matrix>> mtx;
		mtx.reserve(values.size());

		for (auto &r_vec : values)
		{
			mtx.push_back(std::vector<Matrix>(r_vec.size()));
			for (auto &value : r_vec)
			{
				if (value.type_ == "Number")
				{
					mtx.back().push_back(Matrix(std::any_cast<double>(value.value_)));
				}
				else if (value.type_ == "Matrix")
				{
					mtx.back().push_back(std::any_cast<Matrix&>(value.value_));
				}
				else
				{
					THROW_FILE_LINE("invalid type when build Matrix");
				}
			}
		}
		auto m = combineMatrices(mtx);
		return Value{ std::string("Matrix"), m };
	}
	auto Compiler::Imp::CaculateValueInFunction(TokenVec::iterator &i, TokenVec::iterator max_end_token)const->Value
	{
		auto b_par = i + 1;
		if (i + 1 >= max_end_token) THROW_FILE_LINE("invalid expression");
		if (b_par->type != Token::PARENTHESIS_L)THROW_FILE_LINE("function must be followed by \"(\"");

		auto e_par = FindNextOutsideToken(b_par + 1, max_end_token, Token::PARENTHESIS_R);

		// get values, but values dimensions must be 1 x n //
		auto value_mat = GetValues(b_par + 1, e_par);
		if (value_mat.size() != 1)THROW_FILE_LINE("function \"" + std::string(i->word) + "\" + do not has invalid param type");

		// transfer values to param types and param values //
		auto params = value_mat.front();
		std::vector<std::string> p_types(params.size());
		std::vector<std::any> p_values(params.size());
		for (int i = 0; i < params.size(); ++i)
		{
			p_types[i] = params[i].type_;
			p_values[i] = params[i].val();
		}

		// search functions //
		auto &funs = function_map_.find(std::string(i->word))->second.funs_;
		if (auto f = std::find_if(funs.begin(), funs.end(), [&](const auto &v)->bool {return std::equal(p_types.begin(), p_types.end(), std::get<0>(v).begin()); }); f == funs.end())
		{
			THROW_FILE_LINE("function \"" + std::string(i->word) + "\" not found");
		}
		else
		{
			i = e_par + 1;
			return Value{ std::get<1>(*f), std::get<2>(*f)(p_values) };
		}
	}
	auto Compiler::Imp::CaculateValueInOperator(TokenVec::iterator &i, TokenVec::iterator max_end_token)const->Value
	{
		auto opr = i;
		i = FindNextEqualLessPrecedenceBinaryOpr(opr + 1, max_end_token, opr->opr->priority_ul);
		auto value = CaculateTokens(opr + 1, i);
		auto func = opr->opr->ul_funs_.at(value.type_);
		return Value{func.first, func.second(value.val())};
	}
	auto Compiler::Imp::FindNextOutsideToken(TokenVec::iterator begin_token, TokenVec::iterator end_token, Token::Type type)const->TokenVec::iterator
	{
		Size par_num = 0, bra_num = 0, bce_num = 0;
		auto next_token = begin_token;

		for (;next_token < end_token; ++next_token)
		{
			if ((par_num == 0) && (bra_num == 0) && (bce_num == 0) && (next_token->type == type))return next_token;
			switch (next_token->type)
			{
			case Token::PARENTHESIS_L:par_num++; break;
			case Token::PARENTHESIS_R:par_num--; break;
			case Token::BRACKET_L:bra_num++; break;
			case Token::BRACKET_R:bra_num--; break;
			case Token::BRACE_L:bce_num++; break;
			case Token::BRACE_R:bce_num--; break;
			default:break;
			}
		}

		return end_token;
	}
	auto Compiler::Imp::FindNextEqualLessPrecedenceBinaryOpr(TokenVec::iterator begin_token, TokenVec::iterator end_token, Size precedence)const->TokenVec::iterator
	{
		auto next_opr = begin_token;
		for (; next_opr < end_token; ++next_opr)
		{
			next_opr = FindNextOutsideToken(next_opr, end_token, Token::OPERATOR);
			if ((next_opr == end_token) || (next_opr->opr->priority_b <= precedence))break;
		}

		return next_opr;
	}
	auto Compiler::Imp::GetValues(TokenVec::iterator b_tok, TokenVec::iterator e_tok)const->std::vector<std::vector<Value> >
	{
		std::vector<std::vector<Value> > ret;

		auto b_row = b_tok;
		while (b_row < e_tok)
		{
			auto e_row = FindNextOutsideToken(b_row, e_tok, Token::SEMICOLON);
			auto b_col = b_row;

			ret.push_back(std::vector<Value>());

			while (b_col < e_row)
			{
				auto e_col = FindNextOutsideToken(b_col, e_row, Token::COMMA);
				ret.back().push_back(CaculateTokens(b_col, e_col));
				b_col = e_col == e_tok ? e_col : e_col + 1;
			}

			b_row = e_row == e_tok ? e_row : e_row + 1;
		}

		return ret;
	}
	auto Compiler::calculateExpression(std::string_view expression) const->std::pair<std::string, std::any>
	{
		auto tokens = imp_->Expression2Tokens(expression);
		auto ret_val = imp_->CaculateTokens(tokens.begin(), tokens.end());
		return std::make_pair(std::move(ret_val.type_), std::move(ret_val.val()));
	}
	auto Compiler::addVariable(std::string_view var, std::string_view type, const std::any &value)->void
	{
		if (imp_->variable_map_.find(var) != imp_->variable_map_.end() ||
			imp_->function_map_.find(var) != imp_->function_map_.end())
		{
			THROW_FILE_LINE("\"" + std::string(var) + "already exists, can't add this variable");
		}

		imp_->variable_map_.insert(make_pair(var, Imp::Variable{std::string(type), std::string(var), value}));
	}
	auto Compiler::addFunction(std::string_view fun, const std::vector<std::string> &params, std::string_view ret_type, BuiltInFunction f)->void
	{
		if (imp_->variable_map_.find(fun) != imp_->variable_map_.end())
		{
			THROW_FILE_LINE("\"" + std::string(fun) + "already exists, can't add this function");
		}
		
		auto &funs = imp_->function_map_[std::string(fun)].funs_;
		if (std::find_if(funs.begin(), funs.end(), [&](const auto &val)->bool {	return std::equal(params.begin(), params.end(), std::get<0>(val).begin()); }) != funs.end())
			THROW_FILE_LINE("function \"" + std::string(fun) + "already exists, can't add function");

		funs.push_back(std::make_tuple(params, std::string(ret_type), f));
	}
	auto Compiler::addOperator(std::string_view opr, int ul_priority, int ur_priority, int b_priority)->void
	{
		if (imp_->operator_map_.find(opr) != imp_->operator_map_.end())THROW_FILE_LINE("Already has operator:" + std::string(opr));

		imp_->operator_map_[std::string(opr)].priority_ul = ul_priority;
		imp_->operator_map_[std::string(opr)].priority_ur = ur_priority;
		imp_->operator_map_[std::string(opr)].priority_b = b_priority;
	}
	auto Compiler::addUnaryLeftOperatorFunction(std::string_view opr, std::string_view p_type, std::string_view ret_type, UnaryOperatorFunction f)->void
	{
		if (auto found_opr = imp_->operator_map_.find(opr);found_opr == imp_->operator_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else if(auto found_type = imp_->type_map_.find(p_type); found_type == imp_->type_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(p_type));
		else if (auto found_func = found_opr->second.ul_funs_.find(p_type); found_func != found_opr->second.ul_funs_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else
		{
			found_opr->second.ul_funs_.insert(std::make_pair(std::string(p_type), std::make_pair(std::string(ret_type), f)));
		}
	}
	auto Compiler::addUnaryRightOperatorFunction(std::string_view opr, std::string_view p_type, std::string_view ret_type, UnaryOperatorFunction f)->void
	{
		if (auto found_opr = imp_->operator_map_.find(opr); found_opr == imp_->operator_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else if (auto found_type = imp_->type_map_.find(p_type); found_type == imp_->type_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(p_type));
		else if (auto found_func = found_opr->second.ur_funs_.find(p_type); found_func != found_opr->second.ur_funs_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else
		{
			found_opr->second.ur_funs_.insert(std::make_pair(std::string(p_type), std::make_pair(std::string(ret_type), f)));
		}
	}
	auto Compiler::addBinaryOperatorFunction(std::string_view opr, std::string_view p1_type, std::string_view p2_type, std::string_view ret_type, BinaryOperatorFunction f)->void
	{
		if (auto found_opr = imp_->operator_map_.find(opr); found_opr == imp_->operator_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else if (auto found_type1 = imp_->type_map_.find(p1_type); found_type1 == imp_->type_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(p1_type));
		else if (auto found_type2 = imp_->type_map_.find(p2_type); found_type2 == imp_->type_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(p2_type));
		else if (auto found = found_opr->second.b_funs_.find(p1_type); found != found_opr->second.b_funs_.end() && found->second.find(p2_type) != found->second.end())
			THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else
		{
			if (found == found_opr->second.b_funs_.end())
				found_opr->second.b_funs_.insert(std::make_pair(std::string(p1_type), std::map<std::string, std::pair<std::string, BinaryOperatorFunction>, std::less<>>()));

			found_opr->second.b_funs_[std::string(p1_type)][std::string(p2_type)] = std::make_pair(std::string(ret_type), f);
		}
	}
	auto Compiler::clearVariables()->void { imp_->variable_map_.clear(); }
	Compiler::~Compiler() = default;
	Compiler::Compiler(const std::string &name)
	{
		//imp_->operator_map_["+"].SetBinaryOpr(1, [](Value v1, Value v2) ->Value
		//{
		//	return Value{ "result", 1 };
		//});
		//imp_->operator_map_["+"].SetUnaryLeftOpr(1, [](Matrix m) {return m; });
		//imp_->operator_map_["-"].SetBinaryOpr(1, [](Matrix m1, Matrix m2) {return m1 - m2; });
		//imp_->operator_map_["-"].SetUnaryLeftOpr(1, [](Matrix m) {return -m; });
		//imp_->operator_map_["*"].SetBinaryOpr(2, [](Matrix m1, Matrix m2) {return m1 * m2; });
		//imp_->operator_map_["/"].SetBinaryOpr(2, [](Matrix m1, Matrix m2) {return m1 / m2; });

		imp_->type_map_["Number"];
		imp_->type_map_["Matrix"];

		addOperator("+", 1, 0, 1);
		addUnaryLeftOperatorFunction("+", "Number", "Number", [](std::any& p1)->std::any{return std::any_cast<double>(p1);});
		addBinaryOperatorFunction("+", "Number", "Number", "Number", [](std::any& p1, std::any&p2)->std::any 
		{
			return std::any_cast<double>(p1) + std::any_cast<double>(p2);
		});

		addOperator("-", 1, 0, 1);
		addUnaryLeftOperatorFunction("-", "Number", "Number", [](std::any& p1)->std::any {return -std::any_cast<double>(p1); });
		addBinaryOperatorFunction("-", "Number", "Number", "Number", [](std::any& p1, std::any&p2)->std::any
		{
			return std::any_cast<double>(p1) - std::any_cast<double>(p2);
		});

		addOperator("*", 0, 0, 2);
		addBinaryOperatorFunction("*", "Number", "Number", "Number", [](std::any& p1, std::any&p2)->std::any
		{
			return std::any_cast<double>(p1) * std::any_cast<double>(p2);
		});

		addOperator("/", 0, 0, 2);
		addBinaryOperatorFunction("/", "Number", "Number", "Number", [](std::any& p1, std::any&p2)->std::any
		{
			return std::any_cast<double>(p1) / std::any_cast<double>(p2);
		});

		addVariable("pi", "Number", double(3.141592653));

		addFunction("sin", std::vector<std::string>{"Number"}, "Number", [](std::vector<std::any> params)->std::any 
		{
			return std::sin(std::any_cast<double>(params[0]));
		});
	}
	Compiler::Compiler(const Compiler &) = default;
	Compiler::Compiler(Compiler &&) = default;
	Compiler& Compiler::operator=(const Compiler &) = default;
	Compiler& Compiler::operator=(Compiler &&) = default;
}