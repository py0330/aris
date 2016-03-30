#ifndef ARIS_EXPRESSION_CALCULATOR_H_
#define ARIS_EXPRESSION_CALCULATOR_H_

#include<vector>
#include<set>
#include<map>
#include<functional>
#include<string>
#include<list>
#include<iostream>
#include<iomanip>
#include<initializer_list>
#include<cmath>
#include<algorithm>

namespace aris
{
	namespace core
	{
		class Matrix
		{
		public:
			~Matrix() {};
			Matrix() :m_(0), n_(0), is_row_major_(true) {};
			Matrix(const Matrix &other) = default;
			Matrix(Matrix &&other) { this->swap(other); };
			Matrix &operator=(Matrix other) { this->swap(other); return *this; };
			Matrix(double value);
			Matrix(std::size_t m, std::size_t n, double value = 0);
			Matrix(std::size_t m, std::size_t n, const double *data);
			Matrix(const std::initializer_list<Matrix> &data);
			auto swap(Matrix &other)->Matrix&;
			auto empty() const->bool { return data_vec_.empty(); };
			auto size() const->std::size_t { return m()*n(); };
			auto data()->double * { return data_vec_.data(); };
			auto data() const->const double * { return data_vec_.data(); };
			auto begin() ->double * { return data(); };
			auto begin() const ->const double * { return data(); };
			auto end() ->double * { return data() + size(); };
			auto end()  const ->const double * { return data() + size(); };
			auto m() const->std::size_t { return m_; };
			auto n() const->std::size_t { return n_; };
			auto resize(std::size_t m, std::size_t n)->Matrix &;
			auto transpose()->Matrix &;
			
			auto copySubMatrixTo(const Matrix &subMat, std::size_t beginRow, std::size_t beginCol, std::size_t rowNum, std::size_t colNum)->void;
			auto copySubMatrixTo(const Matrix &subMat, std::size_t beginRow, std::size_t beginCol)->void { copySubMatrixTo(subMat, beginRow, beginCol, subMat.m(), subMat.n()); };

			auto toString() const->std::string;
			auto toDouble() const->double { return data()[0]; };
			auto dsp() const ->void
			{
				std::cout << this->toString();
			}

			auto operator()(std::size_t i, std::size_t j)->double &
			{
				return is_row_major_ ? data()[i*n() + j] : data()[j*m() + i];
			};
			auto operator()(std::size_t i, std::size_t j) const->const double &
			{
				return is_row_major_ ? data()[i*n() + j] : data()[j*m() + i];
			};
			
			friend Matrix operator + (const Matrix &m1, const Matrix &m2);
			friend Matrix operator - (const Matrix &m1, const Matrix &m2);
			friend Matrix operator * (const Matrix &m1, const Matrix &m2);
			friend Matrix operator / (const Matrix &m1, const Matrix &m2);
			friend Matrix operator - (const Matrix &m1);
			friend Matrix operator + (const Matrix &m1);

			template <typename MATRIX_LIST>
			friend Matrix combineColMatrices(const MATRIX_LIST &matrices);
			template <typename MATRIX_LIST>
			friend Matrix combineRowMatrices(const MATRIX_LIST &matrices);
			template <typename MATRIX_LISTLIST>
			friend Matrix combineMatrices(const MATRIX_LISTLIST &matrices);

			private:
				int m_, n_;
				bool is_row_major_;
				std::vector<double> data_vec_;
		};

		template <typename MATRIX_LIST>
		Matrix combineColMatrices(const MATRIX_LIST &matrices)
		{
			Matrix ret;

			/*获取全部矩阵的行数和列数*/
			for (const auto &mat : matrices)
			{
				if (mat.size() == 0)continue;
				if ((ret.n() != 0) && (ret.n() != mat.n()))throw std::runtime_error("input do not have valid size");

				ret.m_ += mat.m();
				ret.n_ = mat.n();
			}

			ret.resize(ret.m(), ret.n());

			/*赋值*/
			std::size_t beginRow = 0;
			for (const auto &mat : matrices)
			{
				for (std::size_t i = 0; i < mat.m(); i++)
				{
					for (std::size_t j = 0; j < mat.n(); j++)
					{
						ret(i + beginRow, j) = mat(i, j);
					}
				}
				beginRow += mat.m();
			}

			return ret;
		};
		template <typename MATRIX_LIST>
		Matrix combineRowMatrices(const MATRIX_LIST &matrices)
		{
			Matrix ret;

			/*获取全部矩阵的行数和列数*/
			for (const auto &mat : matrices)
			{
				if (mat.size() == 0)continue;
				if ((ret.m() != 0) && (ret.m() != mat.m()))throw std::runtime_error("input do not have valid size");

				ret.m_ = mat.m();
				ret.n_ += mat.n();
			}

			ret.resize(ret.m(), ret.n());

			/*赋值*/
			int beginCol = 0;
			for (const auto &mat : matrices)
			{
				for (std::size_t i = 0; i < mat.m(); i++)
				{
					for (std::size_t j = 0; j < mat.n(); j++)
					{
						ret(i, j + beginCol) = mat(i, j);
					}
				}
				beginCol += mat.n();
			}

			return ret;
		};
		template <typename MATRIX_LISTLIST>
		Matrix combineMatrices(const MATRIX_LISTLIST &matrices)
		{
			std::list<Matrix> mat_col_list;
			for (const auto &mat_list : matrices)mat_col_list.push_back(combineRowMatrices(mat_list));
			return combineColMatrices(mat_col_list);
		};

		class Calculator
		{
		public:
			Calculator()
			{
				operator_map_["+"].SetBinaryOpr(1, [](Matrix m1, Matrix m2) {return m1 + m2; });
				operator_map_["+"].SetUnaryLeftOpr(1, [](Matrix m) {return m; });
				operator_map_["-"].SetBinaryOpr(1, [](Matrix m1, Matrix m2) {return m1 - m2; });
				operator_map_["-"].SetUnaryLeftOpr(1, [](Matrix m) {return -m; });
				operator_map_["*"].SetBinaryOpr(2, [](Matrix m1, Matrix m2) {return m1 * m2; });
				operator_map_["/"].SetBinaryOpr(2, [](Matrix m1, Matrix m2) {return m1 / m2; });

				addFunction("sqrt", [](std::vector<Matrix> v)
				{
					Matrix ret = v.front();

					for (auto &d : ret)	d = std::sqrt(d);

					return ret;
				}, 1);
			}

			auto calculateExpression(const std::string &expression) const->Matrix;
			auto evaluateExpression(const std::string &expression)const->std::string;
			auto addVariable(const std::string &name, const Matrix &value)->void;
			auto addVariable(const std::string &name, const std::string &value)->void;
			auto addFunction(const std::string &name, std::function<Matrix(std::vector<Matrix>)> f, int n)->void;
			auto clearVariables()->void { variable_map_.clear(); string_map_.clear(); };

		private:
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
				std::string word;

				double num;
				const Matrix *var;
				const Calculator::Function *fun;
				const Calculator::Operator *opr;
			};
			class Operator
			{
			public:
				std::string name;

				int priority_ul;///unary left
				int priority_ur;///unary right
				int priority_b;///binary

				typedef std::function<Matrix(Matrix)> U_FUN;
				typedef std::function<Matrix(Matrix, Matrix)> B_FUN;

				U_FUN fun_ul;
				U_FUN fun_ur;
				B_FUN fun_b;

				Operator() :priority_ul(0), priority_ur(0), priority_b(0){};

				void SetUnaryLeftOpr(int priority, U_FUN fun){ priority_ul = priority; this->fun_ul = fun; };
				void SetUnaryRightOpr(int priority, U_FUN fun){ priority_ur = priority; this->fun_ur = fun; };
				void SetBinaryOpr(int priority, B_FUN fun){ priority_b = priority; this->fun_b = fun; };
			};
			class Function
			{
				typedef std::function<Matrix(std::vector<Matrix>)> FUN;
			public:
				std::string name;
				std::map<int, FUN> funs;

				void AddOverloadFun(int n, FUN fun){ funs.insert(make_pair(n, fun)); };
			};

			typedef std::vector<Token> TokenVec;
			TokenVec Expression2Tokens(const std::string &expression)const;
			Matrix CaculateTokens(TokenVec::iterator beginToken, TokenVec::iterator maxEndToken) const;

			Matrix CaculateValueInParentheses(TokenVec::iterator &i, TokenVec::iterator maxEndToken) const;
			Matrix CaculateValueInBraces(TokenVec::iterator &i, TokenVec::iterator maxEndToken) const;
			Matrix CaculateValueInFunction(TokenVec::iterator &i, TokenVec::iterator maxEndToken) const;
			Matrix CaculateValueInOperator(TokenVec::iterator &i, TokenVec::iterator maxEndToken) const;

			TokenVec::iterator FindNextOutsideToken(TokenVec::iterator leftPar, TokenVec::iterator endToken, Token::Type type) const;
			TokenVec::iterator FindNextEqualLessPrecedenceBinaryOpr(TokenVec::iterator beginToken, TokenVec::iterator endToken, int precedence)const;
			std::vector<std::vector<Matrix> > GetMatrices(TokenVec::iterator beginToken, TokenVec::iterator endToken)const;
		
		private:
			std::map<std::string, Operator> operator_map_;
			std::map<std::string, Function> function_map_;
			std::map<std::string, Matrix> variable_map_;
			std::map<std::string, std::string> string_map_;//string variable
		};
	}
}






#endif