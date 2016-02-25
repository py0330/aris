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

namespace Aris
{
	namespace Core
	{
		class Matrix
		{
		public:
			Matrix(double value);
			Matrix(int m, int n);
			Matrix(int m, int n, const double *data);
			Matrix(const std::initializer_list<Matrix> &data) :Matrix()
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


				(*this) = Aris::Core::combineMatrices(mat_list_list);
			}

			Matrix() :m_(0), n_(0), is_row_major_(true), data_(nullptr) {};
			Matrix(const Matrix &other) :m_(other.m()), n_(other.n()), is_row_major_(true), data_(m()*n() > 0 ? new double[m()*n()] : nullptr) { std::copy_n(other.data_, m()*n(), data_); };
			Matrix(Matrix &&other) :Matrix() { this->swap(other); };
			Matrix &operator=(Matrix other) { this->swap(other);	return *this; };
			Matrix &swap(Matrix &other)
			{
				std::swap(this->m_, other.m_);
				std::swap(this->n_, other.n_);
				std::swap(this->is_row_major_, other.is_row_major_);
				std::swap(this->data_, other.data_);

				return *this;
			}
			~Matrix();


			bool empty() const { return !data_; }
			int m() const { return m_; };
			int n() const { return n_; };
			int rowNum() const { return m(); }
			int colNum() const { return n(); }
			int length() const { return m()*n(); };
			double * data() { return data_; };
			const double * data() const { return data_; };

			void transpose() { is_row_major_ = !is_row_major_; std::swap(m_, n_); };
			void resize(int m, int n);
			void copySubMatrixTo(const Matrix &subMat, int beginRow, int beginCol, int rowNum, int colNum);
			void copySubMatrixTo(const Matrix &subMat, int beginRow, int beginCol){ copySubMatrixTo(subMat, beginRow, beginCol, subMat.m(), subMat.n()); };
			void forEachElement(std::function<double(double)> f)
			{
				for (int i = 0; i < length(); ++i)
				{
					data()[i] = f(data()[i]);
				}
			};

			double & operator()(int i, int j)
			{ 
				return is_row_major_ ? data()[i*n() + j] : data()[j*m() + i];
			};
			double operator() (int i, int j) const
			{
				return is_row_major_ ? data()[i*n() + j] : data()[j*m() + i];
			};
			Matrix operator()(const Matrix &i, const Matrix &j) const
			{
				Matrix ret;
				ret.resize(i.length(), j.length());

				for (int a = 0; a < i.length(); ++a)
				{
					for (int b = 0; b < j.length(); ++b)
					{
						if ((i.data()[a] < 0) || ((int)i.data()[a] >= m()) || (j.data()[b] < 0) || (( int)j.data()[b] >= n()))
						{
							throw std::logic_error("matrix index out of range");
						}
						else
						{
							ret(a, b) = this->operator()((int)i.data()[a], (int)j.data()[b]);
						}

					}
				}

				return ret;
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

			std::string toString() const;
			double toDouble() const { return data()[0]; };
			void dsp()
			{
				std::cout << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right) << std::setprecision(5);

				std::cout << std::endl;
				for ( int i = 0; i < m(); i++)
				{
					for ( int j = 0; j < n(); j++)
					{
						std::cout << this->operator()(i,j) << "   ";
					}
					std::cout << std::endl;
				}
				std::cout << std::endl;
			}

			private:
				int m_, n_;
				bool is_row_major_;
				double *data_;
		};

		template <typename MATRIX_LIST>
		Matrix combineColMatrices(const MATRIX_LIST &matrices)
		{
			Matrix ret;

			/*获取全部矩阵的行数和列数*/
			for (const auto &i : matrices)
			{
				if (i.length() == 0)
					continue;

				ret.m_ += i.m();
				ret.n_ = std::max(i.n(), ret.n());
			}

			/*判断所有矩阵是否拥有一样的列数*/
			for (const auto &i : matrices)
			{
				if (i.length() == 0)
					continue;

				if (ret.n() != i.n())
				{
					ret.m_ = 0;
					ret.n_ = 0;
					throw std::logic_error("input do not have valid size");
				}
			}

			/*判断最后的矩阵是否为空*/
			if (ret.m()*ret.n() == 0)
				return ret;

			/*申请内存*/
			ret.data_ = new double[ret.m()*ret.n()];

			/*赋值*/
			int beginRow = 0;
			for (const auto &a : matrices)
			{
				for (int i = 0; i < a.m; i++)
				{
					for (int j = 0; j < a.n; j++)
					{
						ret(i + beginRow, j) = a(i, j);
					}
				}
				beginRow += a.m();
			}

			return ret;
		};
		template <typename MATRIX_LIST>
		Matrix combineRowMatrices(const MATRIX_LIST &matrices)
		{
			Matrix ret;

			/*获取全部矩阵的行数和列数*/
			for (const auto &i : matrices)
			{
				if (i.length() == 0)
					continue;

				ret.m_ = i.m();
				ret.n_ += i.n();
			}

			/*判断所有矩阵是否拥有一样的列数*/
			for (const auto &i : matrices)
			{
				if (i.length() == 0)
					continue;

				if (ret.m() != i.m())
				{
					ret.m_ = 0;
					ret.n_ = 0;
					throw std::logic_error("input do not have valid size");
				}
			}

			/*判断最后的矩阵是否为空*/
			if (ret.m()*ret.n() == 0)
				return ret;

			/*申请内存*/
			ret.data_ = new double[ret.m()*ret.n()];

			/*赋值*/
			int beginCol = 0;
			for (const auto &a : matrices)
			{
				for (int i = 0; i < a.m(); i++)
				{
					for (int j = 0; j < a.n(); j++)
					{
						ret(i, j + beginCol) = a(i, j);
					}
				}
				beginCol += a.n;
			}

			return ret;
		};
		template <typename MATRIX_LISTLIST>
		Matrix combineMatrices(const MATRIX_LISTLIST &matrices)
		{
			Matrix ret;

			/*获取全部矩阵的行数和列数*/
			for (const auto &i : matrices)
			{
				int loc_m = 0, loc_n = 0;

				/*获得行矩阵们的总列数和行数*/
				for (const auto &j : i)
				{
					if (j.length() == 0)
						continue;

					if (loc_m == 0)
						loc_m = j.m();

					if (loc_m != j.m())
					{
						ret.m_ = 0;
						ret.n_ = 0;
						throw std::logic_error("input do not have valid size");
					}

					loc_n += j.n();
				}

				if (loc_m*loc_n == 0)
					continue;

				if (ret.n() == 0)
					ret.n_ = loc_n;

				if (ret.n() != loc_n)
				{
					ret.m_ = 0;
					ret.n_ = 0;
					throw std::logic_error("input do not have valid size");
				}

				ret.m_ += loc_m;
			}


			/*判断最后的矩阵是否为空*/
			if (ret.m()*ret.n() == 0)
				return ret;

			/*申请内存*/
			ret.data_ = new double[ret.m()*ret.n()];

			/*赋值*/
			int beginRow = 0;
			for (const auto &a : matrices)
			{
				int beginCol = 0;
				int locRow = 0;

				for (const auto &b : a)
				{
					for (int i = 0; i < b.m(); i++)
					{
						for (int j = 0; j < b.n(); j++)
						{
							ret(i + beginRow, j + beginCol) = b(i, j);
						}
					}
					beginCol += b.n();

					if (b.m())locRow = b.m();

				}
				beginRow += locRow;
			}

			return ret;
		};

		class Calculator
		{
		private:
			class OPERATOR;
			class FUNCTION;

			class TOKEN
			{
			public:
				enum TYPE
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
					CONST_VALUE,    //const matrix
					OPERATOR,    //operator
					VARIABLE,    //variable
					FUNCTION     //function
				};

				TYPE type;

				int id;

				std::string word;
				Matrix cst;
				const Matrix *var;
				const Calculator::FUNCTION *fun;
				const Calculator::OPERATOR *opr;
			};
			class OPERATOR
			{
			public:
				std::string name;

				int priority_ul;
				int priority_ur;
				int priority_b;

				typedef std::function<Matrix(Matrix)> U_FUN;
				typedef std::function<Matrix(Matrix, Matrix)> B_FUN;

				U_FUN fun_ul;
				U_FUN fun_ur;
				B_FUN fun_b;

				OPERATOR() :priority_ul(0), priority_ur(0), priority_b(0){};

				void SetUnaryLeftOpr(int priority, U_FUN fun){ priority_ul = priority; this->fun_ul = fun; };
				void SetUnaryRightOpr(int priority, U_FUN fun){ priority_ur = priority; this->fun_ur = fun; };
				void SetBinaryOpr(int priority, B_FUN fun){ priority_b = priority; this->fun_b = fun; };
			};
			class FUNCTION
			{
				typedef std::function<Matrix(std::vector<Matrix>)> FUN;
			public:
				std::string name;
				std::map<int, FUN> funs;

				void AddOverloadFun(int n, FUN fun){ funs.insert(make_pair(n, fun)); };
			};

			typedef std::vector<TOKEN> TOKENS;
			mutable TOKENS tokens;
			TOKENS Expression2Tokens(const std::string &expression)const;
			Matrix CaculateTokens(TOKENS::iterator beginToken, TOKENS::iterator endToken) const;
			
			Matrix CaculateValueInParentheses(TOKENS::iterator &i, TOKENS::iterator endToken) const;
			Matrix CaculateValueInBraces(TOKENS::iterator &i, TOKENS::iterator endToken) const;
			Matrix CaculateValueInFunction(TOKENS::iterator &i, TOKENS::iterator endToken) const;
			Matrix CaculateValueInOperator(TOKENS::iterator &i, TOKENS::iterator endToken) const;

			TOKENS::iterator FindNextOutsideToken(TOKENS::iterator leftPar, TOKENS::iterator endToken, TOKEN::TYPE type) const;
			TOKENS::iterator FindNextEqualLessPrecedenceBinaryOpr(TOKENS::iterator beginToken, TOKENS::iterator endToken, int precedence)const;
			std::vector<std::vector<Matrix> > GetMatrices(TOKENS::iterator beginToken, TOKENS::iterator endToken)const;
		
		private:
			std::map<std::string, OPERATOR> operators;
			std::map<std::string, FUNCTION> functions;
			std::map<std::string, Matrix> variables;

			

		public:
			Calculator()
			{
				operators["+"].SetBinaryOpr(1, [](Matrix m1, Matrix m2){return m1 + m2; });
				operators["+"].SetUnaryLeftOpr(1, [](Matrix m){return m; });
				operators["-"].SetBinaryOpr(1, [](Matrix m1, Matrix m2){return m1 - m2; });
				operators["-"].SetUnaryLeftOpr(1, [](Matrix m){return -m; });
				operators["*"].SetBinaryOpr(2, [](Matrix m1, Matrix m2){return m1 * m2; });
				operators["/"].SetBinaryOpr(2, [](Matrix m1, Matrix m2){return m1 / m2; });

				AddFunction("sqrt", [](std::vector<Matrix> v)
				{
					Matrix ret = v.front();
					ret.forEachElement([](double d){return std::sqrt(d); });

					return ret;
				}, 1);
			}

			Matrix CalculateExpression(const std::string &expression) const;
			void AddVariable(const std::string &name, const Matrix &value);
			void AddFunction(const std::string &name, std::function<Matrix(std::vector<Matrix>)> f,int n)
			{
				functions[name].AddOverloadFun(n,f);
			};
			void ClearVariables()
			{
				variables.clear();
			};
		};
	}
}






#endif