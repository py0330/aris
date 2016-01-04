#ifndef ARIS_EXPCAL_H_
#define ARIS_EXPCAL_H_

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
	namespace DynKer
	{
		class Matrix
		{
		public:
			Matrix(double value);
			Matrix(int m, int n);
			Matrix(int m, int n, const double *Data);
			Matrix(const std::initializer_list<Matrix> &Data) :Matrix()
			{
				std::list<std::list<Matrix> > matListList;

				bool needNewRow = true;
				for (auto &i : Data)
				{
					if (i.Empty())
					{
						needNewRow = true;
					}
					else
					{
						if (needNewRow)
						{
							matListList.push_back(std::list<Matrix>());
							needNewRow = false;
						}

						matListList.back().push_back(i);
					}
				}


				(*this) = Aris::DynKer::CombineMatrices(matListList);
			}

			Matrix() :m(0), n(0), isRowMajor(true), pData(nullptr) {};
			Matrix(const Matrix &other) :m(other.m), n(other.n), isRowMajor(true), pData(m*n > 0 ? new double[m*n] : nullptr) { std::copy_n(other.pData, m*n, pData); };
			Matrix(Matrix &&other) :Matrix() { this->Swap(other); };
			Matrix &operator=(Matrix other) { this->Swap(other);	return *this; };
			Matrix &Swap(Matrix &other)
			{
				std::swap(this->m, other.m);
				std::swap(this->n, other.n);
				std::swap(this->isRowMajor, other.isRowMajor);
				std::swap(this->pData, other.pData);

				return *this;
			}
			~Matrix();


			bool Empty() const { return !pData; }
			int RowNum() const { return m; }
			int ColNum() const { return n; }
			int Length() const { return m*n; };
			double * Data() { return pData; };
			const double * Data() const { return pData; };

			void Transpose() { isRowMajor = !isRowMajor; std::swap(m, n); };
			void Resize(int m, int n);
			void CopySubMatrixTo(const Matrix &subMat, int beginRow, int beginCol, int rowNum, int colNum);
			void CopySubMatrixTo(const Matrix &subMat, int beginRow, int beginCol){ CopySubMatrixTo(subMat, beginRow, beginCol, subMat.m, subMat.n); };
			void ForEachElement(std::function<double(double)> f)
			{
				for (int i = 0; i < Length(); ++i)
				{
					pData[i] = f(pData[i]);
				}
			};

			double & operator()(int i, int j)
			{ 
				return isRowMajor ? pData[i*n + j] : pData[j*m + i];
			};
			double operator() (int i, int j) const
			{
				return isRowMajor ? pData[i*n + j] : pData[j*m + i];
			};
			Matrix operator()(const Matrix &i, const Matrix &j) const
			{
				Matrix ret;
				ret.Resize(i.Length(), j.Length());

				for (int a = 0; a < i.Length(); ++a)
				{
					for (int b = 0; b < j.Length(); ++b)
					{
						if ((i.pData[a] < 0) || ((int)i.pData[a] >= m) || (j.pData[b] < 0) || (( int)j.pData[b] >= n))
						{
							throw std::logic_error("matrix index out of range");
						}
						else
						{
							ret(a, b) = this->operator()((int)i.pData[a], (int)j.pData[b]);
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
			friend Matrix CombineColMatrices(const MATRIX_LIST &matrices);
			template <typename MATRIX_LIST>
			friend Matrix CombineRowMatrices(const MATRIX_LIST &matrices);
			template <typename MATRIX_LISTLIST>
			friend Matrix CombineMatrices(const MATRIX_LISTLIST &matrices);

			std::string ToString() const;
			double ToDouble() const { return pData[0]; };
			void dsp()
			{
				std::cout << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right) << std::setprecision(5);

				std::cout << std::endl;
				for ( int i = 0; i < m; i++)
				{
					for ( int j = 0; j < n; j++)
					{
						std::cout << this->operator()(i,j) << "   ";
					}
					std::cout << std::endl;
				}
				std::cout << std::endl;
			}

			
			private:
				int m, n;
				bool isRowMajor;
				double *pData;
		};

		template <typename MATRIX_LIST>
		Matrix CombineColMatrices(const MATRIX_LIST &matrices)
		{
			Matrix ret;

			/*获取全部矩阵的行数和列数*/
			for (const auto &i : matrices)
			{
				if (i.Length() == 0)
					continue;

				ret.m += i.m;
				ret.n = std::max(i.n, ret.n);
			}

			/*判断所有矩阵是否拥有一样的列数*/
			for (const auto &i : matrices)
			{
				if (i.Length() == 0)
					continue;

				if (ret.n != i.n)
				{
					ret.m = 0;
					ret.n = 0;
					throw std::logic_error("input do not have valid size");
				}
			}

			/*判断最后的矩阵是否为空*/
			if (ret.m*ret.n == 0)
				return ret;

			/*申请内存*/
			ret.pData = new double[ret.m*ret.n];

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
				beginRow += a.m;
			}

			return ret;
		};
		template <typename MATRIX_LIST>
		Matrix CombineRowMatrices(const MATRIX_LIST &matrices)
		{
			Matrix ret;

			/*获取全部矩阵的行数和列数*/
			for (const auto &i : matrices)
			{
				if (i.Length() == 0)
					continue;

				ret.m = i.m;
				ret.n += i.n;
			}

			/*判断所有矩阵是否拥有一样的列数*/
			for (const auto &i : matrices)
			{
				if (i.Length() == 0)
					continue;

				if (ret.m != i.m)
				{
					ret.m = 0;
					ret.n = 0;
					throw std::logic_error("input do not have valid size");
				}
			}

			/*判断最后的矩阵是否为空*/
			if (ret.m*ret.n == 0)
				return ret;

			/*申请内存*/
			ret.pData = new double[ret.m*ret.n];

			/*赋值*/
			int beginCol = 0;
			for (const auto &a : matrices)
			{
				for (int i = 0; i < a.m; i++)
				{
					for (int j = 0; j < a.n; j++)
					{
						ret(i, j + beginCol) = a(i, j);
					}
				}
				beginCol += a.n;
			}

			return ret;
		};
		template <typename MATRIX_LISTLIST>
		Matrix CombineMatrices(const MATRIX_LISTLIST &matrices)
		{
			Matrix ret;

			/*获取全部矩阵的行数和列数*/
			for (const auto &i : matrices)
			{
				int loc_m = 0, loc_n = 0;

				/*获得行矩阵们的总列数和行数*/
				for (const auto &j : i)
				{
					if (j.Length() == 0)
						continue;

					if (loc_m == 0)
						loc_m = j.m;

					if (loc_m != j.m)
					{
						ret.m = 0;
						ret.n = 0;
						throw std::logic_error("input do not have valid size");
					}

					loc_n += j.n;
				}

				if (loc_m*loc_n == 0)
					continue;

				if (ret.n == 0)
					ret.n = loc_n;

				if (ret.n != loc_n)
				{
					ret.m = 0;
					ret.n = 0;
					throw std::logic_error("input do not have valid size");
				}

				ret.m += loc_m;
			}


			/*判断最后的矩阵是否为空*/
			if (ret.m*ret.n == 0)
				return ret;

			/*申请内存*/
			ret.pData = new double[ret.m*ret.n];

			/*赋值*/
			int beginRow = 0;
			for (const auto &a : matrices)
			{
				int beginCol = 0;
				int locRow = 0;

				for (const auto &b : a)
				{
					for (int i = 0; i < b.m; i++)
					{
						for (int j = 0; j < b.n; j++)
						{
							ret(i + beginRow, j + beginCol) = b(i, j);
						}
					}
					beginCol += b.n;

					if (b.m != 0)
						locRow = b.m;

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
			TOKENS tokens;
			TOKENS Expression2Tokens(const std::string &expression);
			Matrix CaculateTokens(TOKENS::iterator beginToken, TOKENS::iterator endToken);
			
			Matrix CaculateValueInParentheses(TOKENS::iterator &i, TOKENS::iterator endToken);
			Matrix CaculateValueInBraces(TOKENS::iterator &i, TOKENS::iterator endToken);
			Matrix CaculateValueInFunction(TOKENS::iterator &i, TOKENS::iterator endToken);
			Matrix CaculateValueInOperator(TOKENS::iterator &i, TOKENS::iterator endToken);

			TOKENS::iterator FindNextOutsideToken(TOKENS::iterator leftPar, TOKENS::iterator endToken, TOKEN::TYPE type);
			TOKENS::iterator FindNextEqualLessPrecedenceBinaryOpr(TOKENS::iterator beginToken, TOKENS::iterator endToken, int precedence);
			std::vector<std::vector<Matrix> > GetMatrices(TOKENS::iterator beginToken, TOKENS::iterator endToken);
		
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
					ret.ForEachElement([](double d){return std::sqrt(d); });

					return ret;
				}, 1);
			}

			Matrix CalculateExpression(const std::string &expression);
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