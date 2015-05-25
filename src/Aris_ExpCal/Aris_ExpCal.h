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

#include<cblas.h>

namespace Aris
{
	namespace DynKer
	{
		class MATRIX
		{
		private:
			unsigned int m, n;
			double *pData;
			bool isRowMajor;

		public:
			MATRIX();
			MATRIX(const MATRIX &other);
			MATRIX(MATRIX &&other);
			MATRIX &operator=(const MATRIX &other);
			MATRIX &operator=(MATRIX &&other);
			~MATRIX();

			MATRIX(double value);
			MATRIX(unsigned int m, unsigned int n);
			MATRIX(unsigned int m, unsigned int n, const double *Data);
			MATRIX(const std::initializer_list<double> &Data);
			MATRIX(const std::initializer_list<std::vector<MATRIX> > &matrices);

			unsigned RowNum()
			{
				return m;
			}
			unsigned ColNum()
			{
				return n;
			}

			void Transpose(){ isRowMajor = !isRowMajor; int loc = m; m = n; n = loc; };
			void Resize(unsigned int m, unsigned int n);
			unsigned int Length() const { return m*n; };
			double * Data(){ return pData; };
			const double * Data() const{ return pData; };
			void CopySubMatrixTo(const MATRIX &subMat, unsigned beginRow, unsigned beginCol,unsigned rowNum,unsigned colNum);
			void CopySubMatrixTo(const MATRIX &subMat, unsigned beginRow, unsigned beginCol){ CopySubMatrixTo(subMat, beginRow, beginCol, subMat.m, subMat.n); };
			void ForEachElement(std::function<double(double)> f)
			{
				for (unsigned i = 0; i < Length(); ++i)
				{
					pData[i] = f(pData[i]);
				}
			};

			double & operator()(unsigned int i, unsigned int j)
			{ 
				return isRowMajor ? pData[i*n + j] : pData[j*m + i];
			};
			double operator() (unsigned int i, unsigned int j) const
			{
				return isRowMajor ? pData[i*n + j] : pData[j*m + i];
			};
			MATRIX operator()(const MATRIX &i, const MATRIX &j) const
			{
				MATRIX ret;
				ret.Resize(i.Length(), j.Length());

				for (unsigned int a = 0; a < i.Length(); ++a)
				{
					for (unsigned int b = 0; b < j.Length(); ++b)
					{
						if ((i.pData[a] < 0) || ((unsigned int)i.pData[a] >= m) || (j.pData[b] < 0) || ((unsigned int)j.pData[b] >= n))
						{
							throw std::logic_error("matrix index out of range");
						}
						else
						{
							ret(a, b) = this->operator()((unsigned)i.pData[a], (unsigned)j.pData[b]);
						}

					}
				}

				return ret;
			};
			
			friend MATRIX operator + (const MATRIX &m1, const MATRIX &m2)
			{
				MATRIX ret;
				
				if ((m1.m == 1) && (m1.n == 1))
				{
					ret = MATRIX(m2);

					for (unsigned i = 0; i < ret.Length(); ++i)
					{
						ret.pData[i] += m1(0, 0);
					}
				}
				else if ((m2.m == 1) && (m2.n == 1))
				{
					ret = MATRIX(m1);

					for (unsigned i = 0; i < ret.Length(); ++i)
					{
						ret.pData[i] += m2(0, 0);
					}
				}
				else if ((m1.m == m2.m) && (m1.n == m2.n))
				{
					ret.Resize(m1.m, m1.n);

					for (unsigned i = 0; i < ret.m; i++)
					{
						for (unsigned j = 0; j < ret.n; j++)
						{
							ret(i, j) = m1(i, j) + m2(i, j);
						}
					}
				}
				else
				{
					throw std::logic_error("Can't plus matrices, the dimensions are not equal");
				}
				return ret;
			}
			friend MATRIX operator - (const MATRIX &m1, const MATRIX &m2)
			{
				MATRIX ret;
				
				if ((m1.m == 1) && (m1.n == 1))
				{
					ret = MATRIX(m2);

					for (unsigned i = 0; i < ret.Length(); ++i)
					{
						ret.pData[i] = m1(0, 0) - ret.pData[i];
					}
				}
				else if ((m2.m == 1) && (m2.n == 1))
				{
					ret = MATRIX(m1);

					for (unsigned i = 0; i < ret.Length(); ++i)
					{
						ret.pData[i] -= m2(0, 0);
					}
				}
				else if ((m1.m == m2.m) && (m1.n == m2.n))
				{
					ret.Resize(m1.m, m1.n);

					for (unsigned i = 0; i < ret.m; i++)
					{
						for (unsigned j = 0; j < ret.n; j++)
						{
							ret(i, j) = m1(i, j) - m2(i, j);
						}
					}
				}
				else
				{
					throw std::logic_error("Can't minus matrices, the dimensions are not equal");
				}

				return ret;
			}
			friend MATRIX operator * (const MATRIX &m1, const MATRIX &m2)
			{
				MATRIX ret;
				
				if ((m1.m == 1) && (m1.n == 1))
				{
					ret=MATRIX(m2);
					
					for (unsigned i = 0; i < ret.Length(); ++i)
					{
						ret.pData[i] *= m1(0, 0);
					}
				}
				else if ((m2.m == 1) && (m2.n == 1))
				{
					ret = MATRIX(m1);

					for (unsigned i = 0; i < ret.Length(); ++i)
					{
						ret.pData[i] *= m2(0, 0);
					}
				}
				else if (m1.n == m2.m)
				{
					ret.Resize(m1.m, m2.n);

					if (m1.isRowMajor)
					{
						if (m2.isRowMajor)
						{
							cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 
								m1.m, m2.n, m1.n, 
								1, m1.Data(), m1.n, m2.Data(), m2.n, 
								0, ret.Data(), ret.m);
						}
						else
						{
							cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
								m1.m, m2.n, m1.n,
								1, m1.Data(), m1.n, m2.Data(), m2.m,
								0, ret.Data(), ret.m);
						}
					}
					else
					{
						if (m2.isRowMajor)
						{
							cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans,
								m1.m, m2.n, m1.n,
								1, m1.Data(), m1.m, m2.Data(), m2.n,
								0, ret.Data(), ret.m);
						}
						else
						{
							cblas_dgemm(CblasRowMajor, CblasTrans, CblasTrans,
								m1.m, m2.n, m1.n,
								1, m1.Data(), m1.m, m2.Data(), m2.m,
								0, ret.Data(), ret.m);
						}
					}

					
				}
				else
				{
					throw std::logic_error("Can't multiply matrices, the dimensions are not equal");
				}

				return ret;
			}
			friend MATRIX operator / (const MATRIX &m1, const MATRIX &m2)
			{
				MATRIX ret;

				if ((m1.m == 1) && (m1.n == 1))
				{
					ret = MATRIX(m2);

					for (unsigned i = 0; i < ret.Length(); ++i)
					{
						ret.pData[i] = m1(0, 0)/ret.pData[i];
					}
				}
				else if ((m2.m == 1) && (m2.n == 1))
				{
					ret = MATRIX(m1);

					for (unsigned i = 0; i < ret.Length(); ++i)
					{
						ret.pData[i] /= m2(0, 0);
					}
				}
				else
				{
					throw std::logic_error("Right now, divide operator of matrices is not added");
				}

				return ret;
			}
			friend MATRIX operator - (const MATRIX &m1)
			{
				MATRIX m;
				m.Resize(m1.m, m1.n);

				for (unsigned int i = 0; i < m1.m*m1.n; i++)
				{
					m.pData[i] = -m1.pData[i];
				}
				return m;
			}
			friend MATRIX operator + (const MATRIX &m1)
			{
				return m1;
			}

			template <typename MATRIX_LIST>
			friend MATRIX CombineColMatrices(const MATRIX_LIST &matrices)
			{
				MATRIX ret;

				/*获取全部矩阵的行数和列数*/
				for (const auto &i : matrices)
				{
					if (i.Length() == 0)
						continue;

					ret.m += i.m;
					ret.n = i.n;
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
				unsigned int beginRow = 0;
				for (const auto &a : matrices)
				{
					for (unsigned i = 0; i < a.m; i++)
					{
						for (unsigned j = 0; j < a.n; j++)
						{
							ret(i + beginRow, j) = a(i, j);
						}
					}
					beginRow += a.m;
				}

				return ret;
			};
			template <typename MATRIX_LIST>
			friend MATRIX CombineRowMatrices(const MATRIX_LIST &matrices)
			{
				MATRIX ret;

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
				unsigned int beginCol = 0;
				for (const auto &a : matrices)
				{
					for (unsigned i = 0; i < a.m; i++)
					{
						for (unsigned j = 0; j < a.n; j++)
						{
							ret(i, j + beginCol) = a(i, j);
						}
					}
					beginCol += a.n;
				}

				return ret;
			};
			template <typename MATRIX_LISTLIST>
			friend MATRIX CombineMatrices(const MATRIX_LISTLIST &matrices)
			{
				MATRIX ret;

				/*获取全部矩阵的行数和列数*/
				for (const auto &i : matrices)
				{
					unsigned loc_m = 0, loc_n = 0;

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
				unsigned beginRow = 0;
				for (const auto &a : matrices)
				{
					unsigned beginCol = 0;
					unsigned locRow = 0;

					for (const auto &b : a)
					{
						for (unsigned i = 0; i < b.m; i++)
						{
							for (unsigned j = 0; j < b.n; j++)
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

			std::string ToString() const;
			double ToDouble() const;
			void dsp()
			{
				std::cout << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right) << std::setprecision(5);

				std::cout << std::endl;
				for (unsigned int i = 0; i < m; i++)
				{
					for (unsigned int j = 0; j < n; j++)
					{
						std::cout << this->operator()(i,j) << "   ";
					}
					std::cout << std::endl;
				}
				std::cout << std::endl;
			}
		};

		class CALCULATOR
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
				MATRIX cst;
				const MATRIX *var;
				const CALCULATOR::FUNCTION *fun;
				const CALCULATOR::OPERATOR *opr;
			};
			class OPERATOR
			{
			public:
				std::string name;

				unsigned int priority_ul;
				unsigned int priority_ur;
				unsigned int priority_b;

				typedef std::function<MATRIX(MATRIX)> U_FUN;
				typedef std::function<MATRIX(MATRIX, MATRIX)> B_FUN;

				U_FUN fun_ul;
				U_FUN fun_ur;
				B_FUN fun_b;

				OPERATOR() :priority_ul(0), priority_ur(0), priority_b(0){};

				void SetUnaryLeftOpr(unsigned int priority, U_FUN fun){ priority_ul = priority; this->fun_ul = fun; };
				void SetUnaryRightOpr(unsigned int priority, U_FUN fun){ priority_ur = priority; this->fun_ur = fun; };
				void SetBinaryOpr(unsigned int priority, B_FUN fun){ priority_b = priority; this->fun_b = fun; };
			};
			class FUNCTION
			{
				typedef std::function<MATRIX(std::vector<MATRIX>)> FUN;
			public:
				std::string name;
				std::map<unsigned int, FUN> funs;

				void AddOverloadFun(unsigned int n, FUN fun){ funs.insert(make_pair(n, fun)); };
			};

			typedef std::vector<TOKEN> TOKENS;
			TOKENS tokens;
			TOKENS Expression2Tokens(const std::string &expression);
			MATRIX CaculateTokens(TOKENS::iterator beginToken, TOKENS::iterator endToken);
			
			MATRIX CaculateValueInParentheses(TOKENS::iterator &i, TOKENS::iterator endToken);
			MATRIX CaculateValueInBraces(TOKENS::iterator &i, TOKENS::iterator endToken);
			MATRIX CaculateValueInFunction(TOKENS::iterator &i, TOKENS::iterator endToken);
			MATRIX CaculateValueInOperator(TOKENS::iterator &i, TOKENS::iterator endToken);

			TOKENS::iterator FindNextOutsideToken(TOKENS::iterator leftPar, TOKENS::iterator endToken, TOKEN::TYPE type);
			TOKENS::iterator FindNextEqualLessPrecedenceBinaryOpr(TOKENS::iterator beginToken, TOKENS::iterator endToken, unsigned precedence);
			std::vector<std::vector<MATRIX> > GetMatrices(TOKENS::iterator beginToken, TOKENS::iterator endToken);
		
		private:
			std::map<std::string, OPERATOR> operators;
			std::map<std::string, FUNCTION> functions;
			std::map<std::string, MATRIX> variables;

			

		public:
			CALCULATOR()
			{
				operators["+"].SetBinaryOpr(1, [](MATRIX m1, MATRIX m2){return m1 + m2; });
				operators["+"].SetUnaryLeftOpr(1, [](MATRIX m){return m; });
				operators["-"].SetBinaryOpr(1, [](MATRIX m1, MATRIX m2){return m1 - m2; });
				operators["-"].SetUnaryLeftOpr(1, [](MATRIX m){return -m; });
				operators["*"].SetBinaryOpr(2, [](MATRIX m1, MATRIX m2){return m1 * m2; });
				operators["/"].SetBinaryOpr(2, [](MATRIX m1, MATRIX m2){return m1 / m2; });

				AddFunction("sqrt", [](std::vector<MATRIX> v)
				{
					MATRIX ret = v.front();
					ret.ForEachElement([](double d){return std::sqrt(d); });

					return ret;
				}, 1);
			}

			MATRIX CalculateExpression(const std::string &expression);
			void AddVariable(const std::string &name, const MATRIX &value);
			void AddFunction(const std::string &name, std::function<MATRIX(std::vector<MATRIX>)> f,unsigned int n)
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