#include <filesystem>
#include <random>
#include <chrono>
#include <algorithm>

#include "aris.hpp"

#include "api.hpp"
#include "json.hpp"
#include "fifo_map.hpp"

namespace aris::server
{
	using Matrix = aris::core::Matrix;


	struct Calculator::Imp
	{
		struct Value
		{
			std::string type_;
			std::any value_;

			auto val()->std::any& { return std::any_cast<void*>(&value_) ? *reinterpret_cast<std::any*>(std::any_cast<void*>(value_)) : value_; }
		};
		struct Typename { };
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
			std::vector<std::tuple<std::vector<std::string>, std::string, Calculator::BuiltInFunction>> funs_;
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
				STRING,           // string
				VARIABLE,         // variable
				FUNCTION          // function
			};

			Type type;
			std::string_view word;

			double num;
			const Typename *tpn_;
			const Variable *var;
			const Function *fun;
			const Operator *opr;
		};

		using TokenVec = std::vector<Token>;
		using Iterator = TokenVec::iterator;
		static auto getWord(std::string_view &input)->std::string_view
		{
			static const std::string seperateStr("()[]{},;");
			static const std::string operatorStr("*+-/\\^|<>=");
			static const std::string spaceStr(" \t\n\r\f\v");
			static const std::string numStr("0123456789.");
			static const std::string varStr("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_0123456789");

			// triml and check if empty //
			auto next_pos = input.find_first_not_of(spaceStr);
			input = next_pos == std::string_view::npos ? std::string_view() : std::string_view(input.data() + next_pos, input.size() - next_pos);
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
			else if (input[0] == '\"')
			{
				int i;
				for (i = 1; i < input.size() && !(input[i] == '\"' && input[i - 1] != '\\'); ++i);

				auto ret = input.substr(0, i + 1);
				input = i + 1 == input.size() ? std::string_view() : input.substr(i + 1);
				return ret;
			}
			// get word //
			else
			{
				int i;
				for (i = 1; i < input.size() && varStr.find(input[i]) != std::string_view::npos; ++i);

				auto ret = input.substr(0, i);
				input = i == input.size() ? std::string_view() : input.substr(i);
				return ret;
			}
		}
		auto Expression2Tokens(std::string_view expression)const->TokenVec;
		auto CaculateTokens(Iterator begin_token, Iterator max_end_token) const->Value;

		auto CaculateValueInParentheses(Iterator &begin_token, Iterator max_end_token) const->Value;
		auto CaculateValueInBraces(Iterator &begin_token, Iterator max_end_token) const->Value;
		auto CaculateValueInFunction(Iterator &begin_token, Iterator max_end_token) const->Value;
		auto CaculateValueInOperator(Iterator &begin_token, Iterator max_end_token) const->Value;

		auto FindNextOutsideToken(Iterator left_par, Iterator end_token, Token::Type type) const->Iterator;
		auto FindNextEqualLessPrecedenceBinaryOpr(Iterator begin_token, Iterator end_token, Size precedence)const->Iterator;
		auto GetValues(Iterator begin_token, Iterator end_token)const->std::vector<std::vector<Value> >;

		std::map<std::string, Typename, std::less<>> typename_map_;
		std::map<std::string, Operator, std::less<>> operator_map_;
		std::map<std::string, Function, std::less<>> function_map_;
		std::map<std::string, Variable, std::less<>> variable_map_;
	};
	auto Calculator::Imp::Expression2Tokens(std::string_view expression)const -> TokenVec
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
				// 字符串
				if (token.word.data()[0] == '\"')
				{
					if (token.word.back() != '\"')THROW_FILE_LINE("invalid string:" + std::string(token.word));
					token.type = Token::STRING;
					break;
				}
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
	auto Calculator::Imp::CaculateTokens(Iterator b_tok, Iterator e_tok) const ->Value
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
				case Token::STRING:
					value = Value{ "String", std::string(i->word.substr(1, i->word.size() - 2)) };
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
	auto Calculator::Imp::CaculateValueInParentheses(Iterator &i, Iterator max_end_token)const->Value
	{
		auto b_par = i + 1;
		auto e_par = FindNextOutsideToken(i + 1, max_end_token, Token::PARENTHESIS_R);
		i = e_par + 1;

		return CaculateTokens(b_par, e_par);
	}
	auto Calculator::Imp::CaculateValueInBraces(Iterator &i, Iterator max_end_token)const->Value
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
					mtx.back().push_back(Matrix(std::any_cast<double>(value.val())));
				}
				else if (value.type_ == "Matrix")
				{
					mtx.back().push_back(std::any_cast<Matrix&>(value.val()));
				}
				else
				{
					THROW_FILE_LINE("invalid type when build Matrix");
				}
			}
		}
		auto m = combineMatrices(mtx);
		return Value{ "Matrix", m };
	}
	auto Calculator::Imp::CaculateValueInFunction(Iterator &i, Iterator max_end_token)const->Value
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
		auto &funs = function_map_.find(i->word)->second.funs_;
		if (auto f = std::find_if(funs.begin(), funs.end(), [&](const auto &v)->bool {return p_types.size() == std::get<0>(v).size() && std::equal(p_types.begin(), p_types.end(), std::get<0>(v).begin()); }); f == funs.end())
		{
			THROW_FILE_LINE("function \"" + std::string(i->word) + "\" not found");
		}
		else
		{
			i = e_par + 1;
			return Value{ std::get<1>(*f), std::get<2>(*f)(p_values) };
		}
	}
	auto Calculator::Imp::CaculateValueInOperator(Iterator &i, Iterator max_end_token)const->Value
	{
		auto opr = i;
		i = FindNextEqualLessPrecedenceBinaryOpr(opr + 1, max_end_token, opr->opr->priority_ul);
		auto value = CaculateTokens(opr + 1, i);
		auto func = opr->opr->ul_funs_.at(value.type_);
		return Value{ func.first, func.second(value.val()) };
	}
	auto Calculator::Imp::FindNextOutsideToken(Iterator begin_token, Iterator end_token, Token::Type type)const->Iterator
	{
		Size par_num = 0, bra_num = 0, bce_num = 0;
		auto next_token = begin_token;

		for (; next_token < end_token; ++next_token)
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
	auto Calculator::Imp::FindNextEqualLessPrecedenceBinaryOpr(Iterator begin_token, Iterator end_token, Size precedence)const->Iterator
	{
		auto next_opr = begin_token;
		for (; next_opr < end_token; ++next_opr)
		{
			next_opr = FindNextOutsideToken(next_opr, end_token, Token::OPERATOR);
			if ((next_opr == end_token) || (next_opr->opr->priority_b <= precedence))break;
		}

		return next_opr;
	}
	auto Calculator::Imp::GetValues(Iterator b_tok, Iterator e_tok)const->std::vector<std::vector<Value> >
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
	auto Calculator::calculateExpression(std::string_view expression) const->std::pair<std::string, std::any>
	{
		auto tokens = imp_->Expression2Tokens(expression);
		auto ret_val = imp_->CaculateTokens(tokens.begin(), tokens.end());
		return std::make_pair(std::move(ret_val.type_), std::move(ret_val.val()));
	}
	auto Calculator::addTypename(std::string_view tpn)->void
	{
		if (imp_->typename_map_.find(tpn) != imp_->typename_map_.end() ||
			imp_->variable_map_.find(tpn) != imp_->variable_map_.end() ||
			imp_->function_map_.find(tpn) != imp_->function_map_.end())
		{
			THROW_FILE_LINE("\"" + std::string(tpn) + "already exists, can't add this type");
		}

		imp_->typename_map_.insert(std::pair(std::string(tpn), Imp::Typename()));

		addFunction(tpn, std::vector<std::string>{ std::string(tpn) }, tpn, [](std::vector<std::any>& v)->std::any {return v[0]; });
		addBinaryOperatorFunction("=", tpn, tpn, tpn, [](std::any& p1, std::any&p2)->std::any {	return p1 = p2;	});
	}
	auto Calculator::addVariable(std::string_view var, std::string_view type, const std::any &value)->void
	{
		if (imp_->typename_map_.find(var) != imp_->typename_map_.end() ||
			imp_->variable_map_.find(var) != imp_->variable_map_.end() ||
			imp_->function_map_.find(var) != imp_->function_map_.end())
		{
			THROW_FILE_LINE("\"" + std::string(var) + "already exists, can't add this variable");
		}

		imp_->variable_map_.insert(std::pair(std::string(var), Imp::Variable{ std::string(type), std::string(var), value }));
	}
	auto Calculator::addFunction(std::string_view fun, const std::vector<std::string> &params, std::string_view ret_type, BuiltInFunction f)->void
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
	auto Calculator::addOperator(std::string_view opr, int ul_priority, int ur_priority, int b_priority)->void
	{
		if (imp_->operator_map_.find(opr) != imp_->operator_map_.end())THROW_FILE_LINE("Already has operator:" + std::string(opr));

		imp_->operator_map_[std::string(opr)].priority_ul = ul_priority;
		imp_->operator_map_[std::string(opr)].priority_ur = ur_priority;
		imp_->operator_map_[std::string(opr)].priority_b = b_priority;
	}
	auto Calculator::addUnaryLeftOperatorFunction(std::string_view opr, std::string_view p_type, std::string_view ret_type, UnaryOperatorFunction f)->void
	{
		if (auto found_opr = imp_->operator_map_.find(opr); found_opr == imp_->operator_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else if (auto found_type = imp_->typename_map_.find(p_type); found_type == imp_->typename_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(p_type));
		else if (auto found_func = found_opr->second.ul_funs_.find(p_type); found_func != found_opr->second.ul_funs_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else
		{
			found_opr->second.ul_funs_.insert(std::make_pair(std::string(p_type), std::make_pair(std::string(ret_type), f)));
		}
	}
	auto Calculator::addUnaryRightOperatorFunction(std::string_view opr, std::string_view p_type, std::string_view ret_type, UnaryOperatorFunction f)->void
	{
		if (auto found_opr = imp_->operator_map_.find(opr); found_opr == imp_->operator_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else if (auto found_type = imp_->typename_map_.find(p_type); found_type == imp_->typename_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(p_type));
		else if (auto found_func = found_opr->second.ur_funs_.find(p_type); found_func != found_opr->second.ur_funs_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else
		{
			found_opr->second.ur_funs_.insert(std::make_pair(std::string(p_type), std::make_pair(std::string(ret_type), f)));
		}
	}
	auto Calculator::addBinaryOperatorFunction(std::string_view opr, std::string_view p1_type, std::string_view p2_type, std::string_view ret_type, BinaryOperatorFunction f)->void
	{
		if (auto found_opr = imp_->operator_map_.find(opr); found_opr == imp_->operator_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else if (auto found_type1 = imp_->typename_map_.find(p1_type); found_type1 == imp_->typename_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(p1_type));
		else if (auto found_type2 = imp_->typename_map_.find(p2_type); found_type2 == imp_->typename_map_.end())THROW_FILE_LINE("failed add operator func:" + std::string(p2_type));
		else if (auto found = found_opr->second.b_funs_.find(p1_type); found != found_opr->second.b_funs_.end() && found->second.find(p2_type) != found->second.end())
			THROW_FILE_LINE("failed add operator func:" + std::string(opr));
		else
		{
			if (found == found_opr->second.b_funs_.end())
				found_opr->second.b_funs_.insert(std::make_pair(std::string(p1_type), std::map<std::string, std::pair<std::string, BinaryOperatorFunction>, std::less<>>()));

			found_opr->second.b_funs_[std::string(p1_type)][std::string(p2_type)] = std::make_pair(std::string(ret_type), f);
		}
	}
	auto Calculator::clearVariables()->void { imp_->variable_map_.clear(); }
	//auto Calculator::loadXml(const aris::core::XmlElement &xml_ele)->void 
	//{
	//	Object::loadXml(xml_ele);
	//	imp_->variable_map_.clear();
	//	
	//}
	//auto Calculator::saveXml(aris::core::XmlElement &xml_ele)const->void {}
	Calculator::~Calculator() = default;
	Calculator::Calculator(const std::string &name)
	{
		

		/*
		addOperator("<", 1, 0, 1);
		addBinaryOperatorFunction("<", "Block", "Block", "Block", ADD_MATRIX_OPR(<));
		addBinaryOperatorFunction("<", "Number", "Block", "Block", ADD_NUM_MATRIX_OPR(<));
		addBinaryOperatorFunction("<", "Block", "Number", "Block", ADD_MATRIX_NUM_OPR(<));
		addBinaryOperatorFunction("<", "Number", "Number", "Block", ADD_NUMBER_OPR(<));

		addOperator("<=", 1, 0, 1);
		addBinaryOperatorFunction("<=", "Block", "Block", "Block", ADD_MATRIX_OPR(<= ));
		addBinaryOperatorFunction("<=", "Number", "Block", "Block", ADD_NUM_MATRIX_OPR(<= ));
		addBinaryOperatorFunction("<=", "Block", "Number", "Block", ADD_MATRIX_NUM_OPR(<= ));
		addBinaryOperatorFunction("<=", "Number", "Number", "Block", ADD_NUMBER_OPR(<= ));

		addOperator(">", 1, 0, 1);
		addBinaryOperatorFunction(">", "Block", "Block", "Block", ADD_MATRIX_OPR(>));
		addBinaryOperatorFunction(">", "Number", "Block", "Block", ADD_NUM_MATRIX_OPR(>));
		addBinaryOperatorFunction(">", "Block", "Number", "Block", ADD_MATRIX_NUM_OPR(>));
		addBinaryOperatorFunction(">", "Number", "Number", "Block", ADD_NUMBER_OPR(>));

		addOperator(">=", 1, 0, 1);
		addBinaryOperatorFunction(">=", "Block", "Block", "Block", ADD_MATRIX_OPR(>= ));
		addBinaryOperatorFunction(">=", "Number", "Block", "Block", ADD_NUM_MATRIX_OPR(>= ));
		addBinaryOperatorFunction(">=", "Block", "Number", "Block", ADD_MATRIX_NUM_OPR(>= ));
		addBinaryOperatorFunction(">=", "Number", "Number", "Block", ADD_NUMBER_OPR(>= ));

		addOperator("==", 1, 0, 1);
		addBinaryOperatorFunction("==", "Block", "Block", "Block", ADD_MATRIX_OPR(== ));
		addBinaryOperatorFunction("==", "Number", "Block", "Block", ADD_NUM_MATRIX_OPR(== ));
		addBinaryOperatorFunction("==", "Block", "Number", "Block", ADD_MATRIX_NUM_OPR(== ));
		addBinaryOperatorFunction("==", "Number", "Number", "Number", ADD_NUMBER_OPR(== ));
		*/
		addVariable("pi", "Number", double(3.141592653));
		addFunction("sin", std::vector<std::string>{"Number"}, "Number", [](std::vector<std::any> &params)->std::any
		{
			return std::sin(std::any_cast<double>(params[0]));
		});
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Calculator);
	
	
	
	
	
	



	
		
	
	
	
	auto generate_random_id(int length)->std::string 
	{
		const char letters[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ:;.,?!-_+=-*";

		std::string ret;
		ret.reserve(length);
		for (int i = 0; i < length; ++i)ret.push_back(letters[std::rand() % 64]);
		return ret;
	};

	std::filesystem::path path_;
	auto rootPath()-> std::filesystem::path { return path_; };
	auto setRootPath(std::filesystem::path path)->void { path_ = path; }


	template<class K, class V, class dummy_compare, class A>
	using my_workaround_fifo_map = nlohmann::fifo_map<K, V, nlohmann::fifo_map_compare<K>, A>;
	using my_json = nlohmann::basic_json<my_workaround_fifo_map>;

	auto fetchInterfaceConfig()->std::string
	{
		aris::core::XmlDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		my_json js;
		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard"))
			{
				my_json js1;
				js1["name"] = std::string(ele->Attribute("name"));
				js1["editable"] = std::string(ele->Attribute("editable")) == "true";
				js1["i"] = std::string(ele->Attribute("id"));
				js1["cells"] = std::vector<std::string>();
				for (auto e1 = ele->FirstChildElement(); e1; e1 = e1->NextSiblingElement())
				{
					my_json j2;//{"name":"Ethercat配置","type":"EthercatConfiguration","i":"EMlxGXxpwDGgz","w":48,"h":23,"x":0,"y":0,"options":"{}"}
					j2["name"] = e1->Attribute("name");
					j2["type"] = e1->Attribute("type");
					j2["i"] = e1->Attribute("id");
					j2["w"] = e1->IntAttribute("width");
					j2["h"] = e1->IntAttribute("height");
					j2["x"] = e1->IntAttribute("x");
					j2["y"] = e1->IntAttribute("y");
					j2["options"] = e1->Attribute("options");
					js1["cells"].push_back(j2);
				}
				js["dashboards"].push_back(js1);
			}
			else if (ele->Name() == std::string("WebSocket"))
			{
				js["ws"]["url"] = ele->Attribute("url");
				js["ws"]["commandSendInterval"] = ele->IntAttribute("commandSendInterval");
				js["ws"]["commandSendDelay"] = ele->IntAttribute("commandSendDelay");
				js["ws"]["getInterval"] = ele->IntAttribute("getInterval");
				js["ws"]["unityUpdateInterval"] = ele->IntAttribute("unityUpdateInterval");
			}
			else if (ele->Name() == std::string("LayoutConfig"))
			{
				js["layoutConfig"]["cols"] = ele->IntAttribute("cols");
				js["layoutConfig"]["rowHeight"] = ele->IntAttribute("rowHeight");
				js["layoutConfig"]["margin"] = ele->IntAttribute("margin");
				js["layoutConfig"]["containerPadding"] = ele->IntAttribute("containerPadding");
				js["layoutConfig"]["theme"] = ele->Attribute("theme");
			}
		}
		
		return js.dump(-1);
	}
	auto updateDashboard(std::string dash_id, std::string js_str)->std::string 
	{
		aris::core::XmlDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());
		
		aris::core::XmlElement *dash_ele = nullptr;

		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard") && ele->Attribute("id") == dash_id) 
			{
				dash_ele = ele;
				break;
			}
		}

		if (dash_ele == nullptr)return "";

		auto js = my_json::parse(js_str);


		dash_ele->SetAttribute("name", js["name"].get<std::string>().c_str());
		dash_ele->SetAttribute("editable", js["editable"].get<bool>());
		dash_ele->SetAttribute("id", js["i"].get<std::string>().c_str());

		dash_ele->DeleteChildren();
		for (auto js2 : js["cells"])
		{
			auto new_cell = doc.NewElement("Cell");

			new_cell->SetAttribute("name",    js2["name"].get<std::string>().c_str());
			new_cell->SetAttribute("type",    js2["type"].get<std::string>().c_str());
			new_cell->SetAttribute("id",      js2["i"].get<std::string>().c_str());
			new_cell->SetAttribute("width",   js2["w"].get<int>());
			new_cell->SetAttribute("height",  js2["h"].get<int>());
			new_cell->SetAttribute("x",       js2["x"].get<int>());
			new_cell->SetAttribute("y",       js2["y"].get<int>());
			new_cell->SetAttribute("options", js2["options"].get<std::string>().c_str());

			dash_ele->InsertEndChild(new_cell);
		}

		doc.SaveFile((rootPath() / "../robot/interface.xml").string().c_str());

		return js_str;
	}
	auto createCell(std::string dash_id, std::string cell)->std::string
	{
		aris::core::XmlDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		aris::core::XmlElement *dash_ele = nullptr;

		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard") && ele->Attribute("id") == dash_id)
			{
				dash_ele = ele;
				break;
			}
		}

		if (dash_ele == nullptr)return "";


		auto cell_js = my_json::parse(cell);
		auto new_cell = doc.NewElement("Cell");
		auto id = generate_random_id(10);

		new_cell->SetAttribute("name", cell_js["name"].get<std::string>().c_str());
		new_cell->SetAttribute("type", cell_js["type"].get<std::string>().c_str());
		new_cell->SetAttribute("width", cell_js["w"].get<int>());
		new_cell->SetAttribute("height", cell_js["h"].get<int>());
		new_cell->SetAttribute("id", id.c_str());
		new_cell->SetAttribute("x", 0);
		new_cell->SetAttribute("y", 0);
		new_cell->SetAttribute("options", "");

		cell_js["i"] = id;
		cell_js["x"] = 0;
		cell_js["y"] = 0;
		cell_js["options"] = "{}";

		doc.SaveFile((rootPath() / "../robot/interface.xml").string().c_str());

		return cell_js.dump(-1);
	}
	auto deleteCell(std::string dash_id, std::string cell_id)->std::string
	{
		aris::core::XmlDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		aris::core::XmlElement *dash_ele = nullptr;

		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard") && ele->Attribute("id") == dash_id)
			{
				dash_ele = ele;
				break;
			}
		}
		if (dash_ele == nullptr)return "";

		aris::core::XmlElement *cell_ele = nullptr;
		for (auto ele = dash_ele->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			std::cout << ele->Name() << " " << ele->Attribute("id") << "  " << cell_id << std::endl;
			
			if (ele->Name() == std::string("Cell") && ele->Attribute("id") == cell_id)
			{
				cell_ele = ele;
				break;
			}
		}
		if (cell_ele == nullptr)return "";
		

		my_json j2;
		j2["name"] = cell_ele->Attribute("name");
		j2["type"] = cell_ele->Attribute("type");
		j2["i"] = cell_ele->Attribute("id");
		j2["w"] = cell_ele->IntAttribute("width");
		j2["h"] = cell_ele->IntAttribute("height");
		j2["x"] = cell_ele->IntAttribute("x");
		j2["y"] = cell_ele->IntAttribute("y");
		j2["options"] = cell_ele->Attribute("options");
		
		if (cell_ele)dash_ele->DeleteChild(cell_ele);
		doc.SaveFile((rootPath() / "../robot/interface.xml").string().c_str());

		return j2.dump(-1);
	}

	template <typename TP>
	std::time_t to_time_t(TP tp)
	{
		using namespace std::chrono;
		auto sctp = time_point_cast<system_clock::duration>(tp - TP::clock::now()
			+ system_clock::now());
		return system_clock::to_time_t(sctp);
	}

	auto fetchPrograms()->std::string
	{
		my_json js;

		std::filesystem::path program_path(rootPath());
		program_path = program_path / "../robot/program";

		for (auto&dir : std::filesystem::directory_iterator(program_path))
		{
			if (std::filesystem::is_directory(dir))
			{
				my_json pro_dir_js;
				pro_dir_js["name"] = dir.path().filename().string();
				pro_dir_js["path"] = "/program/" + dir.path().filename().string();
				
				auto ftime = dir.last_write_time();
				std::time_t tt = to_time_t(ftime);
				std::tm *gmt = std::gmtime(&tt);
				std::stringstream buffer;
				buffer << std::put_time(gmt, "%A, %d %B %Y %H:%M");
				
				pro_dir_js["modTime"] = buffer.str();
				pro_dir_js["isDir"] = true;
				
				std::filesystem::path dat, pro;
				for (auto&file : std::filesystem::directory_iterator(dir))
				{
					if (file.is_regular_file())
					{
						if (file.path().extension() == ".dat")
						{
							dat = file.path();
						}
						else if (file.path().extension() == ".pro")
						{
							pro = file.path();
						}
					}
				}

				my_json pro_file_js;
				pro_file_js["name"] = pro.filename().string();
				pro_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + pro.filename().string();
				std::ifstream f(pro);
				std::string str((std::istreambuf_iterator<char>(f)),std::istreambuf_iterator<char>());
				pro_file_js["content"] = str;
				pro_dir_js["files"].push_back(pro_file_js);

				
				my_json dat_file_js;
				dat_file_js["name"] = dat.filename().string();
				dat_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + dat.filename().string();
				f.close();
				f.open(dat);
				str = std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
				dat_file_js["content"] = str;
				pro_dir_js["files"].push_back(dat_file_js);
				
				js[dir.path().filename().string()] = pro_dir_js;
			}
		}


		auto ret = js.dump(-1);
		int index = 0;
		while (true) {
			index = ret.find("<", index);
			if (index == std::string::npos) break;
			ret.replace(index, 1, "\\u003c");
			index += 5;
		}
		index = 0;
		while (true) {
			index = ret.find(">", index);
			if (index == std::string::npos) break;
			ret.replace(index, 1, "\\u003e");
			index += 5;
		}
		
		return ret;

	}
	auto createProgram(std::string pro_js_str)->std::string 
	{
		auto js = my_json::parse(pro_js_str);
		auto pro_name = js["name"].get<std::string>();
		auto program_path = rootPath() / "../robot/program" / pro_name;

		if (std::filesystem::exists(program_path))return "";

		std::filesystem::create_directories(program_path);

		std::filesystem::path dat = program_path / (pro_name + ".dat"), pro = program_path / (pro_name + ".pro");


		std::fstream f(pro, std::ios::out | std::ios::trunc);
		f << "<xml xmlns=\"https://developers.google.com/blockly/xml\"></xml>";
		f.close();

		f.open(dat, std::ios::out | std::ios::trunc);
		f << "<xml xmlns=\"https://developers.google.com/blockly/xml\"></xml>";
		f.close();

		auto dir = std::filesystem::directory_entry(program_path);


		my_json pro_dir_js;
		pro_dir_js["name"] = dir.path().filename().string();
		pro_dir_js["path"] = "/program/" + dir.path().filename().string();

		auto ftime = dir.last_write_time();
		std::time_t tt = to_time_t(ftime);
		std::tm *gmt = std::gmtime(&tt);
		std::stringstream buffer;
		buffer << std::put_time(gmt, "%A, %d %B %Y %H:%M");
		buffer.str();

		pro_dir_js["modTime"] = buffer.str();
		pro_dir_js["isDir"] = true;

		my_json pro_file_js;
		pro_file_js["name"] = pro.filename().string();
		pro_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + pro.filename().string();
		pro_file_js["content"] = "<xml xmlns=\"https://developers.google.com/blockly/xml\"></xml>";
		pro_dir_js["files"].push_back(pro_file_js);

		my_json dat_file_js;
		dat_file_js["name"] = dat.filename().string();
		dat_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + dat.filename().string();
		dat_file_js["content"] = "<xml xmlns=\"https://developers.google.com/blockly/xml\"></xml>";
		pro_dir_js["files"].push_back(dat_file_js);

		auto ret = pro_dir_js.dump(-1);
		int index = 0;
		while (true) {
			index = ret.find("<", index);
			if (index == std::string::npos) break;
			ret.replace(index, 1, "\\u003c");
			index += 5;
		}
		index = 0;
		while (true) {
			index = ret.find(">", index);
			if (index == std::string::npos) break;
			ret.replace(index, 1, "\\u003e");
			index += 5;
		}

		return ret;

	}
	auto updateProgram(std::string pro_name, std::string data)->std::string
	{
		auto program_path = rootPath() / "../robot/program";
		auto js = my_json::parse(data);

		std::fstream f(program_path /pro_name/ (pro_name + ".pro"), std::ios::out | std::ios::trunc);
		f << js["files"][0]["content"].get<std::string>();
		f.close();

		f.open(program_path / pro_name / (pro_name + ".dat"), std::ios::out | std::ios::trunc);
		f << js["files"][1]["content"].get<std::string>();
		f.close();

		return data;
	}
	auto deleteProgram(std::string pro_name)->std::string
	{
		auto program_path = rootPath() / "../robot/program" / pro_name;
		if (!std::filesystem::exists(program_path))return "";
		
		std::filesystem::remove_all(program_path);
		
		my_json js;

		js["name"] = pro_name;
		return js.dump(-1);
	}
	auto renameProgram(std::string old_name, std::string new_name_js)->std::string 
	{
		if (!std::filesystem::exists(rootPath() / "../robot/program" / old_name))return "";

		auto js = my_json::parse(new_name_js);

		auto new_name = js["name"].get<std::string>();

		std::filesystem::rename(rootPath() / "../robot/program" / old_name, rootPath() / "../robot/program" / new_name);
		std::filesystem::rename(rootPath() / "../robot/program" / new_name / (old_name + ".pro"), rootPath() / "../robot/program" / new_name / (new_name + ".pro"));
		std::filesystem::rename(rootPath() / "../robot/program" / new_name / (old_name + ".dat"), rootPath() / "../robot/program" / new_name / (new_name + ".dat"));

		///////////////////////以下返回///////////////////////////////////
		auto dir = std::filesystem::directory_entry(rootPath() / "../robot/program" / new_name);

		std::filesystem::path dat = rootPath() / "../robot/program" / new_name / (new_name + ".dat")
			, pro = rootPath() / "../robot/program" / new_name / (new_name + ".pro");

		my_json pro_dir_js;
		pro_dir_js["name"] = dir.path().filename().string();
		pro_dir_js["path"] = "/program/" + dir.path().filename().string();

		auto ftime = dir.last_write_time();
		std::time_t tt = to_time_t(ftime);
		std::tm *gmt = std::gmtime(&tt);
		std::stringstream buffer;
		buffer << std::put_time(gmt, "%A, %d %B %Y %H:%M");
		buffer.str();

		pro_dir_js["modTime"] = buffer.str();
		pro_dir_js["isDir"] = true;

		my_json pro_file_js;
		pro_file_js["name"] = pro.filename().string();
		pro_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + pro.filename().string();
		std::ifstream f(pro);
		std::string str((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
		pro_file_js["content"] = str;
		pro_dir_js["files"].push_back(pro_file_js);

		my_json dat_file_js;
		dat_file_js["name"] = dat.filename().string();
		dat_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + dat.filename().string();
		f.close();
		f.open(dat);
		str = std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
		dat_file_js["content"] = str;
		pro_dir_js["files"].push_back(dat_file_js);

		return pro_dir_js.dump(-1);
	}

	auto fetchESIPath()->std::string;

	auto fetchConfigXml()->std::string;
	auto loadConfigModel()->std::string;

	auto fetchRobotModels()->std::string;
	auto deleteRobotModelPart()->std::string;
	auto deleteRobotModel()->std::string;
	auto createRobotModel()->std::string;

	auto fetchLogName()->std::string;
	auto fetchLogContent()->std::string;
}



