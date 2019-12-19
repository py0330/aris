#include <algorithm>

#include"aris/plan/function.hpp"
#include <aris/control/control.hpp>
#include <aris/dynamic/dynamic.hpp>

#include <sstream>

namespace aris::plan
{
	auto moveAbsolute(double i, double begin_pos, double end_pos, double vel, double acc, double dec, double &current_pos, double &current_vel, double &current_acc, Size& total_count)->void
	{
		auto v = std::abs(vel);
		auto a = std::abs(acc);
		auto d = std::abs(dec);
		auto s = std::abs(end_pos - begin_pos);

		Size n1 = static_cast<Size>(std::ceil(v / a));
		Size n3 = static_cast<Size>(std::ceil(v / d));

		a = v / n1;
		d = v / n3;

		double s1 = a * n1 * n1 / 2.0;
		double s3 = d * n3 * n3 / 2.0;
		double s2 = s - s1 - s3;

		// 判断是否有匀速段
		if (s2 > 0)
		{
			Size n2 = static_cast<Size>(std::ceil(s2 / v));
			double coe = s / (s1 + v * n2 + s3);

			a *= coe;
			d *= coe;
			v *= coe;
			s1 *= coe;
			s2 *= coe;
			s3 *= coe;

			total_count = n1 + n2 + n3;
			if (i < n1)
			{
				current_pos = a * i * i / 2.0;
				current_vel = a * i;
				current_acc = a;
			}
			else if (i < n1 + n2)
			{
				current_pos = s1 + v * (i - n1);
				current_vel = v;
				current_acc = 0.0;
			}
			else if (i < n1 + n2 + n3)
			{
				current_pos = s - d * (n1 + n2 + n3 - i) * (n1 + n2 + n3 - i) / 2.0;
				current_vel = d * (n1 + n2 + n3 - i);
				current_acc = -d;
			}
			else
			{
				current_pos = s;
				current_vel = 0;
				current_acc = 0;
			}
		}
		else
		{
			v = std::sqrt(2 * s * a * d / (a + d));
			n1 = static_cast<Size>(std::ceil(v / a));
			n3 = static_cast<Size>(std::ceil(v / d));

			double coe = s / (a*n1*n1 / 2.0 + d * n3*n3 / 2.0);
			a *= coe;
			d *= coe;

			total_count = n1 + n3;
			if (i < n1)
			{
				current_pos = a * i * i / 2.0;
				current_vel = a * i;
				current_acc = a;
			}
			else if (i < n1 + n3)
			{
				current_pos = s - d * (n1 + n3 - i) * (n1 + n3 - i) / 2.0;
				current_vel = d * (n1 + n3 - i);
				current_acc = -d;
			}
			else
			{
				current_pos = s;
				current_vel = 0;
				current_acc = 0;
			}

		}

		// 修正位置、速度、加速度方向
		if (end_pos < begin_pos)
		{
			current_pos = begin_pos - current_pos;
			current_vel = -current_vel;
			current_acc = -current_acc;
		}
		else
		{
			current_pos = begin_pos + current_pos;
			current_vel = current_vel;
			current_acc = current_acc;
		}
	}
	auto moveAbsolute2(double pa, double va, double aa, double pt, double vt, double at, double vm, double am, double dm, double dt, double zero_check, double &pc, double &vc, double &ac, Size& total_count)->int
	{
		vt = 0.0;
		at = 0.0;

		vm = std::abs(vm);
		am = std::abs(am);
		dm = std::abs(dm);

		// 当前速度超过速度上限 //
		if (std::abs(va) > vm + dm * dt)
		{
			ac = -aris::dynamic::s_sgn2(va) * dm;
			goto return_flag;
		}

		// 查看当前速度是否过快 //
		{
			// 计算完全减速所产生的位移，vdec和sdec有方向 //
			const double vdec = va;
			const double ndec = std::abs(std::ceil(std::abs(vdec) / dm / dt - zero_check));
			const double tdec = ndec * dt;
			const double sdec = vdec * tdec / 2;

			// 当前速度需要完全减速到零，之后可能还需要反转 //
			if ((va > zero_check && (pt - pa) < sdec + zero_check) || (va < -zero_check && (pt - pa) > sdec - zero_check))
			{
				ac = -va / ndec / dt;
				goto return_flag;
			}
		}

		// 二分法算当前可以最快的加速度，并沿着该加速度加速 //
		{
			double lower_bound = pt - pa < 0.0 ? std::min(dm, -va / dt) : std::max(-dm, -va / dt);
			double upper_bound = pt - pa < 0.0 ? std::max(-am, (-vm - va) / dt) : std::min(am, (vm - va) / dt);

			while (std::abs(lower_bound - upper_bound) > zero_check)
			{
				const double a1 = (lower_bound + upper_bound) / 2.0;
				const double v1 = va + a1 * dt;
				const double p1 = pa + 0.5*(va + v1)*dt;
				const double ndec1 = std::abs(std::ceil(std::abs(v1) / dm / dt));
				const double tdec1 = ndec1 * dt;
				const double sdec1 = v1 * tdec1 / 2.0;

				if ((pt - pa >= 0 && (pt - p1) >= sdec1) || (pt - pa < 0 && (pt - p1) <= sdec1))
				{
					lower_bound = a1;
				}
				else
				{
					upper_bound = a1;
				}
			}
			ac = lower_bound;
		}

	return_flag:
		vc = va + ac * dt;
		pc = pa + 0.5 * (va + vc) * dt;
		total_count = 1;
		return std::abs(pt - pc) < zero_check && std::abs(vt - vc) < zero_check ? 0 : 1;
	}


	struct LanguageParser::Imp
	{
		struct CmdInfo
		{
			std::string cmd;
			int next_cmd_true_, next_cmd_false_;
		};
		
		// 关键词 //
		static inline const char *MAIN        = "main";
		static inline const char *ENDMAIN     = "endmain";
		static inline const char *FUNCTION    = "function";
		static inline const char *ENDFUNCTION = "endfunction";
		static inline const char *VAR         = "var";
		static inline const char *IF          = "if";
		static inline const char *ELSE        = "else";
		static inline const char *ELSEIF      = "elseif";
		static inline const char *ENDIF       = "endif";
		static inline const char *WHILE       = "while";
		static inline const char *ENDWHILE    = "endwhile";
		
		auto parseEnvironment(std::map<int, CmdInfo>::iterator b, std::map<int, CmdInfo>::iterator e)->std::map<int, CmdInfo>::iterator
		{
			main_id_ = 0;
			
			for (auto i = b; i != e; ++i)
			{
				auto id = i->first;
				auto &info = i->second;
				auto cmd_name = info.cmd.substr(0, info.cmd.find_first_of(" \t\n\r\f\v("));

				if (auto l = { ENDMAIN, ENDWHILE, IF, ELSEIF, ELSE, ENDIF, WHILE, ENDWHILE };
					std::any_of(l.begin(), l.end(), [&cmd_name](const char *c) {return c == cmd_name; }))
				{
					auto err = "invalid " + info.cmd + " in line:" + std::to_string(id);
					THROW_FILE_LINE(err);
				}
				else if (cmd_name == VAR)
				{
					var_pool_.push_back(info.cmd);
				}
				else if (cmd_name == MAIN)
				{
					main_id_ = i->first;
					current_id_ = i->first;
					if (std::next(i) != e)i->second.next_cmd_true_ = std::next(i)->first;
					i = parseMain(std::next(i), e);
				}
				else if (cmd_name == FUNCTION)
				{
					if (info.cmd.find_first_of(" \t\n\r\f\v(") == std::string::npos)	THROW_FILE_LINE("function does not have name in line: " + std::to_string(i->first));
					auto func = info.cmd.substr(info.cmd.find_first_of(" \t\n\r\f\v("), std::string::npos);
					func.erase(0, func.find_first_not_of(" \t\n\r\f\v"));
					auto funcname = func.substr(0, func.find_first_of(" \t\n\r\f\v("));
					if (functions_.find(funcname) != functions_.end())
					{
						THROW_FILE_LINE("function already exist in line " + std::to_string(i->first));
					}
					functions_.insert(std::make_pair(funcname, i->first));
					
					if (std::next(i) != e)i->second.next_cmd_true_ = std::next(i)->first;
					i = parseFunction(std::next(i), e);
				}
				else 
				{
					auto err = "invalid " + info.cmd + " in line:" + std::to_string(id);
					THROW_FILE_LINE(err);
				}
			}

			if (main_id_ == 0)THROW_FILE_LINE("program must has main");

			return e;
		}
		auto parseMain(std::map<int, CmdInfo>::iterator b, std::map<int, CmdInfo>::iterator e)->std::map<int, CmdInfo>::iterator
		{
			for (auto i = b; i != e; ++i)
			{
				auto id = i->first;
				auto &info = i->second;
				auto cmd_name = info.cmd.substr(0, info.cmd.find_first_of(" \t\n\r\f\v("));

				if (cmd_name == ENDMAIN)
				{
					auto ret = parseCode(b, i);
					std::prev(i)->second.next_cmd_true_ = i->first;
					return ret;
				}
			}

			std::string err = "no endfuncion for function in line " + std::to_string(b->first);
			THROW_FILE_LINE(err);

			return e;
		}
		auto parseFunction(std::map<int, CmdInfo>::iterator b, std::map<int, CmdInfo>::iterator e)->std::map<int, CmdInfo>::iterator
		{
			for (auto i = b; i != e; ++i)
			{
				auto id = i->first;
				auto &info = i->second;
				auto cmd_name = info.cmd.substr(0, info.cmd.find_first_of(" \t\n\r\f\v("));

				if (cmd_name == ENDFUNCTION)
				{
					auto ret = parseCode(b, i);
					std::prev(i)->second.next_cmd_true_ = i->first;
					return ret;
				}
			}

			std::string err = "no endfuncion for function in line " + std::to_string(b->first);
			THROW_FILE_LINE(err);

			return e;
		}
		auto parseCode(std::map<int, CmdInfo>::iterator b, std::map<int, CmdInfo>::iterator e)->std::map<int, CmdInfo>::iterator
		{
			for (auto i = b; i != e; ++i)
			{
				auto id = i->first;
				auto &info = i->second;
				auto cmd_name = info.cmd.substr(0, info.cmd.find_first_of(" \t\n\r\f\v("));

				if (auto l = { MAIN, ENDMAIN, FUNCTION, ENDFUNCTION, VAR, ELSEIF, ELSE, ENDIF, ENDWHILE };
					std::any_of(l.begin(), l.end(), [&cmd_name](const char *c) {return c == cmd_name; }))
				{
					auto err = "invalid " + info.cmd + " in line:" + std::to_string(id);
					THROW_FILE_LINE(err);
				}
				else if (cmd_name == IF)
				{
					if (std::next(i) != e)i->second.next_cmd_true_ = std::next(i)->first;
					i = parseIf(i, e);
				}
				else if (cmd_name == WHILE)
				{
					if (std::next(i) != e)i->second.next_cmd_true_ = std::next(i)->first;
					i = parseWhile(i, e);
				}
				else
				{
					info.next_cmd_true_ = std::next(i) == e ? 0 : std::next(i)->first;
				}
			}

			return e;
		}
		auto parseIf(std::map<int, CmdInfo>::iterator b, std::map<int, CmdInfo>::iterator e)->std::map<int, CmdInfo>::iterator
		{
			std::list<std::map<int, CmdInfo>::iterator> prev_else_line;
			std::map<int, CmdInfo>::iterator last_if_begin = b;

			int is_end = 1;
			bool has_else = false;
			for (auto i = std::next(b); i != e; ++i)
			{
				auto id = i->first;
				auto &info = i->second;
				auto cmd_name = info.cmd.substr(0, info.cmd.find_first_of(" \t\n\r\f\v("));

				if (cmd_name == IF)
				{
					is_end++;
				}
				else if (cmd_name == ELSEIF && is_end == 1)
				{
					if (has_else)
					{
						std::string err = "Find elseif after else in line " + std::to_string(i->first);
						THROW_FILE_LINE(err);
					}

					parseCode(std::next(last_if_begin), i);
					last_if_begin->second.next_cmd_false_ = i->first;
					last_if_begin = i;
					i->second.next_cmd_true_ = std::next(i)->first;

					prev_else_line.push_back(std::prev(i));
				}
				else if (cmd_name == ELSE && is_end == 1)
				{
					if (has_else)
					{
						std::string err = "Find second else in line " + std::to_string(i->first);
						THROW_FILE_LINE(err);
					}
					has_else = true;

					parseCode(std::next(last_if_begin), i);
					last_if_begin->second.next_cmd_false_ = i->first;
					last_if_begin = i;
					i->second.next_cmd_true_ = std::next(i)->first;

					prev_else_line.push_back(std::prev(i));
				}
				else if (cmd_name == ENDIF)
				{
					is_end--;

					if (is_end == 0)
					{
						parseCode(std::next(last_if_begin), i);
						auto prev_cmd_name = std::prev(i)->second.cmd.substr(0, std::prev(i)->second.cmd.find_first_of(" \t\n\r\f\v("));
						if (prev_cmd_name != "if") std::prev(i)->second.next_cmd_true_ = i->first;
						i->second.next_cmd_true_ = std::next(i) == e ? 0 : std::next(i)->first;
						auto last_if_begin_cmd_name = last_if_begin->second.cmd.substr(0, last_if_begin->second.cmd.find_first_of(" \t\n\r\f\v("));
						if (last_if_begin_cmd_name != "else")last_if_begin->second.next_cmd_false_ = i->first;

						for (auto j : prev_else_line)
						{
							j->second.next_cmd_true_ = i->first;
						}

						return i;
					}
				}
			}

			std::string err = "no endif for if in line " + std::to_string(b->first);
			THROW_FILE_LINE(err);

			return b;
		}
		auto parseWhile(std::map<int, CmdInfo>::iterator b, std::map<int, CmdInfo>::iterator e)->std::map<int, CmdInfo>::iterator
		{
			int is_end = 1;
			for (auto i = std::next(b); i != e; ++i)
			{
				auto id = i->first;
				auto &info = i->second;
				auto cmd_name = info.cmd.substr(0, info.cmd.find_first_of(" \t\n\r\f\v("));

				if (cmd_name == WHILE)
				{
					is_end++;
				}
				else if (cmd_name == ENDWHILE)
				{
					is_end--;

					if (is_end == 0)
					{
						parseCode(std::next(b), i);
						std::prev(i)->second.next_cmd_true_ = b->first;
						i->second.next_cmd_true_ = std::next(i) == e ? 0 : std::next(i)->first;
						b->second.next_cmd_false_ = i->first;

						return i;
					}
				}
			}

			return b;
		}

		int main_id_{ 0 }, current_id_{ 0 };
		std::map<int, CmdInfo> cmd_map_;
		std::map<std::string, int> functions_;
		std::vector<std::string> var_pool_;
		std::list<int> function_ret_stack_;
	};
	auto LanguageParser::parseLanguage()->void
	{
		imp_->parseEnvironment(imp_->cmd_map_.begin(), imp_->cmd_map_.end());

		//for (auto &cmd : imp_->cmd_map_)
		//{
		//	std::cout << std::setw(4) << cmd.first << " : " << std::setw(15) << cmd.second.cmd << " | " << std::setw(5) << cmd.second.next_cmd_true_ << " | " << cmd.second.next_cmd_false_ << std::endl;
		//}
		//for (auto &cmd : imp_->functions_)
		//{
		//	std::cout << cmd.first << std::endl;
		//}
	}
	auto LanguageParser::setProgram(const std::string& program)->void
	{
		imp_->cmd_map_.clear();
		imp_->functions_.clear();
		imp_->function_ret_stack_.clear();
		imp_->var_pool_.clear();

		std::stringstream ss(program);
		int id;
		while (ss >> id)
		{
			char c;
			while(ss >> c, c!=':');
			std::string cmd;
			std::getline(ss, cmd);
			cmd.erase(0, cmd.find_first_not_of(" \t\n\r\f\v"));// trim l
			cmd.erase(cmd.find_last_not_of(" \t\n\r\f\v") + 1);// trim r
			if (cmd != "")imp_->cmd_map_[id] = Imp::CmdInfo{ cmd, 0, 0 };
		}
	}
	auto LanguageParser::varPool()->const std::vector<std::string>& { return imp_->var_pool_; }
	auto LanguageParser::gotoMain()->void { imp_->current_id_ = imp_->main_id_; }
	auto LanguageParser::gotoLine(int line)->void 
	{ 
		if (imp_->cmd_map_.find(line) == imp_->cmd_map_.end())THROW_FILE_LINE("This line does not exist");
		imp_->current_id_ = line;
	}
	auto LanguageParser::forward(bool is_this_cmd_successful)->void
	{
		auto cmd_str = currentCmd().substr(0, currentCmd().find_first_of(" \t\n\r\f\v("));
		
		if (auto found = imp_->functions_.find(cmd_str); found != imp_->functions_.end())
		{
			imp_->function_ret_stack_.push_back(imp_->cmd_map_[imp_->current_id_].next_cmd_true_);
			imp_->current_id_ = found->second;
		}
		else if (cmd_str == "endfunction")
		{
			imp_->current_id_ = imp_->function_ret_stack_.back();
			imp_->function_ret_stack_.pop_back();
		}
		else
		{
			imp_->current_id_ = is_this_cmd_successful ? imp_->cmd_map_[imp_->current_id_].next_cmd_true_ : imp_->cmd_map_[imp_->current_id_].next_cmd_false_;
		}
	}
	auto LanguageParser::currentLine()const->int { return imp_->current_id_; }
	auto LanguageParser::currentCmd()const->const std::string& { return imp_->cmd_map_.at(imp_->current_id_).cmd; }
	auto LanguageParser::isCurrentLineKeyWord()const->bool
	{
		auto cmd_name = currentCmd().substr(0, currentCmd().find_first_of(" \t\n\r\f\v("));
		auto l = { Imp::MAIN, Imp::ENDMAIN, Imp::FUNCTION, Imp::ENDFUNCTION	, Imp::VAR, Imp::IF, Imp::ELSE, Imp::ELSEIF, Imp::ENDIF, Imp::WHILE, Imp::ENDWHILE, };
		return std::any_of(l.begin(), l.end(), [&cmd_name](const char *c) {return c == cmd_name; });
	}
	auto LanguageParser::isCurrentLineFunction()const->bool
	{
		auto cmd_name = currentCmd().substr(0, currentCmd().find_first_of(" \t\n\r\f\v("));
		return imp_->functions_.find(cmd_name) != imp_->functions_.end();
	}
	auto LanguageParser::isEnd()const->bool 
	{
		auto found = imp_->cmd_map_.find(imp_->current_id_);
		return (found == imp_->cmd_map_.end()) || (found->second.cmd == "endmain") ? true : false;
	}
	LanguageParser::~LanguageParser() = default;
	LanguageParser::LanguageParser(const std::string &name) :Object(name), imp_(new Imp){}
	ARIS_DEFINE_BIG_FOUR_CPP(LanguageParser);
}
