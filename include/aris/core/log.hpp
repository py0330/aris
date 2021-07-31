#ifndef ARIS_CORE_LOG_H_
#define ARIS_CORE_LOG_H_

#include <string>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <exception>
#include <thread>
#include <set>
#include <mutex>
#include <map>

#include "sqlite3.h"
#include "aris/core/data_structure.hpp"

#include <aris_lib_export.h>

#define LOG_AND_THROW(error) aris::core::log(aris::core::LogLvl::kError, 0, error), throw std::runtime_error(error)

#define THROW_FILE_LINE(error) throw std::runtime_error(std::string(__FILE__) + "_" + std::to_string(__LINE__)+ ":" + error)

#define ARIS_COUT aris::core::cout()

namespace aris::core{
	
#define LANG_ENGLISH 0
#define LANG_CHINESE 1
	// 设置语言 id //
	auto ARIS_API setLanguage(int language_id)->void;
	// 当前语言 id //
	auto ARIS_API currentLanguage()->int;
	// 根据语言切换 //
	//auto ARIS_API locale(std::initializer_list<const char*> format_msg)->std::string {
	//	return (currentLanguage() <= format_msg.size()) ? format_msg.begin()[currentLanguage()] : format_msg.begin()[0];

	//}

	// 日志等级 //
	enum class ARIS_API LogLvl : int {
		kDebug = 0,
		kInfo,
		kWarning,
		kError
	};
	// 设置记录log的方法 //
	auto ARIS_API setLogMethod(std::function<void(LogLvl level, int code, const char *msg)> method = nullptr)->void;
	// 调用记录log的方法，由于涉及语言选择，尽量不要直接使用该方法 //
	auto ARIS_API logMethod(LogLvl level, int code, const char *msg)->void;

	// 记录log //
	template <typename ...Args>
	auto log(LogLvl level, int code, std::initializer_list<const char*> format_msg, Args ... args)->void {
		auto format_str = (currentLanguage() <= format_msg.size()) ? format_msg.begin()[currentLanguage()] : format_msg.begin()[0];

		// 不定长的format //
		int size_s = std::snprintf(nullptr, 0, format_str, args ...) + 1;;
		if (size_s <= 0) { throw std::runtime_error("Error during formatting."); }
		auto size = static_cast<size_t>(size_s);
		auto buf = std::make_unique<char[]>(size);
		std::snprintf(buf.get(), size, format_str, args ...);
		logMethod(level, code, buf.get());
	}

	

	// 以下为默认log方法 //

	// 默认log方法 //
	auto ARIS_API defaultLog(LogLvl level, int code, const char *msg)->void;
	// 设置log文件夹，参数为空时将当前二进制文件路径设为log路径 //
	auto ARIS_API setDefaultLogDirectory(const std::filesystem::path &log_dir_path = std::filesystem::path())->void;
	// 获取log文件夹 //
	auto ARIS_API defaultLogDirectory()->std::filesystem::path;
	// 设置log文件名，可以是相对或绝对路径，为空时采用默认值 //
	auto ARIS_API setDefaultLogFile(const std::filesystem::path &log_file_path = std::filesystem::path())->void;
	// 设置单个log文件最大的条数，小于0时无上限 //
	auto ARIS_API setDefaultLogMaxInfoNum(int max_info_num = 100000);

	// 返回log stream //
	auto ARIS_API defaultLogStream()->std::ostream&;




	// 以下为默认log所使用的格式化方法 //

	// 返回当前程序的文件名，例如：C:/program/abc.exe -> abc
	auto ARIS_API logExeName()->std::string;
	// 返回时间格式 "%Y-%m-%d--%H-%M-%S"
	auto ARIS_API logFileTimeFormat(const std::chrono::system_clock::time_point &time)->std::string;

	enum {
		LOG_TYPE_WIDTH = 5,
		LOG_TIME_WIDTH = 20,
		LOG_FILE_WIDTH = 25,
		LOG_LINE_WIDTH = 5,
		LOG_SPACE_WIDTH = LOG_TYPE_WIDTH + 1 + LOG_TIME_WIDTH + 1 + LOG_FILE_WIDTH + 1 + LOG_LINE_WIDTH + 1,
	};

	auto ARIS_API cout()->std::ostream&;











	auto ARIS_API dateFormat(const std::chrono::system_clock::time_point &time)->std::string;
	auto ARIS_API timeFormat(const std::chrono::system_clock::time_point &time)->std::string;
	auto ARIS_API datetimeFormat(const std::chrono::system_clock::time_point &time)->std::string;








	enum class ARIS_API LogType : int {
		kSystem = 0,
    	kConfig,
    	kOperation
	};

	//enum class ARIS_API LogLvl : int {
	//	kDebug = 0,
	//	kInfo,
	//	kWarning,
	//	kError
	//};

	class ARIS_API DbLogCell {
	public:
		auto setTimeStamp(const std::chrono::system_clock::time_point &tp) { tp_ = tp; }
		auto timeStamp() const->const std::chrono::system_clock::time_point& { return tp_; }
		auto logLvl() const->LogLvl { return l_; }
		auto setLogLvl(LogLvl l)->void { l_ = l; }
		auto logType() const->LogType { return t_; }
		auto setLogType(LogType t)->void { t_ = t; }
		auto code() const->int { return code_; }
		auto setCode(int code)->void { code_ = code; }
		auto text() const->const std::string& { return text_; }
		auto setText(const std::string &text)->void { text_ = text; }

		auto strategy() const->AccessStrategy { return stg_; }
		auto setStrategy(AccessStrategy stg)->void { stg_ = stg; }

	private:
		std::chrono::system_clock::time_point tp_{std::chrono::system_clock::now()};
		LogLvl l_{LogLvl::kInfo};
		LogType t_{LogType::kSystem};
		int code_{0};
		std::string text_;

		AccessStrategy stg_{AccessStrategy::kYield};

	public:
		DbLogCell() = default;
		explicit DbLogCell(const std::string &text, 
						   const std::chrono::system_clock::time_point tp = std::chrono::system_clock::now(), 
						   LogLvl l = LogLvl::kInfo, 
						   LogType t = LogType::kSystem, 
						   int code = 0, 
						   AccessStrategy stg = AccessStrategy::kYield) 
			: text_(text), tp_(tp), l_(l), t_(t), code_(code), stg_(stg) {}
		virtual ~DbLogCell() = default;
	};

	auto ARIS_API operator<<(std::ostream &s, const DbLogCell &cell)->std::ostream&;

	class ARIS_API LogFilter {
	public:
		struct TimeRule {
			bool enable{true};
			std::chrono::system_clock::time_point begin;
			std::chrono::system_clock::time_point end;
		};

		struct LvlRule {
			bool enable{true};
			std::set<LogLvl> lvls;
		};

		struct TypeRule {
			bool enable{true};
			std::set<LogType> types;
		};

		struct CodeRule {
			bool enable{true};
			int min_code{0};
			int max_code{0};
		};

		struct CountRule {
			bool enable{true};
			unsigned start{0};
			unsigned offset{10};
		};

	public:
		auto setTimeRule(const TimeRule &rule)->void { time_rule_ = rule; }
		auto timeRule() const->const TimeRule& { return time_rule_; }
		auto setLvlRule(const LvlRule &rule)->void { lvl_rule_ = rule; }
		auto lvlRule() const->const LvlRule& { return lvl_rule_; }
		auto setTypeRule(const TypeRule &rule)->void { type_rule_ = rule; }
		auto typeRule() const->const TypeRule& { return type_rule_; }
		auto setCodeRule(const CodeRule &rule)->void { code_rule_ = rule; }
		auto codeRule() const->const CodeRule& { return code_rule_; }
		auto setOrder(bool order)->void { order_ = order; }
		auto order() const->bool { return order_; }
		auto setCountRule(const CountRule &rule)->void { count_rule_ = rule; }
		auto countRule() const->const CountRule& { return count_rule_; }
		
	private:
		TimeRule time_rule_{false};
		LvlRule lvl_rule_{false};
		TypeRule type_rule_{false};
		CodeRule code_rule_{false};
		bool order_{false};
		CountRule count_rule_{false};
	};

	class ARIS_API Sqlite3Log {
	public:
		static auto instance()->Sqlite3Log&;
		auto open(const std::filesystem::path &db_path)->void;
		auto toDb(const DbLogCell &cell)->void;
		auto close()->void;
		auto isOpen() const->bool;
		auto select(const LogFilter &filter) const->std::vector<DbLogCell>;

	private:
		sqlite3 *db_{nullptr};
		bool to_work_{false};
		std::thread worker_;
		std::unique_ptr<LockFreeArrayQueue<DbLogCell>> queue_{nullptr};
		mutable std::recursive_mutex mu_;
		
	private:
		Sqlite3Log() : queue_(new LockFreeArrayQueue<DbLogCell>(1024)) {}
		~Sqlite3Log();
	};

#define DBLOG_DEBUG(text, code, type) std::cout<<aris::core::DbLogCell(text, std::chrono::system_clock::now(), aris::core::LogLvl::kDebug, type, code);
#define DBLOG_INFO(text, code, type) std::cout<<aris::core::DbLogCell(text, std::chrono::system_clock::now(), aris::core::LogLvl::kInfo, type, code);
#define DBLOG_WARNING(text, code, type) std::cout<<aris::core::DbLogCell(text, std::chrono::system_clock::now(), aris::core::LogLvl::kWarning, type, code);
#define DBLOG_ERROR(text, code, type) std::cout<<aris::core::DbLogCell(text, std::chrono::system_clock::now(), aris::core::LogLvl::kError, type, code);
#define RT_DBLOG_DEBUG(text, code, type) std::cout<<aris::core::DbLogCell(text, std::chrono::system_clock::now(), aris::core::LogLvl::kDebug, type, code, aris::core::AccessStrategy::kAbandon);
#define RT_DBLOG_INFO(text, code, type) std::cout<<aris::core::DbLogCell(text, std::chrono::system_clock::now(), aris::core::LogLvl::kInfo, type, code, aris::core::AccessStrategy::kAbandon);
#define RT_DBLOG_WARNING(text, code, type) std::cout<<aris::core::DbLogCell(text, std::chrono::system_clock::now(), aris::core::LogLvl::kWarning, type, code, aris::core::AccessStrategy::kAbandon);
#define RT_DBLOG_ERROR(text, code, type) std::cout<<aris::core::DbLogCell(text, std::chrono::system_clock::now(), aris::core::LogLvl::kError, type, code, aris::core::AccessStrategy::kAbandon);

// #define EN 0
// #define CHS 1
	// class ARIS_API CodeTextTable {
	// public:
	// 	static auto instance()->CodeTextTable& { static CodeTextTable obj; return obj; }
	// 	auto language() const->int { return language_; }
	// 	auto setLanguage(int language)->void { language_ = language; }
	// 	auto add(int code, int language, const std::string &text)->CodeTextTable& {
	// 		if (tbl_.find(code) == tbl_.end()) tbl_[code] = {};
	// 		auto &mlang = tbl_.at(code);
	// 		if (mlang.find(language) != mlang.end())
	// 			THROW_FILE_LINE("Text has existed -- code:"+std::to_string(code)+" language: "+std::to_string(language));
	// 		mlang[language] = text;
	// 		return *this;
	// 	}
	// 	auto get(int code, int language) const->const std::string& {
	// 		auto it_code = tbl_.find(code);
	// 		if (it_code == tbl_.end()) THROW_FILE_LINE("Code '"+std::to_string(code)+"' doesn't exist.");
	// 		auto it_lang = it_code->second.find(language);
	// 		if (it_lang == it_code->second.end()) 
	// 			THROW_FILE_LINE("Code '"+std::to_string(code)+"' has no language '"+std::to_string(language)+"'.");
	// 		return it_lang->second;
	// 	}
	// 	auto get(int code) const->const std::string& { return get(code, language_); }
	// 	auto getAndFormat(int code, int language, ...) const->std::string;
	// 	auto table() const->const std::map<int, std::map<int, std::string>>& { return tbl_; }

	// private:
	// 	int language_{CHS};
	// 	std::map<int, std::map<int, std::string>> tbl_;

	// private:
	// 	CodeTextTable() = default;
	// 	virtual ~CodeTextTable() = default;
	// };

// #define EN_TEXT_REGISTER(code, text) aris::core::CodeTextTable::instance().add(code, EN, text)
// #define CHS_TEXT_REGISTER(code, text) aris::core::CodeTextTable::instance().add(code, CHS, text)
// #define EN_CHS_TEXT_REGISTER(code, en_text, chs_text) \
// EN_TEXT_REGISTER(code, en_text);					  \
// CHS_TEXT_REGISTER(code, chs_text)
// #define TEXT_GET_AND_FORMAT(code, ...) \
// aris::core::CodeTextTable::instance().getAndFormat(code, aris::core::CodeTextTable::instance().language(), ##__VA_ARGS__)

#define EN 0
#define CHN 1
	class ARIS_API LocaleString {
	public:
		static auto locale()->unsigned { return locale_; }
		static auto setLocale(unsigned locale)->void { locale_ = locale; }
		static auto select(std::initializer_list<std::string> strs)->std::string {
#define LOCALE_STRING_HELPER(id) \
case id : \
	if (strs.size() >= id+1) return *(strs.begin()+id); \
	else if (strs.size() == 1) return *(strs.begin()); \
	else return "";

			switch (locale_) {
			LOCALE_STRING_HELPER(EN)
			LOCALE_STRING_HELPER(CHN)
			default :
				if (strs.size() >= 1) return *(strs.begin());
				else return "";
			}

#undef LOCALE_STRING_HELPER

			return "";
		}

	private:
		static unsigned locale_;
	};
}

using aris::core::operator<<;


#endif
