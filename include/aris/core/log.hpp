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
#include <functional>

#include "sqlite3.h"
#include "aris/core/data_structure.hpp"

#include <aris_lib_export.h>

//#define _LOG_I(level, code, info, ...)               \
//	aris::core::log(level, code, \
//	std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/\\") + 1) + "|"+ __LINE__+"|"+aris::core::localeString(info, ##__VA_ARGS__))
//
//
//#define LOG(...) _LOG_I(##__VA_ARGS__)




#define LOG_AND_THROW(error) ARIS_LOG(aris::core::LogLvl::kError, 0, error), throw std::runtime_error(error)

#define THROW_FILE_LINE(error) throw std::runtime_error(std::string(__FILE__) + "_" + std::to_string(__LINE__)+ ":" + error)

#define ARIS_COUT aris::core::cout()

namespace aris::core{
	
	enum class ARIS_API Language : int {
		kEnglish = 0,
		kSimplifiedChinese,
	};
	// 设置语言 id //
	auto ARIS_API setLanguage(int language_id)->void;
	// 当前语言 id //
	auto ARIS_API currentLanguage()->int;
	// 根据语言切换 //
	template <typename ...Args>
	auto localeString(std::initializer_list<const char*> format_list, Args ... args)->std::string {
		auto format = (currentLanguage() <= format_list.size()) ? format_list.begin()[currentLanguage()] : format_list.begin()[0];
		
		int size_s = std::snprintf(nullptr, 0, format, args ...) + 1;
		if (size_s <= 0) { throw std::runtime_error("Error during formatting."); }
		auto size = static_cast<size_t>(size_s);
		std::string ret;
		ret.resize(size);
		std::snprintf(ret.data(), size, format, args ...);
		return ret;
	};

	// log string //
	auto ARIS_API setLogMethod(std::function<void(const char *msg)> method = nullptr)->void;
	auto ARIS_API log(const char *msg)->void;
	auto ARIS_API defaultLogString(const char *msg)->void;

	// log data //
	enum class ARIS_API LogLvl : int {
		kDebug = 0,
		kInfo,
		kWarning,
		kError,
		kFatal
	};
	// 日志数据 //j
	struct LogData {
		std::chrono::system_clock::time_point time;
		const char *file_name;
		int line;
		LogLvl level;
		int code;
		std::string msg;

		static auto makeLogData(std::chrono::system_clock::time_point time,
			const char *file_name,
			int line,
			LogLvl level,
			int code,
			std::string msg)->LogData {
			return LogData{ time , file_name, line, level, code, msg};
		}
	};
	auto ARIS_API setLogMethod(std::function<void(LogData)> method = nullptr)->void;
	auto ARIS_API log(LogData data)->void;
	auto ARIS_API defaultLogData(LogData data)->void;
	
	// 方便宏 //
	// refer to https://stackoverflow.com/questions/50427015/possible-to-pass-comma-and-args-into-c-macro
	#define ARIS_UNPACK(macro, args) macro args

	#define ARIS_LOG_IMP(lvl, code, msgs, ...) aris::core::log(aris::core::LogData::makeLogData(std::chrono::system_clock::now(), __FILE__, __LINE__, lvl, code, aris::core::localeString(msgs, __VA_ARGS__)))
	#define ARIS_LOG(...) ARIS_UNPACK(ARIS_LOG_IMP, (__VA_ARGS__))

	// 默认log的文件夹、单文件最多条数等 //
	// 设置log文件夹，参数为空时将当前二进制文件路径设为log路径 //
	auto ARIS_API setDefaultLogDirectory(const std::filesystem::path &log_dir_path = std::filesystem::path())->void;
	// 获取log文件夹 //
	auto ARIS_API defaultLogDirectory()->std::filesystem::path;
	// 设置log文件名，可以是相对或绝对路径，为空时采用默认值 //
	auto ARIS_API setDefaultLogFile(const std::filesystem::path &log_file_path = std::filesystem::path())->void;
	// 设置单个log文件最大的条数，小于0时无上限 //
	auto ARIS_API setDefaultLogMaxInfoNum(int max_info_num = 100000);

	// 以下为默认log所使用的格式化方法 //

	// 返回当前程序的文件名，例如：C:/program/abc.exe -> abc
	auto ARIS_API logExeName()->std::string;
	// 返回时间格式 "%Y-%m-%d--%H-%M-%S"
	auto ARIS_API logFileTimeFormat(const std::chrono::system_clock::time_point &time)->std::string;
	auto ARIS_API dateFormat(const std::chrono::system_clock::time_point &time)->std::string;
	auto ARIS_API timeFormat(const std::chrono::system_clock::time_point &time)->std::string;
	// 返回时间格式 "%Y-%m-%d %H:%M:%S"
	auto ARIS_API datetimeFormat(const std::chrono::system_clock::time_point &time)->std::string;

	// 线程安全版cout //
	auto ARIS_API cout()->std::ostream&;

	
}

#endif
