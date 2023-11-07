#include <cstring>
#include <fstream>
#include <ctime>
#include <mutex>
#include <algorithm>
#include <iostream>
#include <atomic>
#include <map>
#include <sstream>


#ifdef UNIX
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>//for mkdir()
#include <dirent.h>
#include <sys/types.h>
#endif

#ifdef WIN32
#include <windows.h>
#undef min
#undef max
#endif

#include "aris/core/log.hpp"
#include "aris/core/basic_type.hpp"
#include "aris/core/msg.hpp"

namespace aris::core{
	// 语言 //
	int global_lang_id = 0;
	auto setLanguage(int language_id_)->void { 
#ifdef WIN32
		// Set console code page to UTF-8 so console known how to interpret string data
		SetConsoleOutputCP(CP_UTF8);

		// Enable buffering to prevent VS from chopping up UTF-8 byte sequences
		setvbuf(stdout, nullptr, _IOFBF, 1000);
#endif
		global_lang_id = language_id_;
	}
	auto currentLanguage()->int { return global_lang_id; }

	// 默认log实现 //
	class ThreadSafeStreamBuf :public std::streambuf {
	public:
		virtual auto overflow(int_type c)->int_type override {
			msg_.resize(msg_.capacity());// 保证在下次resize的时候，所有数据都会被copy，这是因为在resize重新分配内存时，不是按照capacity来copy
			msg_.resize(msg_.capacity() + 1);
			setp(msg_.data() + msg_.size(), msg_.data() + msg_.capacity());
			*(pptr() - 1) = c;
			return c;
		}
		virtual auto sync()->int override {
			std::unique_lock<std::recursive_mutex> lck(*real_mutex_);

			msg_.resize(msg_.capacity() - static_cast<MsgSize>(epptr() - pptr()));
			**real_stream_ << msg_.toString() << std::flush;

			msg_.resize(0);
			setp(msg_.data(), msg_.data() + msg_.capacity());

			return 0;
		}
		explicit ThreadSafeStreamBuf(std::ostream** real_stream, std::recursive_mutex *real_mutex) : real_stream_(real_stream), real_mutex_(real_mutex) { }

	private:
		aris::core::Msg msg_;
		std::ostream** real_stream_;
		std::recursive_mutex* real_mutex_;
	};
	template<int>
	class ThreadSafeStream :public std::ostream {
	public:
		static auto setStream(std::ostream& stream) ->void {
			std::unique_lock<std::recursive_mutex> lck(real_mutex_);
			real_stream_ = &stream;
		}
		// 这个构造函数只有在 static 成员没有时，才会设置默认的ostream
		ThreadSafeStream(std::ostream &default_stream) :std::ostream(&buf_) {
			std::unique_lock<std::recursive_mutex> lck(real_mutex_);
			real_stream_ = real_stream_ ? real_stream_ : &default_stream;
		};

		static std::recursive_mutex real_mutex_;
	private:
		ThreadSafeStreamBuf buf_{ &real_stream_, &real_mutex_ };
		static std::ostream* real_stream_;
	};
	template <int a>
	std::recursive_mutex ThreadSafeStream<a>::real_mutex_;
	template <int a>
	std::ostream* ThreadSafeStream<a>::real_stream_;

	static std::filesystem::path log_dir_path_;
	static std::filesystem::path log_file_path_;
	static std::ofstream log_fstream_;
	static std::recursive_mutex log_file_mutex;
	static std::atomic_int max_info_num = 100000, current_info_num = 0;
	static int log_file_num = 0;

	auto defaultLogStream()->std::ostream& {
		// 这个参数仅仅用于设置默认值 //
		static thread_local ThreadSafeStream<1> local_stream_(log_fstream_);

		// 线程锁 //
		std::unique_lock<std::recursive_mutex> lck(ThreadSafeStream<1>::real_mutex_);

		// 如果记录满了，关闭文件，因为上面有锁，所以不可能出现内存stream在sync，而这里在关闭的情况 //
		if (++current_info_num > max_info_num) {
			log_fstream_.close();
			current_info_num = 0;
			log_file_num++;
		}

		// 如果第一次或之前满了，打开文件 //
		if (!log_fstream_.is_open()) {
			setDefaultLogFile();
			ThreadSafeStream<1>::setStream(log_fstream_);
		}

		// 返回memory stream //
		return local_stream_;
	}

	// 默认log的接口 //
	auto setDefaultLogDirectory(const std::filesystem::path &log_dir)->void {
		std::unique_lock<std::recursive_mutex> lck(ThreadSafeStream<1>::real_mutex_);

		log_dir_path_ = log_dir.empty() ? std::filesystem::absolute(logExeDirectory() / "log") : std::filesystem::absolute(log_dir);
		std::filesystem::create_directories(log_dir_path_);
	}
	auto defaultLogDirectory()->std::filesystem::path {
		if (log_dir_path_.empty())setDefaultLogDirectory();
		return log_dir_path_;
	}
	auto setDefaultLogFile(const std::filesystem::path &log_file_path)->void {
		std::unique_lock<std::recursive_mutex> lck(ThreadSafeStream<1>::real_mutex_);

		log_file_path_ = log_file_path.empty() ? std::filesystem::path(logExeName() + "--" + logFileTimeFormat(std::chrono::system_clock::now()) + "--log.txt") : log_file_path;
		log_file_path_ = log_file_path_.has_root_path() ? log_file_path_ : defaultLogDirectory() / log_file_path_;
		log_file_path_.replace_filename(log_file_path_.filename().replace_extension().string() + std::string("_") + std::to_string(log_file_num) + log_file_path_.extension().string());
		std::filesystem::create_directories(log_file_path_.parent_path());

		log_fstream_.close();
		log_fstream_.open(log_file_path_, std::ios::out | std::ios::trunc);
	}
	auto setDefaultLogMaxInfoNum(int max_info_num_) { max_info_num.store(max_info_num_); }

	// log string //
	std::function<void(const char *msg)> log_string_ = defaultLogString;
	auto setLogMethod(std::function<void(const char *msg)> method)->void { log_string_ = method ? method : defaultLogString; }
	auto log(const char *msg)->void { log_string_(msg);}
	auto defaultLogString(const char *msg)->void {
		aris::core::defaultLogStream()
			<< msg
			<< std::endl;
	}

	// 上层日志方法，带日志数据结构 //
	std::function<void(LogData)> log_data_ = defaultLogData;
	auto setLogMethod(std::function<void(LogData )> method)->void { log_data_ = method ? method : defaultLogData; }
	auto log(LogData data)->void { log_data_(data); }
	auto defaultLogData(LogData d)->void {
		enum {
			LOG_TYPE_WIDTH = 7,
			LOG_TIME_WIDTH = 19,
			LOG_FILE_WIDTH = 25,
			LOG_LINE_WIDTH = 5,
			LOG_SPACE_WIDTH = LOG_TYPE_WIDTH + 1 + LOG_TIME_WIDTH + 1 + LOG_FILE_WIDTH + 1 + LOG_LINE_WIDTH + 1,
		};
		
		const char *lvl_str[] {"DEBUG", "INFO", "WARNING", "ERROR", "FATAL"};

		std::stringstream ss;
		std::string_view data(d.msg);
		bool is_first_time = true;
		while (data.size() > 0) {
			auto print = data.substr(0, data.find_first_of("\n"));

			if (is_first_time) {
				ss << std::setw(LOG_TYPE_WIDTH) << lvl_str[(int)d.level] << "|"
					<< std::setw(LOG_TIME_WIDTH) << aris::core::datetimeFormat(d.time) << "|"
					<< std::setw(LOG_FILE_WIDTH) << std::string_view(d.file_name).substr(std::string_view(d.file_name).find_last_of("/\\") + 1) << "|"
					<< std::setw(LOG_LINE_WIDTH) << d.line << "|"
					<< print;
			}
			else {
				ss << "\n"
					<< std::setw(LOG_SPACE_WIDTH) << "|"
					<< print;
					
			}


			is_first_time = false;
			data = data.substr(print.size() == data.size() ? data.size() : print.size() + 1);
		}
		
		//std::cout << ss.str() << std::endl;
		aris::core::defaultLogStream()
			<< ss.str()
			<< std::endl;
	}

	auto logExeName()->std::string{
		const int TASK_NAME_LEN = 1024;

#ifdef WIN32
		char path[TASK_NAME_LEN] = { 0 };
		GetModuleFileName(NULL, path, TASK_NAME_LEN);

		char *p = strrchr(path, '\\');
		if (p == nullptr)
			THROW_FILE_LINE("windows can't identify the program name");

		char proName[TASK_NAME_LEN]{ 0 };

		char *dot = strrchr(path, '.');
		if ((dot != nullptr) && (dot - p > 0))
		{
			auto n = dot - p - 1;
			strncpy(proName, p + 1, n);
			proName[n] = 0;
		}
#endif

#ifdef UNIX
		int count = 0;
		int nIndex = 0;
		char path[TASK_NAME_LEN] = { 0 };
		char cParam[100] = { 0 };
		char *proName = path;

		pid_t pId = getpid();
		sprintf(cParam, "/proc/%d/exe", pId);
		count = readlink(cParam, path, TASK_NAME_LEN);

		if (count < 0 || count >= TASK_NAME_LEN)
		{
			THROW_FILE_LINE("Current System Not Surport Proc.\n");
		}
		else
		{
			nIndex = count - 1;

			for (; nIndex >= 0; nIndex--)
			{
				if (path[nIndex] == '/')
				{
					nIndex++;
					proName += nIndex;
					break;
				}
			}
		}
#endif
		return std::string(proName);

	}
	auto logExeDirectory() -> std::filesystem::path {
		const int TASK_NAME_LEN = 1024;
#ifdef WIN32
		char path[TASK_NAME_LEN] = { 0 };
		GetModuleFileName(NULL, path, TASK_NAME_LEN);

		char* p = strrchr(path, '\\');

		

		if (p == nullptr)
			THROW_FILE_LINE("windows can't identify the program name");

		return std::string(path, p - path);
#endif

#ifdef UNIX
		int count = 0;
		int nIndex = 0;
		char path[TASK_NAME_LEN] = { 0 };
		char cParam[100] = { 0 };
		char* proName = path;

		pid_t pId = getpid();
		sprintf(cParam, "/proc/%d/exe", pId);
		count = readlink(cParam, path, TASK_NAME_LEN);

		if (count < 0 || count >= TASK_NAME_LEN)
		{
			THROW_FILE_LINE("Current System Not Surport Proc.\n");
		}
		else
		{
			nIndex = count - 1;

			for (; nIndex >= 0; nIndex--)
			{
				if (path[nIndex] == '/')
				{
					nIndex++;
					proName += nIndex;
					break;
				}
			}
		}
		return std::string(path, nIndex);
#endif
		//
	}
	auto logFileTimeFormat(const std::chrono::system_clock::time_point &time)->std::string{
		auto time_t_var = std::chrono::system_clock::to_time_t(time);
		auto timeinfo = localtime(&time_t_var);
		char time_format[1024];
		strftime(time_format, 1024, "%Y-%m-%d--%H-%M-%S", timeinfo);
		return std::string(time_format);
	}

	// 线程安全版输入输出 //
	auto cout()->std::ostream& {
		// 这个构造函数只有在 static 成员没有时，才会设置默认的ostream
		static thread_local ThreadSafeStream<0> local_stream_(std::cout);
		return local_stream_;
	}
	auto setCoutStream(std::ostream& cout_stream) { ThreadSafeStream<0>::setStream(cout_stream); }

	auto dateFormat(const std::chrono::system_clock::time_point &time)->std::string {
		const auto &str = datetimeFormat(time);
		return str.substr(0, str.find_first_of(' '));
	}
	auto timeFormat(const std::chrono::system_clock::time_point &time)->std::string {
		const auto &str = datetimeFormat(time);
		return str.substr(str.find_first_of(' ')+1);
	}
	auto datetimeFormat(const std::chrono::system_clock::time_point &time)->std::string {
		auto tt = std::chrono::system_clock::to_time_t(time);

#ifdef UNIX
		std::tm tm;
		localtime_r(&tt, &tm);
#endif

#ifdef WIN32
		std::tm tm;
		localtime_s(&tm, &tt);
#endif

		std::ostringstream os;
		os << std::put_time(&tm, "%F %T");
		return os.str();
	}
}