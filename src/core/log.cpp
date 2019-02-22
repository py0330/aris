#include <cstring>
#include <fstream>
#include <ctime>
#include <mutex>
#include <algorithm>
#include <iostream>


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


namespace aris::core
{
	static std::filesystem::path log_dir_path_;
	static std::filesystem::path log_file_path_;
	static std::ofstream log_fstream_;
	static std::ostream *log_stream_{ nullptr };
	static std::recursive_mutex log_file_mutex;

	class LogStreamBuf :public std::streambuf
	{
	public:
		virtual auto overflow(int_type c)->int_type override
		{
			msg_.resize(msg_.capacity());// 保证在下次resize的时候，所有数据都会被copy，这是因为在resize重新分配内存时，不是按照capacity来copy
			msg_.resize(msg_.capacity() + 1);
			setp(msg_.data() + msg_.size(), msg_.data() + msg_.capacity());
			*(pptr() - 1) = c;
			return c;
		}
		virtual auto sync()->int override
		{
			std::unique_lock<std::recursive_mutex> lck(log_file_mutex);

			if (!log_stream_)logStream();

			msg_.resize(msg_.capacity() - static_cast<MsgSize>(epptr() - pptr()));
			*log_stream_ << msg_.toString() <<std::flush;

			msg_.resize(0);
			setp(msg_.data(), msg_.data() + msg_.capacity());

			return 0;
		}

	private:
		aris::core::Msg msg_;
	};
	class LogStream :public std::ostream { public: LogStream() :std::ostream(&buf_) {}; LogStreamBuf buf_; };
	auto log()->std::ostream&
	{
		static thread_local LogStream log_stream_;
		log_stream_.flush();
		return log_stream_;
	}
	auto logDirectory(const std::filesystem::path &log_dir)->void
	{
		std::unique_lock<std::recursive_mutex> lck(log_file_mutex);

		log_dir_path_ = log_dir.empty() ? std::filesystem::absolute("log") : std::filesystem::absolute(log_dir);
		std::filesystem::create_directories(log_dir_path_);
	}
	auto logFile(const std::filesystem::path &log_file_path)->void
	{
		std::unique_lock<std::recursive_mutex> lck(log_file_mutex);

		log_file_path_ = log_file_path.empty() ? std::filesystem::path(logExeName() + "--" + logFileTimeFormat(std::chrono::system_clock::now()) + "--log.txt") : log_file_path;
		log_file_path_ = log_file_path_.has_root_path() ? log_file_path_ : logDirPath() / log_file_path_;
		std::filesystem::create_directories(log_file_path_.parent_path());

		log_fstream_.close();
		log_fstream_.open(log_file_path_, std::ios::out | std::ios::trunc);
	}
	auto logStream(std::ostream *s)->void 
	{ 
		std::unique_lock<std::recursive_mutex> lck(log_file_mutex);

		if (s == nullptr && !log_fstream_.is_open())logFile();
		log_stream_ = s ? s : &log_fstream_;
	}

	auto logDirPath()->std::filesystem::path 
	{ 
		if (log_dir_path_.empty())logDirectory();
		return log_dir_path_; 
	}
	auto logExeName()->std::string
	{
		const int TASK_NAME_LEN = 1024;

#ifdef WIN32
		char path[TASK_NAME_LEN] = { 0 };
		GetModuleFileName(NULL, path, TASK_NAME_LEN);

		char *p = strrchr(path, '\\');
		if (p == nullptr)
			throw std::logic_error("windows can't identify the program name");

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
			throw std::logic_error("Current System Not Surport Proc.\n");
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
	auto logFileTimeFormat(const std::chrono::system_clock::time_point &time)->std::string
	{
		auto time_t_var = std::chrono::system_clock::to_time_t(time);
		auto timeinfo = localtime(&time_t_var);
		char time_format[1024];
		strftime(time_format, 1024, "%Y-%m-%d--%H-%M-%S", timeinfo);
		return std::string(time_format);
	}
}