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

#include "aris_core_log.h"
#include "aris_core_basic_type.h"
#include "aris_core_msg.h"


namespace aris
{
	namespace core
	{
		std::ofstream log_file;
		std::recursive_mutex log_file_mutex;
		
		class LogStreamBuf :public std::streambuf
		{
		public:
			virtual auto overflow(int_type c)->int_type override 
			{
				msg_.resize(msg_.capacity());
				msg_.resize(msg_.capacity() + 1);
				setp(msg_.data() + msg_.size(), msg_.data() + msg_.capacity());
				*(pptr() - 1) = c;
				return c;
			}
			virtual auto sync()->int override
			{
				std::unique_lock<std::recursive_mutex> lck(log_file_mutex);

				if (!log_file.is_open())logFile(logExeName() + "--" + logFileTimeFormat(std::chrono::system_clock::now()) + "--log.txt");

				msg_.resize(msg_.capacity() - (epptr() - pptr()));
				std::string msg_data(msg_.data(), msg_.size());
				log_file << msg_data;
				log_file.flush();

				msg_.resize(0);
				setp(msg_.data() + msg_.size(), msg_.data() + msg_.capacity());

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
		auto logFile(const std::string &file_name)->void
		{
			std::unique_lock<std::recursive_mutex> lck(log_file_mutex);

			createLogDir();
			log_file.close();
			log_file.open(logDirPath() + file_name, std::ios::out | std::ios::trunc);
		}



		auto createLogDir()->void
		{
#ifdef WIN32
			CreateDirectory("log", NULL);
#endif

#ifdef UNIX
			if (opendir("log") == nullptr)
			{
				umask(0);
				if (mkdir("log", 0777) != 0)
					throw std::logic_error("can't create log folder\n");
			}
#endif
		}
		auto logExeName()->std::string 
		{
			const std::int32_t TASK_NAME_LEN = 1024;

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
			std::int32_t count = 0;
			std::int32_t nIndex = 0;
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
		auto logDirPath()->std::string
		{
#ifdef WIN32
			return std::string("log\\");
#endif

#ifdef UNIX
			return std::string("log/");
#endif
			
		}

	}
}
