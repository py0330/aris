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

#include "aris_core_msg.h"



namespace aris
{
	namespace core
	{
		class LogFile
		{
		public:
			void log(const char *data)
			{
				std::lock_guard<std::mutex> lck(file_mutex_);
				file_ << logFileTimeFormat(std::chrono::system_clock::now()) << ":" << data << std::endl;
			}
			static LogFile &instance()
			{
				static LogFile logFile;
				return logFile;
			}

			std::fstream file_;
			std::string file_name_;
			std::mutex file_mutex_;
			std::chrono::system_clock::time_point begin_time_;

		private:
			LogFile()
			{
				begin_time_ = std::chrono::system_clock::now();
				createLogDir();
				file_name_ = logDirPath() + logExeName() + "_" + logFileTimeFormat(begin_time_) + "_log.txt";

				file_.close();
				file_.open(file_name_.c_str(), std::ios::out | std::ios::trunc);
				if (!file_.good())throw std::runtime_error("can't not Start log function");

				file_ << logFileTimeFormat(begin_time_) << "program \"" << logExeName() << "\"" << " started" << std::endl;
			}
			~LogFile()
			{
				file_.close();
			}
		};
		
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
			strftime(time_format, 1024, "%Y-%m-%d_%H-%M-%S", timeinfo);
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
		auto logFileName()->const std::string&{ return LogFile::instance().file_name_; }
		auto log(const char *data)->const char * { LogFile::instance().log(data);	return data; }
		auto log(const std::string& data)->const std::string &{ log(data.c_str());	return data; }

		auto MsgBase::copy(const char *src)->void 
		{ 
			copy(static_cast<const void *>(src), static_cast<MsgSize>(strlen(src) + 1));
		}
		auto MsgBase::copy(const void *src, MsgSize data_size)->void { resize(data_size); copy(src); }
		auto MsgBase::copy(const void *src)->void { copyAt(src, size(), 0); }
		auto MsgBase::copyAt(const void *src, MsgSize data_size, MsgSize at_this_pos_of_msg)->void
		{
			if ((data_size + at_this_pos_of_msg) > size())resize(data_size + at_this_pos_of_msg);
			std::copy_n(static_cast<const char *>(src), data_size, data() + at_this_pos_of_msg);
		}
		auto MsgBase::copyMore(const void *src, MsgSize data_size)->void{ copyAt(src, data_size, size()); }
		auto MsgBase::paste(void *tar, MsgSize data_size) const->void { std::copy_n(data(), std::min(size(), data_size), static_cast<char*>(tar)); }
		auto MsgBase::paste(void *tar) const->void{ std::copy_n(data(), size(), static_cast<char*>(tar)); }
		auto MsgBase::pasteAt(void *tar, MsgSize data_size, MsgSize at_this_pos_of_msg) const->void
		{
			std::copy_n(data() + at_this_pos_of_msg, std::min(data_size, size() - at_this_pos_of_msg), static_cast<char*>(tar));
		}

		auto Msg::swap(Msg &other)->void { std::swap(data_, other.data_); std::swap(capacity_, other.capacity_); }
		auto Msg::resize(MsgSize data_size)->void
		{
			if (!data_)
			{
				data_.reset(new char[sizeof(MsgHeader) + data_size]());
			}
			else if (capacity_ < data_size)
			{
				capacity_ = std::max(data_size, 2 * capacity_);
				std::unique_ptr<char[]> other(new char[sizeof(MsgHeader) + capacity_]());
				std::copy_n(data_.get(), sizeof(MsgHeader) + size(), other.get());
				std::swap(data_, other);
			}

			header().msg_size_ = data_size;
		}
		auto Msg::header()->MsgHeader& { return *reinterpret_cast<MsgHeader*>(data_.get()); }
		auto Msg::header()const->const MsgHeader& { return *reinterpret_cast<const MsgHeader*>(data_.get()); }
		Msg::~Msg() = default;
		Msg::Msg(MsgID msg_id, MsgSize size) 
		{ 
			resize(size);
			setMsgID(msg_id);
			header().msg_type_ = 0;
			header().reserved1_ = 0;
			header().reserved2_ = 0;
			header().reserved3_ = 0;
		}
		Msg::Msg(const std::string &msg_str) 
		{ 
			resize(static_cast<MsgSize>(msg_str.size() + 1));
			std::copy(msg_str.begin(), msg_str.end(), data());
			setMsgID(0);
			header().msg_type_ = 0;
			header().reserved1_ = 0;
			header().reserved2_ = 0;
			header().reserved3_ = 0;
		}
		Msg::Msg(const MsgBase &other)
		{
			resize(other.size());
			std::copy_n(reinterpret_cast<const char*>(&other.header()), other.size() + sizeof(MsgHeader), reinterpret_cast<char*>(&header()));
		}
		Msg::Msg(const Msg& other)
		{
			resize(other.size());
			std::copy_n(other.data_.get(), sizeof(MsgHeader) + other.size(), data_.get());
		}
		Msg::Msg(Msg&& other) { swap(other); }
		Msg& Msg::operator=(Msg other) { swap(other); return (*this); }

		auto MsgStreamBuf::overflow(int_type c)->int_type
		{
			update();

			msg_->resize(msg_->size() + 1);
			resetBuf();
			if (msg_->capacity() < msg_->size())
			{
				return traits_type::eof();
			}
			else
			{
				*(pptr() - 1) = c;
				return c;
			}
		}
		auto MsgStreamBuf::underflow()->int_type
		{
			update();
			
			if (gptr() == pptr()) return traits_type::eof();
			else return *(reinterpret_cast<char*>(&msg_->header()) + sizeof(MsgHeader));
		}
		auto MsgStreamBuf::update()->void
		{
			auto msg_buf = reinterpret_cast<char*>(&msg_->header()) + sizeof(MsgHeader);
			if (gptr() != msg_buf)std::copy(gptr(), pptr(), msg_buf);
			msg_->resize(static_cast<MsgSize>(pptr() - gptr()));
			resetBuf();
		}
		auto MsgStreamBuf::resetBuf()->void
		{
			auto msg_buf = reinterpret_cast<char*>(&msg_->header()) + sizeof(MsgHeader);
			setp(msg_buf + msg_->size(), msg_buf + msg_->capacity());
			setg(msg_buf, msg_buf, msg_buf + msg_->size());
		}
		MsgStreamBuf::MsgStreamBuf(MsgBase& msg) :msg_(&msg), std::streambuf(){ resetBuf();	};
	}
}
