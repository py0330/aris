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

class LogFile
{
public:
	void log(const char *data)
	{
		std::lock_guard<std::mutex> lck(dataMutex);

		time_t now;
		time(&now);

		file << difftime(now, beginTime) << ":";
		file << data << std::endl;
	}
	void logfile(const char *address)
	{
		file.close();
		file.open(address, std::ios::out | std::ios::trunc);
		if (!file.good())
		{
			throw std::logic_error("can't not Start log function");
		}

		time_t now = beginTime;
		struct tm * timeinfo;
		timeinfo = localtime(&now);
		file << asctime(timeinfo) << std::endl;
	}
	static LogFile &instance()
	{
		static LogFile logFile;
		return logFile;
	}

	std::string fileName;
private:
	std::fstream file;
	std::mutex dataMutex;
	std::time_t beginTime;

	LogFile()
	{
		const std::int32_t TASK_NAME_LEN = 1024;
		char name[TASK_NAME_LEN] = { 0 };

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
			std::int32_t n = dot - p - 1;
			strncpy(proName, p + 1, n);
			proName[n] = 0;
		}

		CreateDirectory("log", NULL);
		strcat(name, "log\\");
#endif

#ifdef UNIX
		std::int32_t count = 0;
		std::int32_t nIndex = 0;
		char path[TASK_NAME_LEN] = { 0 };
		char cParam[100] = { 0 };
		char *proName = path;

		pid_t pId = getpid();
		sprintf(cParam,"/proc/%d/exe",pId);
		count = readlink(cParam, path, TASK_NAME_LEN);

		if (count < 0 || count >= TASK_NAME_LEN)
	    {
	        throw std::logic_error("Current System Not Surport Proc.\n");
	    }
		else
		{
			nIndex = count - 1;

			for( ; nIndex >= 0; nIndex--)
			{
				if( path[nIndex] == '/' )//筛选出进程名
			    {
					nIndex++;
					proName += nIndex;
					break;
			    }
			}
		}


		if(opendir("log")==nullptr)
		{
			umask(0);
			if(mkdir("log",0777 )!=0)
						throw std::logic_error("can't create log folder\n");
		}


		strcat(name, "log/");
#endif

		time(&beginTime);
		struct tm * timeinfo;
		timeinfo = localtime(&beginTime);

		char timeCh[TASK_NAME_LEN] = { 0 };

		strftime(timeCh,TASK_NAME_LEN,"_%Y-%m-%d_%H-%M-%S_log.txt",timeinfo);

		strcat(name,proName);
		strcat(name,timeCh);

		logfile(name);

		this->fileName = name;
	}
	~LogFile()
	{
		file.close();
	}
};

namespace aris
{
	namespace core
	{
		auto logFileName()->const std::string&{ return LogFile::instance().fileName; }
		auto log(const char *data)->const char * { LogFile::instance().log(data);	return data; }
		auto log(const std::string& data)->const std::string &{ log(data.c_str());	return data; }
		auto msSleep(int mSeconds)->void
		{
#ifdef WIN32
			::Sleep(mSeconds);
#endif
#ifdef UNIX
			usleep(mSeconds * 1000);
#endif
		}

		auto MsgBase::size() const->std::int32_t
		{
			return reinterpret_cast<MsgHeader *>(data_)->msg_size_;
		}
		auto MsgBase::setMsgID(std::int32_t msg_id)->void
		{
			reinterpret_cast<MsgHeader *>(data_)->msg_id_ = msg_id;
		}
		auto MsgBase::msgID() const->std::int32_t
		{
			return reinterpret_cast<MsgHeader *>(data_)->msg_id_;
		}
		auto MsgBase::data() const->const char*
		{
			return size() > 0 ? &data_[sizeof(MsgHeader)] : nullptr;
		}
		auto MsgBase::data()->char*
		{
			return size() > 0 ? &data_[sizeof(MsgHeader)] : nullptr;
		}
		auto MsgBase::copy(const char * from_this_memory)->void { copy(static_cast<const void *>(from_this_memory), strlen(from_this_memory) + 1); }
		auto MsgBase::copy(const void * from_this_memory, std::int32_t data_size)->void
		{
			resize(data_size);
			std::copy(reinterpret_cast<const char*>(from_this_memory), reinterpret_cast<const char*>(from_this_memory) + size(), data());
		}
		auto MsgBase::copy(const void * from_this_memory)->void 
		{ 
			std::copy(reinterpret_cast<const char*>(from_this_memory), reinterpret_cast<const char*>(from_this_memory) + size(), data());
		}
		auto MsgBase::copyAt(const void * from_this_memory, std::int32_t data_size, std::int32_t at_this_pos_of_msg)->void
		{
			if ((data_size + at_this_pos_of_msg) > size())resize(data_size + at_this_pos_of_msg);
			//no need to check if length is 0
			memcpy(&data()[at_this_pos_of_msg], from_this_memory, data_size);

		}
		auto MsgBase::copyMore(const void * from_this_memory, std::int32_t data_size)->void
		{
			std::int32_t pos = size();

			if (data_size > 0)
			{
				resize(size() + data_size);
				memcpy(data() + pos, from_this_memory, data_size);
			}
		}
		auto MsgBase::paste(void * to_this_memory, std::int32_t data_size) const->void
		{
			memcpy(to_this_memory, data(), size() < data_size ? size() : data_size);
		}
		auto MsgBase::paste(void * to_this_memory) const->void
		{
			memcpy(to_this_memory, data(), size());
		}
		auto MsgBase::pasteAt(void * to_this_memory, std::int32_t data_size, std::int32_t at_this_pos_of_msg) const->void
		{
			memcpy(to_this_memory, &data()[at_this_pos_of_msg], std::min(data_size, size() - at_this_pos_of_msg));
		}
		auto MsgBase::header()->MsgHeader& { return std::ref(*reinterpret_cast<MsgHeader *>(data_)); }
		auto MsgBase::header()const->const MsgHeader&{ return std::ref(*reinterpret_cast<MsgHeader *>(data_)); }
		auto MsgBase::setType(std::int64_t type)->void { reinterpret_cast<MsgHeader*>(data_)->msg_type_ = type; }
		auto MsgBase::type() const->std::int64_t { return reinterpret_cast<MsgHeader*>(data_)->msg_type_; }

		auto MsgRT::instance()->MsgRtArray&
		{
			static MsgRtArray msg_rt_array;
			return msg_rt_array;
		}
		auto MsgRT::resize(std::int32_t data_size)->void
		{
			reinterpret_cast<MsgHeader *>(data_)->msg_size_ = data_size;
		}
		MsgRT::~MsgRT() { delete[] data_; }
		MsgRT::MsgRT()
		{
			data_ = new char[RT_MSG_SIZE + sizeof(MsgHeader)];
			std::fill(data_, data_ + sizeof(MsgHeader) + RT_MSG_SIZE, 0);
			resize(0);
		}
		
		auto Msg::swap(Msg &other)->void
		{
			std::swap(this->data_, other.data_);
		}
		auto Msg::resize(std::int32_t dataLength)->void
		{
			Msg otherMsg(0, dataLength);

			std::copy_n(this->data_, sizeof(MsgHeader) + std::min(size(), dataLength), otherMsg.data_);

			reinterpret_cast<MsgHeader*>(otherMsg.data_)->msg_size_ = dataLength;

			this->swap(otherMsg);
		}
		Msg::~Msg(){delete [] data_;}
		Msg::Msg(std::int32_t msg_id, std::int32_t data_size)
		{
			data_ = new char[sizeof(MsgHeader) + data_size]();
			reinterpret_cast<MsgHeader *>(data_)->msg_size_ = data_size;
			reinterpret_cast<MsgHeader *>(data_)->msg_id_ = msg_id;
		}
		Msg::Msg(const std::string &msg_str)
		{
			data_ = new char[sizeof(MsgHeader) + msg_str.size() + 1]();
			reinterpret_cast<MsgHeader *>(data_)->msg_size_ = msg_str.size() + 1;
			std::copy(msg_str.data(), msg_str.data() + msg_str.size() + 1, data_ + sizeof(MsgHeader));
		}
		Msg::Msg(const Msg& other)
		{
			data_ = new char[sizeof(MsgHeader) + other.size()];
			std::copy(other.data(), other.data() + sizeof(MsgHeader) + other.size(), data_);
		}
		Msg::Msg(Msg&& other) { swap(other); }
		Msg&Msg::operator=(Msg other) { swap(other); return (*this); }
	}
}
