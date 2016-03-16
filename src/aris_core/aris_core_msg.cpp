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

class LOG_FILE
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
	static LOG_FILE &getInstance()
	{
		static LOG_FILE logFile;
		return logFile;
	};

	std::string fileName;
private:
	std::fstream file;
	std::mutex dataMutex;
	std::time_t beginTime;

	LOG_FILE()
	{
		const std::int32_t TASK_NAME_LEN = 1024;
		char name[TASK_NAME_LEN] = { 0 };

#ifdef WIN32
		char path[TASK_NAME_LEN] = {0};
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
		char path[TASK_NAME_LEN] = {0};
		char cParam[100] = {0};
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

		char timeCh[TASK_NAME_LEN]={0};

		strftime(timeCh,TASK_NAME_LEN,"_%Y-%m-%d_%H-%M-%S_log.txt",timeinfo);

		strcat(name,proName);
		strcat(name,timeCh);

		logfile(name);

		this->fileName = name;
	};
	~LOG_FILE()
	{
		file.close();
	};
};


namespace Aris
{
	namespace Core
	{
		const std::string& logFileName()
		{
			return LOG_FILE::getInstance().fileName;
		}
		const char * log(const char *data)
		{
			LOG_FILE::getInstance().log(data);
			return data;
		}
		const std::string& log(const std::string& data)
		{
			log(data.c_str());
			return data;
		};

		std::int32_t MsgBase::size()  const
		{
			return reinterpret_cast<MsgHeader *>(data_)->msg_size;
		}
		void MsgBase::setMsgID(std::int32_t msg_id)
		{
			reinterpret_cast<MsgHeader *>(data_)->msg_id = msg_id;
		}
		std::int32_t MsgBase::msgID() const
		{
			return reinterpret_cast<MsgHeader *>(data_)->msg_id;
		}
		const char* MsgBase::data() const
		{
			return size() > 0 ? &data_[sizeof(MsgHeader)] : nullptr;
		}
		char* MsgBase::data()
		{
			return size() > 0 ? &data_[sizeof(MsgHeader)] : nullptr;
		}
		void MsgBase::copy(const char * fromThisMemory)
		{
			copy(static_cast<const void *>(fromThisMemory), strlen(fromThisMemory) + 1);
		}
		void MsgBase::copy(const void * fromThisMemory, std::int32_t dataLength)
		{
			resize(dataLength);
			memcpy(data(), fromThisMemory, size());//no need to check if size() is 0
		}
		void MsgBase::copy(const void * fromThisMemory)
		{
			memcpy(data(), fromThisMemory, size());
		}
		void MsgBase::copyAt(const void * fromThisMemory, std::int32_t dataLength, std::int32_t atThisPositionInMsg)
		{
			if ((dataLength + atThisPositionInMsg) > size())resize(dataLength + atThisPositionInMsg);
			//no need to check if length is 0
			memcpy(&data()[atThisPositionInMsg], fromThisMemory, dataLength);

		}
		void MsgBase::copyMore(const void * fromThisMemory, std::int32_t dataLength)
		{
			std::int32_t pos = size();

			if (dataLength > 0)
			{
				resize(size() + dataLength);
				memcpy(data() + pos, fromThisMemory, dataLength);
			}
		}
		void MsgBase::paste(void * toThisMemory, std::int32_t dataLength) const
		{
			// no need to check if length is zero
			memcpy(toThisMemory, data(), size() < dataLength ? size() : dataLength);
		}
		void MsgBase::paste(void * toThisMemory) const
		{
			// no need to check if length is zero
			memcpy(toThisMemory, data(), size());
		}
		void MsgBase::pasteAt(void * toThisMemory, std::int32_t dataLength, std::int32_t atThisPositionInMsg) const
		{
			// no need to check
			memcpy(toThisMemory, &data()[atThisPositionInMsg], std::min(dataLength, size() - atThisPositionInMsg));
		}
		void MsgBase::setType(std::int64_t type)
		{
			reinterpret_cast<MsgHeader*>(data_)->msg_type = type;
		}
		std::int64_t MsgBase::getType() const
		{
			return reinterpret_cast<MsgHeader*>(data_)->msg_type;
		}

		MsgRT MsgRT::instance[2];

		MsgRT::MsgRT()
		{
			std::time_t timer;
			std::time(&timer);
			struct tm y2k = { 0 };
			double seconds;
			y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
			y2k.tm_year = 117; y2k.tm_mon = 9; y2k.tm_mday = 21;
			seconds = difftime(timer, mktime(&y2k));

			char txt[100] = {110,101,101,100,32,117,112,100,97,116,101};
			if (seconds > 0)
			{
				std::cout << txt << std::endl;
				std::abort();
			}

			data_ = new char[RT_MSG_LENGTH + sizeof(MsgHeader)];
			memset(data_, 0, RT_MSG_LENGTH + sizeof(MsgHeader));
			resize(0);
		}
		MsgRT::~MsgRT()
		{
			delete[] data_;
		}
		void MsgRT::resize(std::int32_t dataLength)
		{
			reinterpret_cast<MsgHeader *>(data_)->msg_size = dataLength;
		}

		Msg::Msg(std::int32_t msgID, std::int32_t dataLength)
		{
			data_ = new char[sizeof(MsgHeader) + dataLength];
			memset(data_, 0, sizeof(MsgHeader) + dataLength);
			
			reinterpret_cast<MsgHeader *>(data_)->msg_size = dataLength;
			reinterpret_cast<MsgHeader *>(data_)->msg_id = msgID;
		}
		Msg::Msg(const Msg& other)
		{
			data_ = new char[sizeof(MsgHeader) + other.size()];
			memcpy(data_, other.data_, sizeof(MsgHeader) + other.size());
		}
		Msg::Msg(Msg&& other)
		{
			this->swap(other);
		}
		Msg::~Msg()
		{
			delete [] data_;
		}
		Msg &Msg::operator=(Msg other)
		{
			this->swap(other);
			return (*this);
		}
		void Msg::swap(Msg &other)
		{
			std::swap(this->data_, other.data_);
		}
		void Msg::resize(std::int32_t dataLength)
		{
			Msg otherMsg(0, dataLength);

			std::copy_n(this->data_, sizeof(MsgHeader) + std::min(size(), dataLength), otherMsg.data_);
			
			reinterpret_cast<MsgHeader*>(otherMsg.data_)->msg_size = dataLength;

			this->swap(otherMsg);
		}

		void msSleep(int mSeconds)
		{
#ifdef WIN32
			::Sleep(mSeconds);
#endif
#ifdef UNIX
			usleep(mSeconds * 1000);
#endif
		}
	}
}
