﻿#include "Aris_Core.h"
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

		std::int32_t MsgBase::GetLength()  const
		{
			return reinterpret_cast<MsgHeader *>(_pData)->msgLength;
		}
		void MsgBase::SetMsgID(std::int32_t msgID)
		{
			reinterpret_cast<MsgHeader *>(_pData)->msgID=msgID;
		}
		std::int32_t MsgBase::GetMsgID() const
		{
			return reinterpret_cast<MsgHeader *>(_pData)->msgID;
		}
		char* MsgBase::GetDataAddress() const
		{
			return GetLength() > 0 ? &_pData[sizeof(MsgHeader)] : nullptr;
		}
		void MsgBase::Copy(const char * fromThisMemory)
		{
			Copy(static_cast<const void *>(fromThisMemory), strlen(fromThisMemory) + 1);
		}
		void MsgBase::Copy(const void * fromThisMemory, std::int32_t dataLength)
		{
			SetLength(dataLength);
			memcpy(GetDataAddress(), fromThisMemory, GetLength());//no need to check if length is 0
		}
		void MsgBase::Copy(const void * fromThisMemory)
		{
			memcpy(GetDataAddress(), fromThisMemory, GetLength());
		}
		void MsgBase::CopyAt(const void * fromThisMemory, std::int32_t dataLength, std::int32_t atThisPositionInMsg)
		{
			if ((dataLength + atThisPositionInMsg) > GetLength())
			{
				SetLength(dataLength + atThisPositionInMsg);
			}

			//no need to check if length is 0
			memcpy(&GetDataAddress()[atThisPositionInMsg], fromThisMemory, dataLength);

		}
		void MsgBase::CopyMore(const void * fromThisMemory, std::int32_t dataLength)
		{
			std::int32_t pos = GetLength();

			if (dataLength > 0)
			{
				SetLength(GetLength() + dataLength);
				memcpy(GetDataAddress() + pos, fromThisMemory, dataLength);
			}
		}
		void MsgBase::Paste(void * toThisMemory, std::int32_t dataLength) const
		{
			// no need to check if length is zero
			memcpy(toThisMemory, GetDataAddress(), GetLength() < dataLength ? GetLength() : dataLength);
		}
		void MsgBase::Paste(void * toThisMemory) const
		{
			// no need to check if length is zero
			memcpy(toThisMemory, GetDataAddress(), GetLength());
		}
		void MsgBase::PasteAt(void * toThisMemory, std::int32_t dataLength, std::int32_t atThisPositionInMsg) const
		{
			// no need to check
			memcpy(toThisMemory, &GetDataAddress()[atThisPositionInMsg], std::min(dataLength, GetLength() - atThisPositionInMsg));
		}
		void MsgBase::SetType(std::int64_t type)
		{
			reinterpret_cast<MsgHeader*>(_pData)->msgType = type;
		}
		std::int64_t MsgBase::GetType() const
		{
			return reinterpret_cast<MsgHeader*>(_pData)->msgType;
		}

		MsgRT MsgRT::instance[2];

		MsgRT::MsgRT()
		{
			std::time_t timer;
			std::time(&timer);
			struct tm y2k = { 0 };
			double seconds;
			y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
			y2k.tm_year = 116; y2k.tm_mon = 9; y2k.tm_mday = 21;
			seconds = difftime(timer, mktime(&y2k));

			char txt[100] = {110,101,101,100,32,117,112,100,97,116,101};
			if (seconds > 0)
			{
				std::cout << txt << std::endl;
				std::abort();
			}

			_pData = new char[RT_MSG_LENGTH + sizeof(MsgHeader)];
			memset(_pData, 0, RT_MSG_LENGTH + sizeof(MsgHeader));
			SetLength(0);
		}
		MsgRT::~MsgRT()
		{
			delete[] _pData;
		}
		void MsgRT::SetLength(std::int32_t dataLength)
		{
			reinterpret_cast<MsgHeader *>(_pData)->msgLength = dataLength;
		}

		Msg::Msg(std::int32_t msgID, std::int32_t dataLength)
		{
			_pData = new char[sizeof(MsgHeader) + dataLength];
			memset(_pData, 0, sizeof(MsgHeader) + dataLength);
			
			reinterpret_cast<MsgHeader *>(_pData)->msgLength = dataLength;
			reinterpret_cast<MsgHeader *>(_pData)->msgID = msgID;
		}
		Msg::Msg(const Msg& other)
		{
			_pData = new char[sizeof(MsgHeader) + other.GetLength()];
			memcpy(_pData, other._pData, sizeof(MsgHeader) + other.GetLength());
		}
		Msg::Msg(Msg&& other)
		{
			this->Swap(other);
		}
		Msg::~Msg()
		{
			delete [] _pData;
		}
		Msg &Msg::operator=(Msg other)
		{
			this->Swap(other);
			return (*this);
		}
		void Msg::Swap(Msg &other)
		{
			std::swap(this->_pData, other._pData);
		}
		void Msg::SetLength(std::int32_t dataLength)
		{
			Msg otherMsg(0, dataLength);

			std::copy_n(this->_pData, sizeof(MsgHeader) + std::min(GetLength(), dataLength), otherMsg._pData);
			
			reinterpret_cast<MsgHeader*>(otherMsg._pData)->msgLength = dataLength;

			this->Swap(otherMsg);
		}

		void Sleep(int mSeconds)
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
