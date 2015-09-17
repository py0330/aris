#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "Aris_Core.h"
#include <cstring>
#include <fstream>
#include <ctime>
#include <mutex>
#include <algorithm>
#include <iostream>



#ifdef PLATFORM_IS_LINUX
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>//for mkdir()
#include <dirent.h>
#include <sys/types.h>
#endif

#ifdef PLATFORM_IS_WINDOWS
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
			throw std::logic_error("can't not start log function");
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

private:
	std::fstream file;
	std::mutex dataMutex;
	std::time_t beginTime;

	LOG_FILE()
	{
		const std::int32_t TASK_NAME_LEN = 1024;
		char name[TASK_NAME_LEN] = { 0 };

#ifdef PLATFORM_IS_WINDOWS
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

#ifdef PLATFORM_IS_LINUX
		std::int32_t count = 0;
		std::int32_t nIndex = 0;
		char path[TASK_NAME_LEN] = {0};
		char cParam[100] = {0};
		char *proName = path;
		std::int32_t tmp_len;

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

		std::int32_t MSG_BASE::GetLength()  const
		{
			return reinterpret_cast<MSG_HEADER *>(_pData)->msgLength;
		}
		void MSG_BASE::SetMsgID(std::int32_t msgID)
		{
			reinterpret_cast<MSG_HEADER *>(_pData)->msgID=msgID;
		}
		std::int32_t MSG_BASE::GetMsgID() const
		{
			return reinterpret_cast<MSG_HEADER *>(_pData)->msgID;
		}
		char* MSG_BASE::GetDataAddress() const
		{
			return GetLength() > 0 ? &_pData[sizeof(MSG_HEADER)] : nullptr;
		}
		void MSG_BASE::Copy(const char * fromThisMemory)
		{
			Copy(static_cast<const void *>(fromThisMemory), strlen(fromThisMemory) + 1);
		}
		void MSG_BASE::Copy(const void * fromThisMemory, std::int32_t dataLength)
		{
			SetLength(dataLength);
			memcpy(GetDataAddress(), fromThisMemory, GetLength());//no need to check if length is 0
		}
		void MSG_BASE::Copy(const void * fromThisMemory)
		{
			memcpy(GetDataAddress(), fromThisMemory, GetLength());
		}
		void MSG_BASE::CopyAt(const void * fromThisMemory, std::int32_t dataLength, std::int32_t atThisPositionInMsg)
		{
			if ((dataLength + atThisPositionInMsg) > GetLength())
			{
				SetLength(dataLength + atThisPositionInMsg);
			}

			//no need to check if length is 0
			memcpy(&GetDataAddress()[atThisPositionInMsg], fromThisMemory, dataLength);

		}
		void MSG_BASE::CopyMore(const void * fromThisMemory, std::int32_t dataLength)
		{
			std::int32_t pos = GetLength();

			if (dataLength > 0)
			{
				SetLength(GetLength() + dataLength);
				memcpy(GetDataAddress() + pos, fromThisMemory, dataLength);
			}
		}
		void MSG_BASE::Paste(void * toThisMemory, std::int32_t dataLength) const
		{
			// no need to check if length is zero
			memcpy(toThisMemory, GetDataAddress(), GetLength() < dataLength ? GetLength() : dataLength);
		}
		void MSG_BASE::Paste(void * toThisMemory) const
		{
			// no need to check if length is zero
			memcpy(toThisMemory, GetDataAddress(), GetLength());
		}
		void MSG_BASE::PasteAt(void * toThisMemory, std::int32_t dataLength, std::int32_t atThisPositionInMsg) const
		{
			// no need to check
			memcpy(toThisMemory, &GetDataAddress()[atThisPositionInMsg], std::min(dataLength, GetLength() - atThisPositionInMsg));
		}
		void MSG_BASE::SetType(std::int64_t type)
		{
			reinterpret_cast<MSG_HEADER*>(_pData)->msgType = type;
		}
		std::int64_t MSG_BASE::GetType() const
		{
			return reinterpret_cast<MSG_HEADER*>(_pData)->msgType;
		}

		RT_MSG RT_MSG::instance[2];

		RT_MSG::RT_MSG()
		{
			_pData = new char[RT_MSG_LENGTH + sizeof(MSG_HEADER)];
			memset(_pData, 0, RT_MSG_LENGTH + sizeof(MSG_HEADER));
			SetLength(0);
		}
		RT_MSG::~RT_MSG()
		{
			delete[] _pData;
		}
		void RT_MSG::SetLength(std::int32_t dataLength)
		{
			reinterpret_cast<MSG_HEADER *>(_pData)->msgLength = dataLength;
		}

		MSG::MSG(std::int32_t msgID, std::int32_t dataLength)
		{
			_pData = new char[sizeof(MSG_HEADER) + dataLength];
			memset(_pData, 0, sizeof(MSG_HEADER) + dataLength);
			
			reinterpret_cast<MSG_HEADER *>(_pData)->msgLength = dataLength;
			reinterpret_cast<MSG_HEADER *>(_pData)->msgID = msgID;
		}
		MSG::MSG(const MSG& other)
		{
			_pData = new char[sizeof(MSG_HEADER) + other.GetLength()];
			memcpy(_pData, other._pData, sizeof(MSG_HEADER) + other.GetLength());
		}
		MSG::MSG(MSG&& other)
		{
			this->Swap(other);
		}
		MSG::~MSG()
		{
			delete [] _pData;
		}
		MSG &MSG::operator=(MSG other)
		{
			this->Swap(other);
			return (*this);
		}
		void MSG::Swap(MSG &other)
		{
			std::swap(this->_pData, other._pData);
		}
		void MSG::SetLength(std::int32_t dataLength)
		{
			MSG otherMsg(0, sizeof(MSG_HEADER) + dataLength);

			memcpy(otherMsg._pData, this->_pData, sizeof(MSG_HEADER) + std::min(GetLength(), dataLength));
			
			reinterpret_cast<MSG_HEADER*>(otherMsg._pData)->msgLength = dataLength;

			this->Swap(otherMsg);
		}
	}
}
