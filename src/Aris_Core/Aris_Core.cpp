#include <Platform.h>

#include "Aris_Core.h"
#include <cstring>
#include <fstream>
#include <ctime>
#include <mutex>



#ifdef PLATFORM_IS_LINUX
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>//for mkdir()
#include <dirent.h>
#include <sys/types.h>
#endif

#ifdef PLATFORM_IS_WINDOWS
#include <windows.h>
#endif

class LOG_FILE
{
private:
	std::fstream file;
	std::mutex dataMutex;
	std::time_t beginTime;
public:
	LOG_FILE()
	{
		const int TASK_NAME_LEN = 256;
		char name[TASK_NAME_LEN] = { 0 };

#ifdef PLATFORM_IS_WINDOWS

		char path[TASK_NAME_LEN] = {0};
		GetModuleFileName(NULL, path, TASK_NAME_LEN);

		char *p = strrchr(path, '\\');
		if (p == nullptr)
			throw std::logic_error("windows can't identify the program name");
		

		
		char proName[TASK_NAME_LEN] = { 0 };

		char *dot = strrchr(path, '.');
		if ((dot != nullptr) && (dot - p > 0))
		{
			unsigned n = dot - p - 1;
			strncpy(proName, p + 1, n);
		}

		CreateDirectory("log", NULL);
		strcat(name, "log\\");
#endif

#ifdef PLATFORM_IS_LINUX
		int count = 0;
		int nIndex = 0;
		char path[TASK_NAME_LEN] = {0};
		char cParam[100] = {0};
		char *proName = path;
		int tmp_len;

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
};

static LOG_FILE log_file;


namespace Aris
{
	namespace Core
	{
		const char * log(const char *data)
		{
			log_file.log(data);
			return data;
		}
		
		RT_MSG RT_MSG::instance[2];
		
		void Core::MSG_BASE::SetMsgID(int msgID)
		{
			*((int *)(_pData + 4)) = msgID;
		}
		unsigned int Core::MSG_BASE::GetLength()  const
		{
			return *((unsigned int *)_pData);
		}
		int Core::MSG_BASE::GetMsgID() const
		{
			return *((int *)(_pData + 4));
		}
		char* Core::MSG_BASE::GetDataAddress() const
		{
			if (GetLength() > 0)
				return &_pData[MSG_HEADER_LENGTH];
			else
				return nullptr;
		}

		void Core::MSG_BASE::SetType(long long type)
		{
			*((long long *)(_pData + 8)) = type;
		}
		long long Core::MSG_BASE::GetType() const
		{
			return *((long long *)(_pData + 8));
		}

		void Core::MSG_BASE::Copy(const char * fromThisMemory)
		{
			Copy((void*)fromThisMemory, strlen(fromThisMemory) + 1);
		}
		void Core::MSG_BASE::Copy(const void * fromThisMemory, unsigned int dataLength)
		{
			SetLength(dataLength);
			if (dataLength > 0)
			{
				memcpy(GetDataAddress(), fromThisMemory, dataLength);
			}
		}
		void Core::MSG_BASE::Copy(const void * fromThisMemory)
		{
			if (GetLength() > 0)
			{
				memcpy(GetDataAddress(), fromThisMemory, GetLength());
			}
		}
		void Core::MSG_BASE::CopyAt(const void * fromThisMemory, unsigned int dataLength, unsigned int atThisPositionInMsg)
		{
			if ((dataLength + atThisPositionInMsg) > GetLength())
			{
				SetLength(dataLength + atThisPositionInMsg);
			}

			if (dataLength > 0)
			{
				memcpy(&GetDataAddress()[atThisPositionInMsg], fromThisMemory, dataLength);
			}

		}
		void Core::MSG_BASE::CopyMore(const void * fromThisMemory, unsigned int dataLength)
		{
			unsigned int pos = GetLength();

			if (dataLength > 0)
			{
				SetLength(GetLength() + dataLength);
				memcpy(&GetDataAddress()[pos], fromThisMemory, dataLength);
			}
		}
		void Core::MSG_BASE::Paste(void * toThisMemory, unsigned int dataLength) const
		{
			if ((dataLength > 0) && (GetLength() > 0))
			{
				if (dataLength > GetLength())
				{
					memcpy(toThisMemory, GetDataAddress(), GetLength());
				}
				else
				{
					memcpy(toThisMemory, GetDataAddress(), dataLength);
				}
			}
		}
		void Core::MSG_BASE::Paste(void * toThisMemory) const
		{
			if (GetLength() > 0)
			{
				memcpy(toThisMemory, GetDataAddress(), GetLength());
			}
		}
		void Core::MSG_BASE::PasteAt(void * toThisMemory, unsigned int dataLength, unsigned int atThisPositionInMsg) const
		{
			int actualLength = ((atThisPositionInMsg + dataLength) > GetLength() ? (GetLength() - atThisPositionInMsg) : dataLength);

			if (actualLength > 0)
			{
				memcpy(toThisMemory, &GetDataAddress()[atThisPositionInMsg], actualLength);
			}
		}

		RT_MSG::RT_MSG()
		{
			_pData = new char[8192 + MSG_HEADER_LENGTH];
			memset(_pData, 0, 8192 + MSG_HEADER_LENGTH);
			SetLength(0);
		}
		RT_MSG::~RT_MSG()
		{
			delete[] _pData;
		}
		void RT_MSG::SetLength(unsigned int dataLength)
		{
			*((unsigned int *)_pData) = dataLength;
		}

		Core::MSG::MSG(int msgID, unsigned int dataLength)
		{
			_pData = new char[MSG_HEADER_LENGTH + dataLength];
			memset(_pData, 0, MSG_HEADER_LENGTH);
			*((unsigned int *)_pData) = dataLength;
			*((int *)(_pData + 4)) = msgID;
		}
		Core::MSG::MSG(const MSG& other)
		{
			_pData = new char[MSG_HEADER_LENGTH + other.GetLength()];
			memcpy(_pData, other._pData, MSG_HEADER_LENGTH + other.GetLength());
		}
		Core::MSG::MSG(MSG&& other)
		{
			_pData = other._pData;
			other._pData = nullptr;
		}
		Core::MSG::~MSG()
		{
			delete [] _pData;
		}

		Core::MSG &Core::MSG::operator=(const MSG& other)
		{
			char *pNew = new char[MSG_HEADER_LENGTH + other.GetLength()];
			memcpy(pNew, other._pData, MSG_HEADER_LENGTH + other.GetLength());
			delete[]_pData;
			_pData = pNew;

			return *this;
		}
		Core::MSG &Core::MSG::operator=(MSG&& other)
		{
			delete[]_pData;
			_pData = other._pData;
			other._pData = nullptr;

			return *this;
		}

		void Core::MSG::SetLength(unsigned int dataLength)
		{
			char * pOldData = _pData;
			unsigned int oldLength = this->GetLength();

			_pData = new char[MSG_HEADER_LENGTH + dataLength];
			memcpy(_pData, pOldData, MSG_HEADER_LENGTH + 
				(((oldLength) < (dataLength)) ? (oldLength) : (dataLength)));

			*((unsigned int *)_pData) = dataLength;

			delete [] pOldData;
		}
		void Core::MSG::Swap(MSG &other)
		{
			char * pData;
			pData = this->_pData;
			this->_pData = other._pData;
			other._pData = pData;
		}
	}
}
