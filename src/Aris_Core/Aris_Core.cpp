#include "Aris_Core.h"
#include <cstring>
namespace Aris
{
	namespace Core
	{
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
		void Core::MSG::SetMsgID(int msgID)
		{
			*((int *)(_pData + 4)) = msgID;
		}
		unsigned int Core::MSG::GetLength()  const
		{
			return *((unsigned int *)_pData);
		}
		int Core::MSG::GetMsgID() const
		{
			return *((int *)(_pData + 4));
		}
		char* Core::MSG::GetDataAddress() const
		{
			if (GetLength() > 0)
				return &_pData[MSG_HEADER_LENGTH];
			else
				return nullptr;
		}

		void Core::MSG::SetType(long long type)
		{
			*((long long *)(_pData + 8)) = type;
		}
		long long Core::MSG::GetType() const
		{
			return *((long long *)(_pData + 8));
		}

		void Core::MSG::Copy(const void * fromThisMemory, unsigned int dataLength)
		{
			SetLength(dataLength);
			if (dataLength > 0)
			{
				memcpy(GetDataAddress(), fromThisMemory, dataLength);
			}
		}
		void Core::MSG::Copy(const void * fromThisMemory)
		{
			if (GetLength() > 0)
			{
				memcpy(GetDataAddress(), fromThisMemory, GetLength());
			}
		}
		void Core::MSG::CopyAt(const void * fromThisMemory, unsigned int dataLength, unsigned int atThisPositionInMsg)
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
		void Core::MSG::CopyMore(const void * fromThisMemory, unsigned int dataLength)
		{
			unsigned int pos = GetLength();

			if (dataLength > 0)
			{
				SetLength(GetLength() + dataLength);
				memcpy(&GetDataAddress()[pos], fromThisMemory, dataLength);
			}
		}

		void Core::MSG::Paste(void * toThisMemory, unsigned int dataLength) const
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
		void Core::MSG::Paste(void * toThisMemory) const
		{
			if (GetLength() > 0)
			{
				memcpy(toThisMemory, GetDataAddress(), GetLength());
			}
		}
		void Core::MSG::PasteAt(void * toThisMemory, unsigned int dataLength, unsigned int atThisPositionInMsg) const
		{
			int actualLength = ((atThisPositionInMsg + dataLength) > GetLength() ? (GetLength() - atThisPositionInMsg) : dataLength);

			if (actualLength > 0)
			{
				memcpy(toThisMemory, &GetDataAddress()[atThisPositionInMsg], actualLength);
			}
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