#ifndef ARIS_PIPE_H
#define ARIS_PIPE_H

#include "Aris_XML.h"
#include "Aris_Core.h"

#include <vector>
#include <memory>
#include <cstdint>

namespace Aris
{
	namespace Control
	{	
		class PIPE_BASE
		{
		protected:
			PIPE_BASE(int port, bool isBlock);
			~PIPE_BASE();

			int SendToRT_RawData(const void *pData, int size);
			int SendToNRT_RawData(const void* pData, int size);
			int RecvInRT_RawData(void* pData, int size);
			int RecvInNRT_RawData(void *pData, int size);

			PIPE_BASE(const PIPE_BASE&) = delete;
			PIPE_BASE(PIPE_BASE&&) = delete;

			class IMP;
			std::unique_ptr<IMP> pImp;
		};

		template <typename STANDARD_LAYOUT_STRUCT>
		class PIPE:private PIPE_BASE
		{
		public:
			PIPE(int port, bool isBlock) :PIPE_BASE(port, isBlock) {};
			int SendToRT(const STANDARD_LAYOUT_STRUCT &data)
			{
				return SendToRT_RawData(static_cast<const void*>(&data), sizeof(data));
			};
			int SendToNRT(const STANDARD_LAYOUT_STRUCT &data)
			{
				return SendToNRT_RawData(static_cast<const void*>(&data), sizeof(data));
			};
			int RecvInRT(STANDARD_LAYOUT_STRUCT &data)
			{
				return RecvInRT_RawData(static_cast<void*>(&data), sizeof(data));
			};
			int RecvInNRT(STANDARD_LAYOUT_STRUCT &data)
			{
				return RecvInNRT_RawData(static_cast<void*>(&data), sizeof(data));
			};
		};
		
		template <typename T>
		class PIPE<std::vector<T> > :public PIPE_BASE
		{
		public:
			PIPE(int port, bool isBlock, int size):PIPE_BASE(port, isBlock), size(size) {};
			int SendToRT(const std::vector<T> &vec)
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only send fixed size vector");
				return SendToRT_RawData(vec.data(), sizeof(T)*size);
			}
			int SendToNRT(const std::vector<T> &vec)
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only send fixed size vector");
				return SendToNRT_RawData(vec.data(), sizeof(T)*size);
			}
			int RecvInRT(std::vector<T> &vec)
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only recv fixed size vector");
				return RecvInRT_RawData(vec.data(), sizeof(T)*size);
			}
			int RecvInNRT(std::vector<T> &vec)
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only recv fixed size vector");
				return RecvInNRT_RawData(vec.data(), sizeof(T)*size);
			}

		private:
			int size;
		};

		template <>
		class PIPE<Aris::Core::MSG>:public PIPE_BASE
		{
		public:
			PIPE(int port, bool isBlock);
			int SendToRT(const Aris::Core::MSG &msg);
			int SendToNRT(const Aris::Core::RT_MSG &msg);
			int RecvInRT(Aris::Core::RT_MSG &msg);
			int RecvInNRT(Aris::Core::MSG &msg);
		};

		

	}
}



















#endif
