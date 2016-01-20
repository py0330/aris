#ifndef ARIS_PIPE_H
#define ARIS_PIPE_H

#include <vector>
#include <memory>
#include <cstdint>

#include "aris_xml.h"
#include "aris_core.h"

namespace Aris
{
	namespace Control
	{	
		class PipeBase
		{
		protected:
			PipeBase(int port, bool isBlock);
			~PipeBase();

			int SendToRT_RawData(const void *pData, int size);
			int SendToNRT_RawData(const void* pData, int size);
			int RecvInRT_RawData(void* pData, int size);
			int RecvInNRT_RawData(void *pData, int size);

			PipeBase(const PipeBase&) = delete;
			PipeBase(PipeBase&&) = delete;

			class Imp;
			std::unique_ptr<Imp> pImp;
		};

		template <typename StandardLayoutStruct>
		class Pipe:private PipeBase
		{
		public:
			Pipe(int port, bool isBlock) :PipeBase(port, isBlock) {};
			int SendToRT(const StandardLayoutStruct &data)
			{
				return SendToRT_RawData(static_cast<const void*>(&data), sizeof(data));
			};
			int SendToNRT(const StandardLayoutStruct &data)
			{
				return SendToNRT_RawData(static_cast<const void*>(&data), sizeof(data));
			};
			int RecvInRT(StandardLayoutStruct &data)
			{
				return RecvInRT_RawData(static_cast<void*>(&data), sizeof(data));
			};
			int RecvInNRT(StandardLayoutStruct &data)
			{
				return RecvInNRT_RawData(static_cast<void*>(&data), sizeof(data));
			};
		};
		
		template <typename T>
		class Pipe<std::vector<T> > :public PipeBase
		{
		public:
			Pipe(int port, bool isBlock, int size):PipeBase(port, isBlock), size(size) {};
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
		class Pipe<Aris::Core::Msg>:public PipeBase
		{
		public:
			Pipe(int port, bool isBlock);
			int SendToRT(const Aris::Core::Msg &msg);
			int SendToNRT(const Aris::Core::MsgRT &msg);
			int RecvInRT(Aris::Core::MsgRT &msg);
			int RecvInNRT(Aris::Core::Msg &msg);
		};

		

	}
}



















#endif
