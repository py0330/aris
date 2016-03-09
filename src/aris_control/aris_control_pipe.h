#ifndef ARIS_CONTROL_PIPE_H
#define ARIS_CONTROL_PIPE_H

#include <vector>
#include <memory>
#include <cstdint>

#include "aris_core.h"

namespace Aris
{
	namespace Control
	{	
		class PipeBase
		{
		protected:
			virtual ~PipeBase();
			PipeBase(bool is_block);
			int sendToRTRawData(const void *data, int size);
			int sendToNrtRawData(const void* data, int size);
			int recvInRTRawData(void* data, int size);
			int recvInNrtRawData(void *data, int size);

			PipeBase(const PipeBase&) = delete;
			PipeBase(PipeBase&&) = delete;

			class Imp;
			std::unique_ptr<Imp> pImp;
		};

		template <typename StandardLayoutStruct>
		class Pipe:private PipeBase
		{
		public:
			Pipe(bool isBlock = true) :PipeBase(isBlock) {};
			int sendToRT(const StandardLayoutStruct &data)
			{
				return sendToRTRawData(static_cast<const void*>(&data), sizeof(data));
			};
			int sendToNrt(const StandardLayoutStruct &data)
			{
				return sendToNrtRawData(static_cast<const void*>(&data), sizeof(data));
			};
			int recvInRT(StandardLayoutStruct &data)
			{
				return recvInRTRawData(static_cast<void*>(&data), sizeof(data));
			};
			int recvInNrt(StandardLayoutStruct &data)
			{
				return recvInNrtRawData(static_cast<void*>(&data), sizeof(data));
			};
		};
		
		template <typename T>
		class Pipe<std::vector<T> > :public PipeBase
		{
		public:
			Pipe(bool is_block, int size):PipeBase(is_block), size(size) {};
			int sendToRT(const std::vector<T> &vec)
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only send fixed size vector");
				return sendToRTRawData(vec.data(), sizeof(T)*size);
			}
			int sendToNrt(const std::vector<T> &vec)
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only send fixed size vector");
				return sendToNrtRawData(vec.data(), sizeof(T)*size);
			}
			int recvInRT(std::vector<T> &vec)
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only recv fixed size vector");
				return recvInRTRawData(vec.data(), sizeof(T)*size);
			}
			int recvInNrt(std::vector<T> &vec)
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only recv fixed size vector");
				return recvInNrtRawData(vec.data(), sizeof(T)*size);
			}

		private:
			int size;
		};

		template <>
		class Pipe<Aris::Core::Msg>:public PipeBase
		{
		public:
			Pipe(bool is_block = true);
			int sendToRT(const Aris::Core::Msg &msg);
			int sendToNrt(const Aris::Core::MsgRT &msg);
			int recvInRT(Aris::Core::MsgRT &msg);
			int recvInNrt(Aris::Core::Msg &msg);
		};
	}
}



















#endif
