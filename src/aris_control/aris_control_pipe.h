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
			auto sendToRTRawData(const void *data, int size)->int;
			auto sendToNrtRawData(const void* data, int size)->int;
			auto recvInRTRawData(void* data, int size)->int;
			auto recvInNrtRawData(void *data, int size)->int;

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
			auto sendToRT(const StandardLayoutStruct &data)->int
			{
				return sendToRTRawData(static_cast<const void*>(&data), sizeof(data));
			};
			auto sendToNrt(const StandardLayoutStruct &data)->int
			{
				return sendToNrtRawData(static_cast<const void*>(&data), sizeof(data));
			};
			auto recvInRT(StandardLayoutStruct &data)->int
			{
				return recvInRTRawData(static_cast<void*>(&data), sizeof(data));
			};
			auto recvInNrt(StandardLayoutStruct &data)->int
			{
				return recvInNrtRawData(static_cast<void*>(&data), sizeof(data));
			};
		};
		
		template <typename T>
		class Pipe<std::vector<T> > :public PipeBase
		{
		public:
			Pipe(bool is_block, int size):PipeBase(is_block), size(size) {};
			auto sendToRT(const std::vector<T> &vec)->int
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only send fixed size vector");
				return sendToRTRawData(vec.data(), sizeof(T)*size);
			}
			auto sendToNrt(const std::vector<T> &vec)->int
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only send fixed size vector");
				return sendToNrtRawData(vec.data(), sizeof(T)*size);
			}
			auto recvInRT(std::vector<T> &vec)->int
			{
				if (vec.size() != this->size)throw std::runtime_error("this pipe can only recv fixed size vector");
				return recvInRTRawData(vec.data(), sizeof(T)*size);
			}
			auto recvInNrt(std::vector<T> &vec)->int
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
			auto sendToRT(const Aris::Core::Msg &msg)->int;
			auto sendToNrt(const Aris::Core::MsgRT &msg)->int;
			auto recvInRT(Aris::Core::MsgRT &msg)->int;
			auto recvInNrt(Aris::Core::Msg &msg)->int;
		};
	}
}



















#endif
