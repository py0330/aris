#ifndef ARIS_CORE_PIPE_H
#define ARIS_CORE_PIPE_H

#include <aris_lib_export.h>
#include <aris/core/object.hpp>
#include <aris/core/msg.hpp>

namespace aris::core
{
	class ARIS_API Pipe
	{
	public:
		auto sendMsg(const aris::core::MsgBase &)->bool;
		auto recvMsg(aris::core::MsgBase &)->bool;
		auto resize(Size mem_pool_size);
		auto size()->Size;

		virtual ~Pipe();
		Pipe(const std::string &name = "pipe", std::size_t pool_size = 16384);
		Pipe(const Pipe&) = delete;
		Pipe(Pipe&&);
		Pipe& operator=(const Pipe&) = delete;
		Pipe& operator=(Pipe&&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif
