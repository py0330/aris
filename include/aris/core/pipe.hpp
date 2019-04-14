#ifndef ARIS_CORE_PIPE_H
#define ARIS_CORE_PIPE_H

#include <memory>
#include <cstdint>

#include <aris/core/object.hpp>
#include <aris/core/msg.hpp>

namespace aris::core
{
	class Pipe :public aris::core::Object
	{
	public:
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto sendMsg(const aris::core::MsgBase &)->bool;
		auto recvMsg(aris::core::MsgBase &)->bool;

		virtual ~Pipe();
		Pipe(const std::string &name = "pipe", std::size_t pool_size = 16384);
		Pipe(const Pipe&) = delete;
		Pipe(Pipe&&);
		Pipe& operator=(const Pipe&) = delete;
		Pipe& operator=(Pipe&&);
		ARIS_REGISTER_TYPE(Pipe);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif
