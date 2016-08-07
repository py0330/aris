#ifndef ARIS_CORE_PIPE_H
#define ARIS_CORE_PIPE_H

#include <memory>
#include <cstdint>

#include <aris_core_xml.h>
#include <aris_core_msg.h>

namespace aris
{
	namespace core
	{	
		class Pipe:public aris::core::Object
		{
		public:
			auto sendMsg(const aris::core::MsgBase &)->bool;
			auto recvMsg(aris::core::MsgBase &)->bool;


			virtual ~Pipe();
			Pipe(const std::string &name = "pipe", bool is_block = true, std::size_t pool_size = 16384);
			Pipe(Object &father, const aris::core::XmlElement &xml_ele);
			Pipe(const Pipe&) = delete;
			Pipe(Pipe&&);
			Pipe& operator=(const Pipe&) = delete;
			Pipe& operator=(Pipe&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;


		};
	}
}



















#endif
