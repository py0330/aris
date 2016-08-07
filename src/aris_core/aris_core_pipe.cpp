#include <mutex>
#include <string>
#include <iostream>
#include <memory>
#include <atomic>

#include "aris_core_pipe.h"


namespace aris
{
	namespace core
	{
		struct Pipe::Imp
		{
			bool is_block_;
			std::unique_ptr<char[]> pool_;
			std::atomic_size_t send_pos_{ 0 }, recv_pos_{ 0 };
		};
		auto Pipe::sendMsg(const aris::core::MsgBase &msg)->void
		{
			//if(msg.size()+sizeof(aris::core::MsgHeader))
		}
		auto Pipe::recvMsg(aris::core::MsgBase &msg)->void
		{

		}
		Pipe::~Pipe() = default;
		Pipe::Pipe(Object &father, std::size_t id, const std::string &name, bool is_block, std::size_t pool_size) :Object(father, id, name), imp_(new Imp)
		{
			imp_->is_block_ = is_block;
			imp_->pool_.reset(new char[pool_size]());
		}
		Pipe::Pipe(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) : Object(father, id, xml_ele), imp_(new Imp)
		{
			imp_->is_block_ = attributeBool(xml_ele, "is_block", true);
			imp_->pool_.reset(new char[attributeInt32(xml_ele, "pool_size", 16384)]());
		}
		Pipe::Pipe() = default;
		Pipe::Pipe(Pipe&&) = default;
		Pipe& Pipe::operator=(Pipe&&) = default;
	}
}
