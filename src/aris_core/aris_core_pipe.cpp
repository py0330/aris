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
			std::size_t pool_size_;
			std::unique_ptr<char[]> pool_;
			std::atomic_size_t send_pos_{ 0 }, recv_pos_{ 0 };
		};
		auto Pipe::sendMsg(const aris::core::MsgBase &msg)->bool
		{
			auto send_pos = imp_->send_pos_.load();
			auto recv_pos = imp_->recv_pos_.load();
			
			std::size_t remain_data_size = ((send_pos - recv_pos) % imp_->pool_size_ + imp_->pool_size_) % imp_->pool_size_;
			if (remain_data_size + sizeof(MsgHeader) + msg.size()> imp_->pool_size_) return false;
			std::size_t send_num1 = send_pos + sizeof(MsgHeader) + msg.size() > imp_->pool_size_ ? imp_->pool_size_ - send_pos : sizeof(MsgHeader) + msg.size();
			std::size_t send_num2 = sizeof(MsgHeader) + msg.size() - send_num1;
			std::copy_n(reinterpret_cast<const char *>(&msg.header()), send_num1, &imp_->pool_[send_pos]);
			std::copy_n(reinterpret_cast<const char *>(&msg.header()) + send_num1, send_num2, &imp_->pool_[0]);
			imp_->send_pos_.store( (send_pos + msg.size() + sizeof(MsgHeader)) % imp_->pool_size_);
			return true;
		}
		auto Pipe::recvMsg(aris::core::MsgBase &msg)->bool
		{
			auto send_pos = imp_->send_pos_.load();
			auto recv_pos = imp_->recv_pos_.load();
			
			MsgHeader header;
			if (send_pos == recv_pos) return false;
			std::size_t recv_num1 = recv_pos + sizeof(MsgHeader) > imp_->pool_size_ ? imp_->pool_size_ - recv_pos : sizeof(MsgHeader);
			std::size_t recv_num2 = sizeof(MsgHeader) - recv_num1;
			std::copy_n(&imp_->pool_[recv_pos], recv_num1, reinterpret_cast<char *>(&header));
			std::copy_n(&imp_->pool_[0], recv_num2, reinterpret_cast<char *>(&header) + recv_num1);
			msg.resize(header.msg_size_);
			recv_num1 = recv_pos + sizeof(MsgHeader) + msg.size() > imp_->pool_size_ ? imp_->pool_size_ - recv_pos : sizeof(MsgHeader) + msg.size();
			recv_num2 = sizeof(MsgHeader) + msg.size() - recv_num1;
			std::copy_n(&imp_->pool_[recv_pos], recv_num1, reinterpret_cast<char *>(&msg.header()));
			std::copy_n(&imp_->pool_[0], recv_num2, reinterpret_cast<char *>(&msg.header()) + recv_num1);
			imp_->recv_pos_.store((recv_pos + msg.size() + sizeof(MsgHeader)) % imp_->pool_size_);
			return true;
		}
		Pipe::~Pipe() = default;
		Pipe::Pipe(const std::string &name, std::size_t pool_size) :Object(name), imp_(new Imp)
		{
			imp_->pool_size_ = pool_size;
			imp_->pool_.reset(new char[imp_->pool_size_]());
		}
		Pipe::Pipe(Object &father, const aris::core::XmlElement &xml_ele) : Object(father, xml_ele), imp_(new Imp)
		{
			imp_->pool_size_ = attributeInt32(xml_ele, "pool_size", 16384);
			imp_->pool_.reset(new char[imp_->pool_size_]());
		}
		Pipe::Pipe(Pipe&&) = default;
		Pipe& Pipe::operator=(Pipe&&) = default;
	}
}
