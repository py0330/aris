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
			std::size_t pool_size_;
			std::unique_ptr<char[]> pool_;
			std::atomic_size_t send_pos_{ 0 }, recv_pos_{ 0 };
		};
		auto Pipe::sendMsg(const aris::core::MsgBase &msg)->bool
		{
			std::size_t remain_data_size = ((imp_->send_pos_ - imp_->recv_pos_) % imp_->pool_size_ + imp_->pool_size_) % imp_->pool_size_;
			if (remain_data_size + sizeof(MsgHeader) + msg.size()> imp_->pool_size_) return false;
			std::size_t send_num1 = imp_->send_pos_ + sizeof(MsgHeader) + msg.size() > imp_->pool_size_ ? imp_->pool_size_ - imp_->send_pos_ : sizeof(MsgHeader) + msg.size();
			std::size_t send_num2 = sizeof(MsgHeader) + msg.size() - send_num1;
			std::copy_n(reinterpret_cast<const char *>(&msg.header()), send_num1, &imp_->pool_[imp_->send_pos_]);
			std::copy_n(reinterpret_cast<const char *>(&msg.header()) + send_num1, send_num2, &imp_->pool_[0]);
			imp_->send_pos_ = (imp_->send_pos_ + msg.size() + sizeof(MsgHeader)) % imp_->pool_size_;
			return true;
		}
		auto Pipe::recvMsg(aris::core::MsgBase &msg)->bool
		{
			MsgHeader header;
			if (imp_->send_pos_ == imp_->recv_pos_) return false;
			std::size_t recv_num1 = imp_->recv_pos_ + sizeof(MsgHeader) > imp_->pool_size_ ? imp_->pool_size_ - imp_->recv_pos_ : sizeof(MsgHeader);
			std::size_t recv_num2 = sizeof(MsgHeader) - recv_num1;
			std::copy_n(&imp_->pool_[imp_->recv_pos_], recv_num1, reinterpret_cast<char *>(&header));
			std::copy_n(&imp_->pool_[0], recv_num2, reinterpret_cast<char *>(&header) + recv_num1);
			msg.resize(header.msg_size_);
			recv_num1 = imp_->recv_pos_ + sizeof(MsgHeader) + msg.size() > imp_->pool_size_ ? imp_->pool_size_ - imp_->recv_pos_ : sizeof(MsgHeader) + msg.size();
			recv_num2 = sizeof(MsgHeader) + msg.size() - recv_num1;
			std::copy_n(&imp_->pool_[imp_->recv_pos_], recv_num1, reinterpret_cast<char *>(&msg.header()));
			std::copy_n(&imp_->pool_[0], recv_num2, reinterpret_cast<char *>(&msg.header()) + recv_num1);
			imp_->recv_pos_ = (imp_->recv_pos_ + msg.size() + sizeof(MsgHeader)) % imp_->pool_size_;
			return true;
		}
		Pipe::~Pipe() = default;
		Pipe::Pipe(const std::string &name, bool is_block, std::size_t pool_size) :Object(name), imp_(new Imp)
		{
			imp_->is_block_ = is_block;
			imp_->pool_size_ = pool_size;
			imp_->pool_.reset(new char[imp_->pool_size_]());
		}
		Pipe::Pipe(Object &father, const aris::core::XmlElement &xml_ele) : Object(father, xml_ele), imp_(new Imp)
		{
			imp_->is_block_ = attributeBool(xml_ele, "is_block", true);
			imp_->pool_size_ = attributeInt32(xml_ele, "pool_size", 16384);
			imp_->pool_.reset(new char[imp_->pool_size_]());
		}
		Pipe::Pipe(Pipe&&) = default;
		Pipe& Pipe::operator=(Pipe&&) = default;
	}
}
