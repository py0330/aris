#include <cstring>
#include <fstream>
#include <ctime>
#include <mutex>
#include <algorithm>
#include <iostream>

#include "aris_core_msg.h"

namespace aris
{
	namespace core
	{
		auto MsgBase::copy(const char *src)->void { copy(static_cast<const void *>(src), static_cast<MsgSize>(strlen(src) + 1)); }
		auto MsgBase::copy(const void *src, MsgSize data_size)->void { resize(data_size); copyAt(src, data_size, 0); }
		auto MsgBase::copyAt(const void *src, MsgSize data_size, MsgSize at_this_pos_of_msg)->void
		{
			if ((data_size + at_this_pos_of_msg) > size())resize(data_size + at_this_pos_of_msg);
			std::copy_n(static_cast<const char *>(src), data_size, data() + at_this_pos_of_msg);
		}
		auto MsgBase::copyMore(const void *src, MsgSize data_size)->void{ copyAt(src, data_size, size()); }
		auto MsgBase::paste(void *tar, MsgSize data_size) const->void { std::copy_n(data(), std::min(size(), data_size), static_cast<char*>(tar)); }
		auto MsgBase::paste(void *tar) const->void{ std::copy_n(data(), size(), static_cast<char*>(tar)); }
		auto MsgBase::pasteAt(void *tar, MsgSize data_size, MsgSize at_this_pos_of_msg) const->void
		{
			std::copy_n(data() + at_this_pos_of_msg, std::min(data_size, size() - at_this_pos_of_msg), static_cast<char*>(tar));
		}

		auto Msg::swap(Msg &other)->void { std::swap(data_, other.data_); std::swap(capacity_, other.capacity_); }
		auto Msg::resize(MsgSize data_size)->void
		{
			if (capacity_ < data_size)
			{
				capacity_ = std::max(data_size, 2 * capacity_);
				std::unique_ptr<char[]> other(new char[sizeof(MsgHeader) + capacity_]());
				std::copy_n(data_.get(), sizeof(MsgHeader) + size(), other.get());
				std::swap(data_, other);
			}

			header().msg_size_ = data_size;
		}
		auto Msg::header()->MsgHeader& { return *reinterpret_cast<MsgHeader*>(data_.get()); }
		auto Msg::header()const->const MsgHeader& { return *reinterpret_cast<const MsgHeader*>(data_.get()); }
		Msg::~Msg() = default;
		Msg::Msg(MsgID msg_id, MsgSize size) :data_(std::make_unique<char[]>(sizeof(MsgHeader) + size)), capacity_(size)
		{ 
			header().msg_id_ = 0;
			header().msg_size_ = size;
			header().msg_type_ = 0;
			header().reserved1_ = 0;
			header().reserved2_ = 0;
			header().reserved3_ = 0;
		}
		Msg::Msg(const std::string &str) :data_(std::make_unique<char[]>(sizeof(MsgHeader) + str.size() + 1)), capacity_(str.size() + 1)
		{ 
			std::copy(str.begin(), str.end(), data());
			data()[str.size()] = '\0';
			header().msg_id_ = 0;
			header().msg_size_ = str.size() + 1;
			header().msg_type_ = 0;
			header().reserved1_ = 0;
			header().reserved2_ = 0;
			header().reserved3_ = 0;
		}
		Msg::Msg(const MsgBase &other) : data_(std::make_unique<char[]>(sizeof(MsgHeader) + other.size())), capacity_(other.size())
		{
			std::copy_n(reinterpret_cast<const char*>(&other.header()), other.size() + sizeof(MsgHeader), reinterpret_cast<char*>(&header()));
		}
		Msg::Msg(const Msg& other) : data_(std::make_unique<char[]>(sizeof(MsgHeader) + other.size())), capacity_(other.size())
		{
			std::copy_n(other.data_.get(), sizeof(MsgHeader) + other.size(), data_.get());
		}
		Msg::Msg(Msg&& other) { swap(other); }
		Msg& Msg::operator=(Msg &&other) { swap(other); return (*this); }

		auto MsgStreamBuf::overflow(int_type c)->int_type
		{
			update();

			msg_->resize(msg_->size() + 1);
			resetBuf();
			if (msg_->capacity() < msg_->size())
			{
				return traits_type::eof();
			}
			else
			{
				*(pptr() - 1) = c;
				return c;
			}
		}
		auto MsgStreamBuf::underflow()->int_type
		{
			update();
			
			if (gptr() == pptr()) return traits_type::eof();
			else return *(reinterpret_cast<char*>(&msg_->header()) + sizeof(MsgHeader));
		}
		auto MsgStreamBuf::update()->void
		{
			auto msg_buf = reinterpret_cast<char*>(&msg_->header()) + sizeof(MsgHeader);
			if (gptr() != msg_buf)std::copy(gptr(), pptr(), msg_buf);
			msg_->resize(static_cast<MsgSize>(pptr() - gptr()));
			resetBuf();
		}
		auto MsgStreamBuf::resetBuf()->void
		{
			auto msg_buf = reinterpret_cast<char*>(&msg_->header()) + sizeof(MsgHeader);
			setp(msg_buf + msg_->size(), msg_buf + msg_->capacity());
			setg(msg_buf, msg_buf, msg_buf + msg_->size());
		}
		MsgStreamBuf::MsgStreamBuf(MsgBase& msg) :msg_(&msg), std::streambuf(){ resetBuf();	};
	}
}
