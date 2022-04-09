#ifndef ARIS_CORE_MSG_H_
#define ARIS_CORE_MSG_H_

#include <cstdint>
#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <functional>

#include <aris_lib_export.h>

namespace aris::core
{
	using MsgSize = std::uint32_t;
	using MsgID = std::uint32_t;
	using MsgType = std::uint64_t;

	struct ARIS_API MsgHeader{
		MsgSize msg_size_;
		MsgID msg_id_;
		MsgType msg_type_;
		std::int64_t reserved1_;
		std::int64_t reserved2_;
		std::int64_t reserved3_;
	};
	class ARIS_API MsgBase{
	public:
		auto virtual resize(MsgSize size)->void = 0;
		auto virtual header()->MsgHeader& = 0;
		auto virtual header()const->const MsgHeader& = 0;
		auto virtual capacity()const->MsgSize = 0;
		auto empty()const->bool { return size() == 0; }
		auto size() const->MsgSize { return header().msg_size_; }
		auto setType(MsgType msg_type)->void { header().msg_type_ = msg_type; }
		auto type() const->MsgType { return header().msg_type_; }
		auto setMsgID(MsgID msg_id)->void { header().msg_id_ = msg_id; }
		auto msgID() const->MsgID { return header().msg_id_; }
		auto data() const->const char* { return const_cast<MsgBase*>(this)->data(); }
		auto data()->char* { return reinterpret_cast<char*>(&header()) + sizeof(MsgHeader); }
		auto copy(const std::string &str)->void;
		auto copy(const void *src, MsgSize size)->void;
		auto copyAt(const void *src, MsgSize size, MsgSize at_this_pos_of_msg)->void;
		auto copyMore(const void *src, MsgSize size)->void;
		template<class... Args>
		auto copyStruct(const Args&... args)->void
		{
			resize(0);
			copyStructMore(args...);
		}
		template<class FirstArg, class... Args>
		auto copyStructMore(const FirstArg& first_arg, const Args&... args)->void
		{
			copyMore(static_cast<const void*>(&first_arg), sizeof(FirstArg));
			copyStructMore(args...);
		}
		auto copyStructMore()->void {};
		auto paste(void *tar, MsgSize size) const->void;
		auto paste(void *tar) const->void;
		auto pasteAt(void *tar, MsgSize size, MsgSize at_this_pos_of_msg) const->void;
		template<class FirstArg, class... Args>
		auto pasteStruct(FirstArg& first_arg, Args&... args) const->void
		{
			pasteAt(static_cast<void*>(&first_arg), sizeof(FirstArg), paste_id_);
			paste_id_ += sizeof(FirstArg);
			pasteStruct(args...);
		}
		auto pasteStruct() const->void { paste_id_ = 0; }

		auto toString()const->std::string { return std::string(data(), size()); }

	protected:
		virtual ~MsgBase() = default;
		MsgBase() = default;
		MsgBase(const MsgBase &other) = default;
		MsgBase(MsgBase &&other) = default;
		MsgBase& operator=(const MsgBase& other) = default;
		MsgBase& operator=(MsgBase&& other) = default;

	private:
		mutable MsgSize paste_id_{ 0 };
	};
	class ARIS_API Msg final :public MsgBase{
	public:
		auto virtual resize(MsgSize size)->void override;
		auto virtual header()->MsgHeader& override;
		auto virtual header()const->const MsgHeader& override;
		auto virtual capacity()const->MsgSize override { return capacity_; }
		auto swap(Msg &other)->void;

		virtual ~Msg();
		explicit Msg(MsgID msg_id = 0, MsgSize size = 0);
		explicit Msg(const std::string &msg_str);
		Msg(const MsgBase &other);
		Msg(const Msg& other);
		Msg(Msg&& other)noexcept;
		Msg& operator=(Msg &&other)noexcept;

	private:
		std::unique_ptr<char[]> data_;
		MsgSize capacity_;
	};
	template<std::size_t CAPACITY>
	class MsgFix final :public MsgBase
	{
	public:
		auto virtual resize(MsgSize size)->void override { header().msg_size_ = size; };
		auto virtual header()->MsgHeader& override { return *reinterpret_cast<MsgHeader*>(data_); };
		auto virtual header()const->const MsgHeader& override { return *reinterpret_cast<const MsgHeader*>(data_); };
		auto virtual capacity()const->MsgSize override { return CAPACITY; }

		virtual ~MsgFix() = default;
		explicit MsgFix(MsgID msg_id = 0, MsgSize size = 0) :MsgBase() { resize(size); setMsgID(msg_id); }
		MsgFix(const MsgBase &other)
		{
			resize(other.size());
			std::copy_n(reinterpret_cast<const char*>(&other.header()), other.size() + sizeof(MsgHeader), reinterpret_cast<char*>(&header()));
		}
		MsgFix(const MsgFix &other) = default;
		MsgFix(MsgFix &&other) = default;
		MsgFix &operator=(const MsgFix& other) = default;
		MsgFix &operator=(MsgFix&& other) = default;

	private:
		char data_[CAPACITY + sizeof(MsgHeader)];
	};
	class ARIS_API MsgStreamBuf :public std::streambuf{
	public:
		explicit MsgStreamBuf(MsgBase& msg);
		auto reset()->void;
	protected:
		virtual auto overflow(int_type c)->int_type override;
		virtual auto sync()->int override;

	private:
		MsgBase * msg_;
	};
	class ARIS_API MsgStream : public std::iostream{
	public:
		auto reset()->void { buf.reset(); }
		explicit MsgStream(MsgBase& msg) : buf(msg), std::iostream(&buf) { }

	private:
		MsgStreamBuf buf;
	};

	template<typename Function, typename ...Args>
	auto benchmark(std::size_t count, Function func, Args&&... args)->double
	{
		auto begin_time = std::chrono::high_resolution_clock::now();
		for (std::size_t i = 0; i < count; ++i)func(std::forward<Args>(args)...);
		auto end_time = std::chrono::high_resolution_clock::now();

		return double((end_time - begin_time).count()) / 1e9 / count;
	};
}

#endif
