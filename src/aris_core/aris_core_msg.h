#ifndef ARIS_CORE_MSG_H_
#define ARIS_CORE_MSG_H_

#include <cstring>
#include <cstdint>
#include <cstdio>
#include <string>
#include <chrono>
#include <memory>

/// \defgroup aris
///
///

/// \brief 总命名空间。
/// \ingroup aris
/// 
///
///
namespace aris
{
	namespace control
	{
		template<typename T> class Pipe;
	}
	
	/// \brief aris核心命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace core
	{
		class Socket;
		class Msg;
		class MsgRT;

		struct MsgHeader
		{
			std::int32_t msg_size_;
			std::int32_t msg_id_;
			std::int64_t msg_type_;
			std::int64_t reserved1_;
			std::int64_t reserved2_;
			std::int64_t reserved3_;
		};
		class MsgBase
		{
		public:
			virtual ~MsgBase() = default;
			virtual auto resize(std::int32_t size)->void = 0;
			auto size() const->std::int32_t;
			auto setMsgID(std::int32_t id)->void;
			auto msgID() const->std::int32_t;
			auto data() const->const char*;
			auto data()->char*;
			auto copy(const char *src)->void;
			auto copy(const void *src, std::int32_t size)->void;
			auto copy(const void *src)->void;
			auto copyAt(const void *src, std::int32_t size, std::int32_t at_this_pos_of_msg)->void;
			auto copyMore(const void *src, std::int32_t size)->void;
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
			auto paste(void *tar, std::int32_t size) const->void;
			auto paste(void *tar) const->void;
			auto pasteAt(void *tar, std::int32_t size, std::int32_t at_this_pos_of_msg) const->void;
			template<class FirstArg, class... Args>
			auto pasteStruct(FirstArg& first_arg, Args&... args) const->void
			{
				pasteAt(static_cast<void*>(&first_arg), sizeof(FirstArg), paste_id_);
				paste_id_ += sizeof(FirstArg);
				pasteStruct(args...);
			}
			auto pasteStruct() const->void { paste_id_ = 0; }

		private:
			auto setType(std::int64_t type)->void;
			auto type() const->std::int64_t;
			virtual auto header()->MsgHeader& = 0;
			virtual auto header()const->const MsgHeader& = 0;

		private:
			MsgBase() = default;
			MsgBase(const MsgBase &other) = default;
			MsgBase(MsgBase &&other) = default;
			MsgBase &operator=(const MsgBase& other) = default;
			MsgBase &operator=(MsgBase&& other) = default;

		private:
			mutable std::int32_t paste_id_{ 0 };

			friend class Msg;
			friend class MsgRT;
			template<std::size_t CAPACITY> friend class MsgFix;
			friend class Socket;
			friend class Pipe;
			template<typename T> friend class aris::control::Pipe;
		};
		class Msg final :public MsgBase
		{
		public:
			virtual auto resize(std::int32_t size)->void override;
			auto swap(Msg &other)->void;
			
			virtual ~Msg();
			explicit Msg(std::int32_t msg_id = 0, std::int32_t size = 0);
			explicit Msg(const std::string &msg_str);
			Msg(const Msg& other);
			Msg(Msg&& other);
			Msg& operator=(Msg other);

		private:
			virtual auto header()->MsgHeader& override;
			virtual auto header()const->const MsgHeader& override;

			std::unique_ptr<char[]> data_;

			friend class Socket;
			template<typename T> friend class aris::control::Pipe;
		};

		class MsgRT final :public MsgBase
		{
		public:
			enum { RT_MSG_SIZE = 8192 };
			enum { RT_MSG_NUM = 2 };
			using MsgRtArray = MsgRT[RT_MSG_NUM];

			static auto instance()->MsgRtArray&;
			virtual auto resize(std::int32_t size)->void override;

		private:

			virtual ~MsgRT();
			MsgRT();
			MsgRT(const MsgRT &other) = delete;
			MsgRT(MsgRT &&other) = delete;
			MsgRT &operator=(const MsgRT& other) = delete;
			MsgRT &operator=(MsgRT&& other) = delete;

			virtual auto header()->MsgHeader& override;
			virtual auto header()const->const MsgHeader& override;

			char data_[RT_MSG_SIZE];

			friend class Socket;
			template<typename T> friend class aris::control::Pipe;
		};

		template<std::size_t CAPACITY>
		class MsgFix final :public MsgBase
		{
		public:
			virtual auto resize(std::int32_t size)->void override { header().msg_size_ = size; };

			virtual ~MsgFix() = default;
			explicit MsgFix(std::int32_t msg_id = 0, std::int32_t size = 0) :MsgBase() {}
			explicit MsgFix(const std::string &msg_str) :MsgBase(msg_str) {}
			MsgFix(const MsgFix &other) = default;
			MsgFix(MsgFix &&other) = default;
			MsgFix &operator=(const MsgFix& other) = default;
			MsgFix &operator=(MsgFix&& other) = default;

		private:
			virtual auto header()->MsgHeader& override { return *reinterpret_cast<MsgHeader*>(data_); };
			virtual auto header()const->const MsgHeader& override{ return *reinterpret_cast<const MsgHeader*>(data_); };

			char data_[CAPACITY + sizeof(MsgHeader)];

			friend class Socket;
			template<typename T> friend class aris::control::Pipe;
		};

		auto createLogDir()->void;
		auto logExeName()->std::string;
		auto logFileTimeFormat(const std::chrono::system_clock::time_point &time)->std::string;
		auto logDirPath()->std::string;
		auto logFileName()->const std::string&;
		auto log(const char *data)->const char *;
		auto log(const std::string& data)->const std::string&;

	}
}


#endif
