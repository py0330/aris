#ifndef ARIS_CORE_MSG_H_
#define ARIS_CORE_MSG_H_

#include <cstring>
#include <cstdint>
#include <cstdio>
#include <string>


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
			auto copy(const char * from_this_memory)->void;
			auto copy(const void * from_this_memory, std::int32_t size)->void;
			auto copy(const void * from_this_memory)->void;
			auto copyAt(const void * from_this_memory, std::int32_t size, std::int32_t at_this_pos_of_msg)->void;
			auto copyMore(const void * from_this_memory, std::int32_t size)->void;
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
			auto paste(void * to_this_memory, std::int32_t size) const->void;
			auto paste(void * to_this_memory) const->void;
			auto pasteAt(void * to_this_memory, std::int32_t size, std::int32_t at_this_pos_of_msg) const->void;
			template<class FirstArg, class... Args>
			auto pasteStruct(FirstArg& first_arg, Args&... args) const->void
			{
				pasteAt(static_cast<void*>(&first_arg), sizeof(FirstArg), paste_id_);
				paste_id_ += sizeof(FirstArg);
				pasteStruct(args...);
			}
			auto pasteStruct() const->void { paste_id_ = 0; }

		private:
			auto header()->MsgHeader&;
			auto header()const->const MsgHeader&;
			auto setType(std::int64_t type)->void;
			auto type() const->std::int64_t;

		private:
			MsgBase() = default;
			MsgBase(const MsgBase &other) = delete;
			MsgBase(MsgBase &&other) = delete;
			MsgBase &operator=(const MsgBase& other) = delete;
			MsgBase &operator=(MsgBase&& other) = delete;

		private:
			mutable std::int32_t paste_id_{ 0 };
			char *data_{ nullptr };

			friend class Msg;
			friend class MsgRT;
			friend class Socket;
			friend class Socket;
			template<typename T> friend class aris::control::Pipe;
		};
		class Msg final :public MsgBase
		{
		public:
			virtual auto resize(std::int32_t size)->void;
			auto swap(Msg &other)->void;
			
			virtual ~Msg();
			explicit Msg(std::int32_t msg_id = 0, std::int32_t size = 0);
			explicit Msg(const std::string &msg_str);
			Msg(const Msg& other);
			Msg(Msg&& other);
			Msg& operator=(Msg other);

		private:
			friend class Socket;
		};
		class MsgRT final :public MsgBase
		{
		public:
			enum { RT_MSG_SIZE = 8192 };
			enum { RT_MSG_NUM = 2 };
			using MsgRtArray = MsgRT[RT_MSG_NUM];

			static auto instance()->MsgRtArray&;
			virtual auto resize(std::int32_t size)->void;

		private:
			virtual ~MsgRT();
			MsgRT();
			MsgRT(const MsgRT &other) = delete;
			MsgRT(MsgRT &&other) = delete;
			MsgRT &operator=(const MsgRT& other) = delete;
			MsgRT &operator=(MsgRT&& other) = delete;
		};

		auto logFileName()->const std::string&;
		auto log(const char *data)->const char *;
		auto log(const std::string& data)->const std::string&;

		auto msSleep(int miliseconds)->void;

	}
}


#endif
