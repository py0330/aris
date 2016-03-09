#ifndef ARIS_CORE_MSG_H_
#define ARIS_CORE_MSG_H_

#include <cstring>
#include <cstdint>
#include <cstdio>
#include <string>


/// \defgroup Aris
///
///

/// \brief 总命名空间。
/// \ingroup Aris
/// 
///
///
namespace Aris
{
	namespace Control
	{
		template<typename T> class Pipe;
	}
	
	/// \brief Aris核心命名空间
	/// \ingroup Aris
	/// 
	///
	///
	namespace Core
	{
		class Socket;
		class Msg;
		class MsgRT;

		struct MsgHeader
		{
			std::int32_t msg_size;
			std::int32_t msg_id;
			std::int64_t msg_type;
			std::int64_t reserved1;
			std::int64_t reserved2;
			std::int64_t reserved3;
		};
		class MsgBase
		{
		public:
			virtual ~MsgBase() = default;
			/** \brief 设置Msg中所包含的数据的长度
			* \param dataLength    数据长度
			*/
			virtual auto resize(std::int32_t size)->void = 0;
			/** \brief 获取Msg中所包含的数据的长度
			*
			*/
			auto size() const->std::int32_t;
			/** \brief 设置Msg的ID，在消息循环中根据该ID来查找对应的消息回调函数
			* \param msgID   Msg的ID
			*/
			auto setMsgID(std::int32_t id)->void;
			/** \brief 获取Msg中的ID
			*
			*/
			auto msgID() const->std::int32_t;
			/** \brief 获取Msg中的数据地址
			*
			*/
			auto data() const->const char*;
			/** \brief 获取Msg中的数据地址
			*
			*/
			auto data()->char*;
			/** \brief 从fromThisMemory指针中拷贝字符串。
			* \param fromThisMemory    待拷贝的内存地址。
			* \param dataLength        数据长度
			*
			*/
			auto copy(const char * from_this_memory)->void;
			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，在拷贝完之后，Msg的长度自动设置为dataLength。
			* \param fromThisMemory    待拷贝的内存地址。
			* \param dataLength        数据长度
			*
			*/
			auto copy(const void * from_this_memory, std::int32_t size)->void;
			/** \brief 从fromThisMemory指针中拷贝Msg.GetLength()大小的数据。
			* \param fromThisMemory    待拷贝的内存地址。
			*
			*/
			auto copy(const void * from_this_memory)->void;
			/** \brief 从fromThisMemory指针中拷贝Msg.GetLength()大小的数据到Msg内存中的指定地点。
			* \param fromThisMemory       待拷贝的内存地址。
			* \param dataLength           数据长度
			* \param atThisPositionInMsg  将数据拷贝到Msg.data()[atThisPositionInMsg]处。
			*/
			auto copyAt(const void * from_this_memory, std::int32_t size, std::int32_t at_this_pos_of_msg)->void;
			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，这些数据添加到自身的尾部，在拷贝完之后，Msg的长度自动增加dataLength。
			* \param fromThisMemory    目标内存地址。
			*
			*/
			auto copyMore(const void * from_this_memory, std::int32_t size)->void;
			/** \brief 向toThisMemory指针中粘贴dataLength长度的数据，若dataLength大于自身的数据长度，则只拷贝自身长度的内存。
			* \param fromThisMemory    目标内存地址。
			* \param dataLength        数据长度
			*
			*/
			auto paste(void * to_this_memory, std::int32_t size) const->void;
			/** \brief 向toThisMemory指针中粘贴Msg.GetLength()长度的数据。
			* \param fromThisMemory    目标内存地址。
			*
			*/
			auto paste(void * to_this_memory) const->void;
			/** \brief 向toThisMemory指针中粘贴dataLength长度的数据，若dataLength大于自身的数据长度，则只拷贝自身长度的内存。
			* \param fromThisMemory    目标内存地址。
			* \param dataLength        数据长度
			*
			*/
			auto pasteAt(void * to_this_memory, std::int32_t size, std::int32_t at_this_pos_of_msg) const->void;

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
			auto getType() const->std::int64_t;

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
			template<typename T> friend class Aris::Control::Pipe;
		};
		class Msg final :public MsgBase
		{
		public:
			/** \brief Destructor
			*
			*/
			virtual ~Msg();
			/** \brief Copy Constructor
			* \param other    another message
			*/
			Msg(const Msg& other);
			/** \brief Move Constructor
			* \param other    another message
			*/
			Msg(Msg&& other);
			/** \brief Default Constructor
			* \param length   The length of message(not count message header), the unit of which is byte.
			* \param msgID  The ID of message
			*/
			explicit Msg(std::int32_t msg_id = 0, std::int32_t size = 0);
			/** \brief Assignment Operator, which is deep copy from another message
			* \param other    another message
			*/
			auto operator=(Msg other)->Msg &;
			/** \brief 跟另外一个Msg对象交换数据。仅仅改变双方指针，因此效率高于任何一个构造函数。
			* \param other    另外一个消息。
			*
			*/
			auto swap(Msg &other)->void;
			/** \brief Set msg length
			*
			*/
			virtual auto resize(std::int32_t size)->void;

		private:
			friend class Socket;
		};
		class MsgRT final :public MsgBase
		{
		public:
			enum { RT_MSG_LENGTH = 8192 };
			static MsgRT instance[2];

			virtual auto resize(std::int32_t size)->void;

		private:
			MsgRT();
			virtual ~MsgRT();
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
