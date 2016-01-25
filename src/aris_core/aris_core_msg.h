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
			virtual void resize(std::int32_t size) = 0;
			/** \brief 获取Msg中所包含的数据的长度
			*
			*/
			std::int32_t size() const;
			/** \brief 设置Msg的ID，在消息循环中根据该ID来查找对应的消息回调函数
			* \param msgID   Msg的ID
			*/
			void setMsgID(std::int32_t id);
			/** \brief 获取Msg中的ID
			*
			*/
			std::int32_t msgID() const;
			/** \brief 获取Msg中的数据地址
			*
			*/
			const char* data() const;
			/** \brief 获取Msg中的数据地址
			*
			*/
			char* data();
			/** \brief 从fromThisMemory指针中拷贝字符串。
			* \param fromThisMemory    待拷贝的内存地址。
			* \param dataLength        数据长度
			*
			*/
			void copy(const char * from_this_memory);
			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，在拷贝完之后，Msg的长度自动设置为dataLength。
			* \param fromThisMemory    待拷贝的内存地址。
			* \param dataLength        数据长度
			*
			*/
			void copy(const void * from_this_memory, std::int32_t size);
			/** \brief 从fromThisMemory指针中拷贝Msg.GetLength()大小的数据。
			* \param fromThisMemory    待拷贝的内存地址。
			*
			*/
			void copy(const void * from_this_memory);
			/** \brief 从fromThisMemory指针中拷贝Msg.GetLength()大小的数据到Msg内存中的指定地点。
			* \param fromThisMemory       待拷贝的内存地址。
			* \param dataLength           数据长度
			* \param atThisPositionInMsg  将数据拷贝到Msg.data()[atThisPositionInMsg]处。
			*/
			void copyAt(const void * from_this_memory, std::int32_t size, std::int32_t at_this_pos_of_msg);
			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，这些数据添加到自身的尾部，在拷贝完之后，Msg的长度自动增加dataLength。
			* \param fromThisMemory    目标内存地址。
			*
			*/
			void copyMore(const void * from_this_memory, std::int32_t size);
			/** \brief 向toThisMemory指针中粘贴dataLength长度的数据，若dataLength大于自身的数据长度，则只拷贝自身长度的内存。
			* \param fromThisMemory    目标内存地址。
			* \param dataLength        数据长度
			*
			*/
			void paste(void * to_this_memory, std::int32_t size) const;
			/** \brief 向toThisMemory指针中粘贴Msg.GetLength()长度的数据。
			* \param fromThisMemory    目标内存地址。
			*
			*/
			void paste(void * to_this_memory) const;
			/** \brief 向toThisMemory指针中粘贴dataLength长度的数据，若dataLength大于自身的数据长度，则只拷贝自身长度的内存。
			* \param fromThisMemory    目标内存地址。
			* \param dataLength        数据长度
			*
			*/
			void pasteAt(void * to_this_memory, std::int32_t size, std::int32_t at_this_pos_of_msg) const;
			/** \brief 默认析构函数
			*
			*/
			

			template<class... Args>
			void copyStruct(const Args&... args)
			{
				resize(0);
				copyStructMore(args...);
			}

			template<class FirstArg, class... Args>
			void copyStructMore(const FirstArg& first_arg, const Args&... args)
			{
				copyMore(static_cast<const void*>(&first_arg), sizeof(FirstArg));
				copyStructMore(args...);
			}

			template<class FirstArg, class... Args>
			void pasteStruct(FirstArg& first_arg, Args&... args) const
			{
				pasteAt(static_cast<void*>(&first_arg), sizeof(FirstArg), pasteID);
				pasteID += sizeof(FirstArg);
				pasteStruct(args...);
			}

		private:
			void setType(std::int64_t type);
			std::int64_t GetType() const;
			void copyStructMore()
			{
			}
			void pasteStruct() const
			{
				paste_id = 0;
			}

		private:
			MsgBase() = default;
			MsgBase(const MsgBase &other) = delete;
			MsgBase(MsgBase &&other) = delete;
			MsgBase &operator=(const MsgBase& other) = delete;
			MsgBase &operator=(MsgBase&& other) = delete;

		private:
			mutable std::int32_t paste_id{ 0 };
			char *_pData{ nullptr };

			friend class Msg;
			friend class MsgRT;
			friend class Socket;
			template<typename T> friend class Aris::Control::Pipe;
		};
		class Msg final :public MsgBase
		{
		public:
			/** \brief Default Constructor
			* \param length   The length of message(not count message header), the unit of which is byte.
			* \param msgID  The ID of message
			*/
			explicit Msg(std::int32_t msg_id = 0, std::int32_t size = 0);
			/** \brief Copy Constructor
			* \param other    another message
			*/
			Msg(const Msg& other);
			/** \brief Move Constructor
			* \param other    another message
			*/
			Msg(Msg&& other);
			/** \brief Destructor
			*
			*/
			virtual ~Msg();
			/** \brief Assignment Operator, which is deep copy from another message
			* \param other    another message
			*/
			Msg &operator=(Msg other);
			/** \brief 跟另外一个Msg对象交换数据。仅仅改变双方指针，因此效率高于任何一个构造函数。
			* \param other    另外一个消息。
			*
			*/
			void swap(Msg &other);
			/** \brief Set msg length
			*
			*/
			virtual void resize(std::int32_t size);

		private:
			friend class Socket;
		};
		class MsgRT final :public MsgBase
		{
		public:
			virtual void resize(std::int32_t size);

			enum { RT_MSG_LENGTH = 8192 };
			static MsgRT instance[2];

		private:
			MsgRT();
			virtual ~MsgRT();
			MsgRT(const MsgRT &other) = delete;
			MsgRT(MsgRT &&other) = delete;
			MsgRT &operator=(const MsgRT& other) = delete;
			MsgRT &operator=(MsgRT&& other) = delete;
		};


		const std::string& logFileName();
		const char * log(const char *data);
		const std::string& log(const std::string& data);

		void msSleep(int miliseconds);

	}
}


#endif
