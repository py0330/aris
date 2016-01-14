#ifndef ARIS_CORE_H_
#define ARIS_CORE_H_

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
			std::int32_t msgLength;
			std::int32_t msgID;
			std::int64_t msgType;
			std::int64_t reserved1;
			std::int64_t reserved2;
			std::int64_t reserved3;
		};
		class MsgBase
		{
		public:
			/** \brief 设置Msg中所包含的数据的长度
			* \param dataLength    数据长度
			*/
			virtual void SetLength(std::int32_t dataLength) = 0;
			/** \brief 获取Msg中所包含的数据的长度
			*
			*/
			std::int32_t GetLength() const;
			/** \brief 设置Msg的ID，在消息循环中根据该ID来查找对应的消息回调函数
			* \param msgID   Msg的ID
			*/
			void SetMsgID(std::int32_t msgID);
			/** \brief 获取Msg中的ID
			*
			*/
			std::int32_t GetMsgID() const;
			/** \brief 获取Msg中的数据地址
			*
			*/
			char* GetDataAddress() const;
			/** \brief 从fromThisMemory指针中拷贝字符串。
			* \param fromThisMemory    待拷贝的内存地址。
			* \param dataLength        数据长度
			*
			*/
			void Copy(const char * fromThisMemory);
			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，在拷贝完之后，Msg的长度自动设置为dataLength。
			* \param fromThisMemory    待拷贝的内存地址。
			* \param dataLength        数据长度
			*
			*/
			void Copy(const void * fromThisMemory, std::int32_t dataLength);
			/** \brief 从fromThisMemory指针中拷贝Msg.GetLength()大小的数据。
			* \param fromThisMemory    待拷贝的内存地址。
			*
			*/
			void Copy(const void * fromThisMemory);
			/** \brief 从fromThisMemory指针中拷贝Msg.GetLength()大小的数据到Msg内存中的指定地点。
			* \param fromThisMemory       待拷贝的内存地址。
			* \param dataLength           数据长度
			* \param atThisPositionInMsg  将数据拷贝到Msg.GetDataAddress()[atThisPositionInMsg]处。
			*/
			void CopyAt(const void * fromThisMemory, std::int32_t dataLength, std::int32_t atThisPositionInMsg);
			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，这些数据添加到自身的尾部，在拷贝完之后，Msg的长度自动增加dataLength。
			* \param fromThisMemory    目标内存地址。
			*
			*/
			void CopyMore(const void * fromThisMemory, std::int32_t dataLength);
			/** \brief 向toThisMemory指针中粘贴dataLength长度的数据，若dataLength大于自身的数据长度，则只拷贝自身长度的内存。
			* \param fromThisMemory    目标内存地址。
			* \param dataLength        数据长度
			*
			*/
			void Paste(void * toThisMemory, std::int32_t dataLength) const;
			/** \brief 向toThisMemory指针中粘贴Msg.GetLength()长度的数据。
			* \param fromThisMemory    目标内存地址。
			*
			*/
			void Paste(void * toThisMemory) const;
			/** \brief 向toThisMemory指针中粘贴dataLength长度的数据，若dataLength大于自身的数据长度，则只拷贝自身长度的内存。
			* \param fromThisMemory    目标内存地址。
			* \param dataLength        数据长度
			*
			*/
			void PasteAt(void * toThisMemory, std::int32_t dataLength, std::int32_t atThisPositionInMsg) const;
			/** \brief 默认析构函数
			*
			*/
			virtual ~MsgBase() = default;

			template<class... Args>
			void CopyStruct(const Args&... args)
			{
				SetLength(0);
				CopyStructMore(args...);
			}

			template<class FirstArg, class... Args>
			void CopyStructMore(const FirstArg& firstArg, const Args&... args)
			{
				CopyMore(static_cast<const void*>(&firstArg), sizeof(FirstArg));
				CopyStructMore(args...);
			}

			template<class FirstArg, class... Args>
			void PasteStruct(FirstArg& firstArg, Args&... args) const
			{
				PasteAt(static_cast<void*>(&firstArg), sizeof(FirstArg), pasteID);
				pasteID += sizeof(FirstArg);
				PasteStruct(args...);
			}

		private:
			void SetType(std::int64_t type);
			std::int64_t GetType() const;
			void CopyStructMore()
			{
			}
			void PasteStruct() const
			{
				pasteID = 0;
			}

		private:
			MsgBase() = default;
			MsgBase(const MsgBase &other) = delete;
			MsgBase(MsgBase &&other) = delete;
			MsgBase &operator=(const MsgBase& other) = delete;
			MsgBase &operator=(MsgBase&& other) = delete;

		private:
			mutable std::int32_t pasteID{ 0 };
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
			explicit Msg(std::int32_t msgID = 0, std::int32_t dataLength = 0);
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
			void Swap(Msg &other);
			/** \brief Set msg length
			*
			*/
			virtual void SetLength(std::int32_t dataLength);

		private:
			friend class Socket;
		};
		class MsgRT final :public MsgBase
		{
		public:
			virtual void SetLength(std::int32_t dataLength);

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

		void Sleep(int mSeconds);

	}
}


#endif
