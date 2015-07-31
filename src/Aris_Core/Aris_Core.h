#ifndef ARIS_CORE_H_
#define ARIS_CORE_H_

#include <cstring>
#include <cstdint>
#include <cstdio>

namespace Aris
{
	namespace RT_CONTROL
	{
		class ACTUATION;
	}
	
	namespace Core
	{
		class CONN;
		class MSG;
		class RT_MSG;

		struct MSG_HEADER
		{
			std::int32_t msgLength;
			std::int32_t msgID;
			std::int64_t msgType;
			std::int64_t reserved1;
			std::int64_t reserved2;
			std::int64_t reserved3;
		};
		class MSG_BASE
		{
		public:
			/** \brief 设置MSG中所包含的数据的长度
			* \param dataLength    数据长度
			*/
			virtual void SetLength(std::int32_t dataLength) = 0;
			/** \brief 获取MSG中所包含的数据的长度
			*
			*/
			std::int32_t GetLength() const;
			/** \brief 设置MSG的ID，在消息循环中根据该ID来查找对应的消息回调函数
			* \param msgID   MSG的ID
			*/
			void SetMsgID(std::int32_t msgID);
			/** \brief 获取MSG中的ID
			*
			*/
			std::int32_t GetMsgID() const;
			/** \brief 获取MSG中的数据地址
			*
			*/
			char* GetDataAddress() const;
			/** \brief 从fromThisMemory指针中拷贝字符串。
			* \param fromThisMemory    待拷贝的内存地址。
			* \param dataLength        数据长度
			*
			*/
			void Copy(const char * fromThisMemory);
			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，在拷贝完之后，MSG的长度自动设置为dataLength。
			* \param fromThisMemory    待拷贝的内存地址。
			* \param dataLength        数据长度
			*
			*/
			void Copy(const void * fromThisMemory, std::int32_t dataLength);
			/** \brief 从fromThisMemory指针中拷贝MSG.GetLength()大小的数据。
			* \param fromThisMemory    待拷贝的内存地址。
			*
			*/
			void Copy(const void * fromThisMemory);
			/** \brief 从fromThisMemory指针中拷贝MSG.GetLength()大小的数据到MSG内存中的指定地点。
			* \param fromThisMemory       待拷贝的内存地址。
			* \param dataLength           数据长度
			* \param atThisPositionInMsg  将数据拷贝到MSG.GetDataAddress()[atThisPositionInMsg]处。
			*/
			void CopyAt(const void * fromThisMemory, std::int32_t dataLength, std::int32_t atThisPositionInMsg);
			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，这些数据添加到自身的尾部，在拷贝完之后，MSG的长度自动增加dataLength。
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
			/** \brief 向toThisMemory指针中粘贴MSG.GetLength()长度的数据。
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
			virtual ~MSG_BASE() = default;

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
			MSG_BASE() = default;
			MSG_BASE(const MSG_BASE &other) = delete;
			MSG_BASE(MSG_BASE &&other) = delete;
			MSG_BASE &operator=(const MSG_BASE& other) = delete;
			MSG_BASE &operator=(MSG_BASE&& other) = delete;

		private:
			mutable std::int32_t pasteID{ 0 };
			char *_pData{ nullptr };

			friend class MSG;
			friend class RT_MSG;
			friend class CONN;
			friend class Aris::RT_CONTROL::ACTUATION;
		};
		class MSG final :public MSG_BASE
		{
		public:
			/** \brief Default Constructor
			* \param length   The length of message(not count message header), the unit of which is byte.
			* \param msgID  The ID of message
			*/
			explicit MSG(std::int32_t msgID = 0, std::int32_t dataLength = 0);
			/** \brief Copy Constructor
			* \param other    another message
			*/
			MSG(const MSG& other);
			/** \brief Move Constructor
			* \param other    another message
			*/
			MSG(MSG&& other);
			/** \brief Destructor
			*
			*/
			virtual ~MSG();
			/** \brief Assignment Operator, which is deep copy from another message
			* \param other    another message
			*/
			MSG &operator=(MSG other);
			/** \brief 跟另外一个MSG对象交换数据。仅仅改变双方指针，因此效率高于任何一个构造函数。
			* \param other    另外一个消息。
			*
			*/
			void Swap(MSG &other);
			/** \brief Set msg length
			*
			*/
			virtual void SetLength(std::int32_t dataLength);

		private:
			friend class CONN;
			friend class Aris::RT_CONTROL::ACTUATION;
		};
		class RT_MSG final :public MSG_BASE
		{
		public:
			virtual void SetLength(std::int32_t dataLength);

			enum { RT_MSG_LENGTH = 8192 };
			static RT_MSG instance[2];

		private:
			RT_MSG();
			virtual ~RT_MSG();
			RT_MSG(const RT_MSG &other) = delete;
			RT_MSG(RT_MSG &&other) = delete;
			RT_MSG &operator=(const RT_MSG& other) = delete;
			RT_MSG &operator=(RT_MSG&& other) = delete;

			friend class Aris::RT_CONTROL::ACTUATION;
		};
		

		const char * log(const char *data);
	}
}


#endif
