#ifndef ARIS_CORE_H_
#define ARIS_CORE_H_

namespace Aris
{
	namespace RT_CONTROL
	{
		class ACTUATION;
	}
	
	namespace Core
	{
		class CONN;

// 0-3  字节，  unsigned int 代表数据大小
// 4-7  字节，  int          代表msgID
// 8-15 字节，  long long    代表type
// 16-23字节，  long long    目前保留，准备用于时间戳
// 24-31字节，  long long    目前保留，准备用于时间戳
// 32-39字节，  long long    用户可以自定义的8字节数据
#define MSG_HEADER_LENGTH 40
		class MSG
		{
			friend class CONN;
			friend class Aris::RT_CONTROL::ACTUATION;

		private:
			char *_pData; /*!< \brief 消息对应的数据地址 */
		public:
			/** \brief 默认构造函数
			* \param length   MSG结构中包含的数据长度
			* \param msgID  MSG结构所对应的消息枚举
			*/
			MSG(int msgID = 0, unsigned int dataLength = 0);
			/** \brief 拷贝构造函数，本函数为深度复制
			* \param other    另外一个MSG
			*/
			MSG(const MSG& other);
			/** \brief 移动构造函数
			* \param other    另外一个MSG
			*/
			MSG(MSG&& other);
			/** \brief 析构函数
			*
			*/
			~MSG();

			/** \brief 赋值操作符，本函数为深度复制
			* \param other    另外一个MSG
			*/
			MSG &operator=(const MSG& other);
			/** \brief 右值赋值操作符
			* \param other    另外一个MSG
			*/
			MSG &operator=(MSG&& other);

			/** \brief 设置MSG中所包含的数据的长度
			* \param dataLength    数据长度
			*/
			void SetLength(unsigned int dataLength);
			/** \brief 设置MSG的ID，在消息循环中根据该ID来查找对应的消息回调函数
			* \param msgID   MSG的ID
			*/
			void SetMsgID(int msgID);
			
			/** \brief 获取MSG中所包含的数据的长度
			* 
			*/
			unsigned int GetLength() const;
			/** \brief 获取MSG中的ID
			*
			*/
			int GetMsgID() const;
			/** \brief 获取MSG中的数据地址
			*
			*/
			char* GetDataAddress() const;

			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，在拷贝完之后，MSG的长度自动设置为dataLength。
			* \param fromThisMemory    待拷贝的内存地址。
			* \param dataLength        数据长度
			*
			*/
			void Copy(const void * fromThisMemory, unsigned int dataLength);
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
			void CopyAt(const void * fromThisMemory, unsigned int dataLength, unsigned int atThisPositionInMsg);
			/** \brief 从fromThisMemory指针中拷贝dataLength长度的数据，这些数据添加到自身的尾部，在拷贝完之后，MSG的长度自动增加dataLength。
			* \param fromThisMemory    目标内存地址。
			*
			*/
			void CopyMore(const void * fromThisMemory, unsigned int dataLength);

			/** \brief 向toThisMemory指针中粘贴dataLength长度的数据，若dataLength大于自身的数据长度，则只拷贝自身长度的内存。
			* \param fromThisMemory    目标内存地址。
			* \param dataLength        数据长度
			*
			*/
			void Paste(void * toThisMemory,unsigned int dataLength) const;
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
			void PasteAt(void * toThisMemory, unsigned int dataLength, unsigned int atThisPositionInMsg) const;

			/** \brief 跟另外一个MSG对象交换数据。仅仅改变双方指针，因此效率高于任何一个构造函数。
			* \param other    另外一个消息。
			*
			*/
			void Swap(MSG &other);

		private:
			void SetType(long long type);
			long long GetType() const;
		};



	}
}


#endif
