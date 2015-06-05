#ifndef ARIS_THREAD_H_
#define ARIS_THREAD_H_

#include "Platform.h"

namespace Aris
{
	namespace Core
	{
		/** \brief 互斥量
		*
		* 用于保护变量或内存同一时间只能被一个线程访问。当某线程锁住互斥量时，其他想锁住该互斥量的线程阻塞
		*，直到锁住的线程释放该互斥量。
		*
		*/
		class MUTEX
		{
		private:
#ifdef PLATFORM_IS_WINDOWS 
			const static int _SIZE = 24;
#endif
#ifdef PLATFORM_IS_LINUX 
			const static int _SIZE = 40;
#endif
			char _pData[_SIZE];

		private:
			

		public:
			MUTEX(const MUTEX & other) = delete;
			MUTEX &operator=(const MUTEX& other) = delete;
			MUTEX(MUTEX && other) = delete;
			MUTEX &operator=(MUTEX&& other) = delete;

			MUTEX();/*!< \brief 构造函数 */
			~MUTEX();/*!< \brief 析构函数 */
			int Lock();/*!< \brief 锁住互斥量，同一时间只可能有一个线程锁住互斥量，在此期间其他试图锁住该互斥量的线程阻塞 */
			int Unlock();/*!< \brief 释放互斥量，本线程释放之前锁死的互斥量，使得其他线程可以锁住该互斥量 */
		};
		/** \brief 事件
		*
		* 用于线程之间的通讯，不同于信号量，事件只有两个状态，触发或复位。
		*
		*/
		class EVENT
		{
		private:
#ifdef PLATFORM_IS_WINDOWS 
			const static int _SIZE = 4;
#endif
#ifdef PLATFORM_IS_LINUX 
			const static int _SIZE = 96;
#endif
			char _pData[_SIZE];

		public:
			EVENT(const EVENT & other) = delete;
			EVENT &operator=(const EVENT& other) = delete;
			EVENT(EVENT && other) = delete;
			EVENT &operator=(EVENT&& other) = delete;

			EVENT(bool iniState = false);/*!< \brief 构造函数 */
			~EVENT();/*!< \brief 析构函数 */
			int SetEvent();/*!< \brief 触发事件，该函数使得事件处于触发的状态，其他等待该事件的线程结束阻塞继续执行 */
			int ResetEvent();/*!< \brief 复位事件，该函数使得事件处于复位的状态 */
			int WaitForEvent();/*!< \brief 等待事件发生，若时间处于触发状态，则继续状态，否则线程阻塞，直到该时间被触发 */
		
		};

		/** \brief 线程
		*
		* 用于创建一个线程。
		*
		*/
		class THREAD
		{
		private:
#ifdef PLATFORM_IS_WINDOWS 
			const static int _SIZE = 44;
#endif
#ifdef PLATFORM_IS_LINUX 
			const static int _SIZE = 80;
#endif
			char _pData[_SIZE];

		public:
			THREAD(const THREAD & other) = delete;
			THREAD &operator=(const THREAD& other) = delete;
			THREAD(THREAD && other) = delete;
			THREAD &operator=(THREAD&& other) = delete;

			/** \brief 构造函数
			*
			* \param THREAD_FUNC 线程对应的函数，应当为形如void* Function(void* in){}的函数。
			*/
			THREAD(void* (*THREAD_FUNC)(void *pIn)=0);
			~THREAD();/*!< \brief 析构函数 */

			/** \brief 设置本线程对应的函数
			*
			* 本函数只有在线程没在运行的时候才能执行。
			*
			* \param THREAD_FUNC 线程对应的函数，应当为形如void* Function(void* in){}的函数。
			*/
			int SetFunction(void* (*THREAD_FUNC)(void *pIn));/*!< \brief 设置本线程对应的函数 */
			/** \brief 创建并执行线程
			*
			* 本函数只有在设置了线程对应函数，并且当前线程没有处于运行状态时才能执行。
			*
			* \param pInput 在执行线程对应函数时函数的输入参数。
			*/
			int Start(void* pInput);
			/** \brief 等待线程运行结束
			*
			* 若线程正在运行，则一直等待直到线程退出。
			*/
			int Join();
			/** \brief 终止线程，本函数不安全，强烈不推荐使用，应当让线程自己结束运行
			*
			* 本函数只有在线程处于运行状态时才可使用。但是强烈不推荐使用。
			*/
			int Terminate();
			/** \brief 查看线程是否在运行
			*
			*/
			bool IsRunning();
			/*! \brief 获得线程执行的结果
			* 本函数只有在线程没有处于运行状态时才可使用。
			*
			* \param pOut 线程执行完毕后的返回值。
			*/
			int GetResult(void* pOut);
		};
	}
}

#endif /* MESSAGE_H_ */
