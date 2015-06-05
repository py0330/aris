#include "Platform.h"
#include "Aris_Thread.h"
#include <new>

#ifdef PLATFORM_IS_WINDOWS 
#define   WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#ifdef PLATFORM_IS_LINUX 
#include <pthread.h>
#include <semaphore.h>
#include "signal.h"
#endif



namespace Aris
{
	namespace Core
	{


		struct MUTEX_STRUCT
		{
#ifdef PLATFORM_IS_WINDOWS 
			CRITICAL_SECTION m_CriticalSection;
#endif
#ifdef PLATFORM_IS_LINUX 
			pthread_mutex_t m_Mutex;
#endif
		};

		MUTEX::MUTEX()
		{
			static_assert(sizeof(MUTEX_STRUCT) <= MUTEX::_SIZE, "Aris::Core::MUTEX need more memory");

			#ifdef PLATFORM_IS_WINDOWS
			InitializeCriticalSection(&(((MUTEX_STRUCT*)_pData)->m_CriticalSection));
			#endif
			#ifdef PLATFORM_IS_LINUX 
			pthread_mutex_init(&(((MUTEX_STRUCT*)_pData)->m_Mutex), NULL);
			#endif
		}
		MUTEX::~MUTEX()
		{
			#ifdef PLATFORM_IS_WINDOWS
			DeleteCriticalSection(&(((MUTEX_STRUCT*)_pData)->m_CriticalSection));
			#endif
			#ifdef PLATFORM_IS_LINUX 
			pthread_mutex_destroy(&(((MUTEX_STRUCT*)_pData)->m_Mutex));
			#endif
		}
		int MUTEX::Lock()
		{
			#ifdef PLATFORM_IS_WINDOWS
			EnterCriticalSection(&(((MUTEX_STRUCT*)_pData)->m_CriticalSection));
			return 0;
			#endif
			#ifdef PLATFORM_IS_LINUX 
			return pthread_mutex_lock(&(((MUTEX_STRUCT*)_pData)->m_Mutex));
			#endif
		}
		int MUTEX::Unlock()
		{
			#ifdef PLATFORM_IS_WINDOWS
			LeaveCriticalSection(&(((MUTEX_STRUCT*)_pData)->m_CriticalSection));
			return 0;
			#endif
			#ifdef PLATFORM_IS_LINUX 
			return pthread_mutex_unlock(&(((MUTEX_STRUCT*)_pData)->m_Mutex));
			#endif
			
		}

		struct EVENT_STRUCT
		{
#ifdef PLATFORM_IS_WINDOWS 
			HANDLE m_Event;
#endif
#ifdef PLATFORM_IS_LINUX 
			pthread_cond_t m_Event;
			pthread_mutex_t m_Mutex;
			bool m_State;
#endif
		};

		EVENT::EVENT(bool iniState)
		{
			static_assert(sizeof(EVENT_STRUCT) <= EVENT::_SIZE, "Aris::Core::EVENT need more memory");

			#ifdef PLATFORM_IS_WINDOWS
			((EVENT_STRUCT*)_pData)->m_Event = CreateEvent(NULL, TRUE, iniState, NULL);
			#endif
			#ifdef PLATFORM_IS_LINUX 
			((EVENT_STRUCT*)_pData)->m_State=iniState;
			pthread_cond_init(&(((EVENT_STRUCT*)_pData)->m_Event),NULL);
			#endif
		}
		EVENT::~EVENT()
		{
			#ifdef PLATFORM_IS_WINDOWS
			CloseHandle(((EVENT_STRUCT*)_pData)->m_Event);
			#endif
			#ifdef PLATFORM_IS_LINUX 
			pthread_mutex_destroy(&(((EVENT_STRUCT*)_pData)->m_Mutex));
			pthread_cond_destroy(&(((EVENT_STRUCT*)_pData)->m_Event));
			#endif
		}
		int EVENT::SetEvent()
		{
			#ifdef PLATFORM_IS_WINDOWS
			::SetEvent(((EVENT_STRUCT*)_pData)->m_Event);
			return 0;
			#endif
			#ifdef PLATFORM_IS_LINUX 
			pthread_mutex_lock(&(((EVENT_STRUCT*)_pData)->m_Mutex));
			((EVENT_STRUCT*)_pData)->m_State=true;
			pthread_cond_broadcast(&(((EVENT_STRUCT*)_pData)->m_Event));
			pthread_mutex_unlock(&(((EVENT_STRUCT*)_pData)->m_Mutex));
			return 0;
			#endif
		}
		int EVENT::ResetEvent()
		{
			#ifdef PLATFORM_IS_WINDOWS
			::ResetEvent(((EVENT_STRUCT*)_pData)->m_Event);
			return 0;
			#endif
			#ifdef PLATFORM_IS_LINUX 
			pthread_mutex_lock(&(((EVENT_STRUCT*)_pData)->m_Mutex));
			((EVENT_STRUCT*)_pData)->m_State=false;
			pthread_mutex_unlock(&(((EVENT_STRUCT*)_pData)->m_Mutex));
			return 0;
			#endif
		}
		int EVENT::WaitForEvent()
		{
			#ifdef PLATFORM_IS_WINDOWS
			WaitForSingleObject(((EVENT_STRUCT*)_pData)->m_Event, INFINITE);
			return 0;
			#endif
			#ifdef PLATFORM_IS_LINUX 
			pthread_mutex_lock(&(((EVENT_STRUCT*)_pData)->m_Mutex));
			while (!(((EVENT_STRUCT*)_pData)->m_State))
				pthread_cond_wait(&(((EVENT_STRUCT*)_pData)->m_Event), &(((EVENT_STRUCT*)_pData)->m_Mutex));
			pthread_mutex_unlock(&(((EVENT_STRUCT*)_pData)->m_Mutex));
			return 0;
			#endif
		}

		struct THREAD_STRUCT
		{
#ifdef PLATFORM_IS_WINDOWS 
			HANDLE m_handle;
			DWORD WINAPI ThreadFunc(LPVOID pIn);
#endif
#ifdef PLATFORM_IS_LINUX 
			pthread_t m_handle;
#endif
			void* (*m_Func)(void *pIn);
			void* m_Input;
			void* m_Result;
			bool m_IsRunning;
			MUTEX m_Mutex;
		};

		#ifdef PLATFORM_IS_WINDOWS
		DWORD WINAPI ThreadFunc(LPVOID pIn)
		#endif
		#ifdef PLATFORM_IS_LINUX 
		void *ThreadFunc(void *pIn)
		#endif
		{
			THREAD_STRUCT* pThread = (THREAD_STRUCT*)(pIn);

			pThread->m_Result = pThread->m_Func(pThread->m_Input);

			pThread->m_Mutex.Lock();

			pThread->m_IsRunning = false;
#ifdef PLATFORM_IS_WINDOWS
			CloseHandle(pThread->m_handle);
#endif
#ifdef PLATFORM_IS_LINUX
			pthread_detach(pThread->m_handle);
#endif
			pThread->m_Mutex.Unlock();
			
			return 0;
		}

		THREAD::THREAD(void* (*THREAD_FUNC)(void *pIn))
		{
			static_assert(sizeof(THREAD_STRUCT) <= THREAD::_SIZE, "Aris::Core::THREAD need more memory");

			((THREAD_STRUCT*)_pData)->m_Func = THREAD_FUNC;
			((THREAD_STRUCT*)_pData)->m_Input = 0;
			((THREAD_STRUCT*)_pData)->m_Result = 0;
			((THREAD_STRUCT*)_pData)->m_IsRunning = 0;

			new (&(((THREAD_STRUCT*)_pData)->m_Mutex)) MUTEX;
		}
		THREAD::~THREAD()
		{
			Terminate();
			(&((THREAD_STRUCT*)_pData)->m_Mutex)->~MUTEX();
		}
		int THREAD::SetFunction(void* (*THREAD_FUNC)(void *pIn))
		{
			int ret;
			
			((THREAD_STRUCT*)_pData)->m_Mutex.Lock();
			if (!((THREAD_STRUCT*)_pData)->m_IsRunning)
			{
				((THREAD_STRUCT*)_pData)->m_Func = THREAD_FUNC;
				ret = 0;
			}
			else
			{
				ret = -1;
			}
			((THREAD_STRUCT*)_pData)->m_Mutex.Unlock();

			return ret;
		}
		int THREAD::Start(void* pInput)
		{
			int ret;
			
			((THREAD_STRUCT*)_pData)->m_Mutex.Lock();
			if ((!((THREAD_STRUCT*)_pData)->m_IsRunning) && (((THREAD_STRUCT*)_pData)->m_Func != 0))
			{
				((THREAD_STRUCT*)_pData)->m_IsRunning = true;
				((THREAD_STRUCT*)_pData)->m_Input = pInput;
				
				#ifdef PLATFORM_IS_WINDOWS
					((THREAD_STRUCT*)_pData)->m_handle = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ThreadFunc, (LPVOID)this->_pData, 0, NULL);
				#endif
				#ifdef PLATFORM_IS_LINUX 
					pthread_create(&(((THREAD_STRUCT*)_pData)->m_handle), NULL, ThreadFunc, ((THREAD_STRUCT*)_pData));
				#endif
				ret = 0;
			}
			else
			{
				ret = -1;
			}
			((THREAD_STRUCT*)_pData)->m_Mutex.Unlock();

			return ret;
		}
		int THREAD::Join()
		{
			((THREAD_STRUCT*)_pData)->m_Mutex.Lock();
#ifdef PLATFORM_IS_WINDOWS
			if (((THREAD_STRUCT*)_pData)->m_IsRunning)
			{
				WaitForSingleObject(((THREAD_STRUCT*)_pData)->m_handle, INFINITE);
				return 0;
			}
			else
			{
				return 0;
			}
#endif
#ifdef PLATFORM_IS_LINUX 
			if (((THREAD_STRUCT*)_pData)->m_IsRunning)
			{
				pthread_join(((THREAD_STRUCT*)_pData)->m_handle, NULL);
				return 0;
			}
			else
			{
				return 0;
			}
#endif
			((THREAD_STRUCT*)_pData)->m_Mutex.Unlock();
		}
		int THREAD::Terminate()
		{
			int ret;
			
			((THREAD_STRUCT*)_pData)->m_Mutex.Lock();
			if (((THREAD_STRUCT*)_pData)->m_IsRunning)
			{
				#ifdef PLATFORM_IS_WINDOWS
				TerminateThread(((THREAD_STRUCT*)_pData)->m_handle, 0);
				CloseHandle(((THREAD_STRUCT*)_pData)->m_handle);
				#endif
				#ifdef PLATFORM_IS_LINUX 
				pthread_kill(((THREAD_STRUCT*)_pData)->m_handle,SIGQUIT);
				pthread_join(((THREAD_STRUCT*)_pData)->m_handle,NULL);
				#endif
				((THREAD_STRUCT*)_pData)->m_IsRunning = false;
				ret = 0;
			}
			else
			{
				ret = -1;
			}
			((THREAD_STRUCT*)_pData)->m_Mutex.Unlock();

			return ret;
		}
		int THREAD::GetResult(void* pOut)
		{
			int ret;
			
			((THREAD_STRUCT*)_pData)->m_Mutex.Lock();
			if (((THREAD_STRUCT*)_pData)->m_IsRunning)
			{
				ret = -1;
			}
			else
			{
				pOut = ((THREAD_STRUCT*)_pData)->m_Result;
				ret=0;
			}
			((THREAD_STRUCT*)_pData)->m_Mutex.Unlock();

			return ret;
		}
		bool THREAD::IsRunning()
		{
			return ((THREAD_STRUCT*)_pData)->m_IsRunning;
		}
	}
}
