#include "Platform.h"
//#include "Aris_Thread.h"
#include "Aris_Message.h"

#include <mutex>
#include <queue>
#include <vector>
#include <algorithm>
#include <map>

#ifdef PLATFORM_IS_WINDOWS 
#include <Windows.h>
#endif
#ifdef PLATFORM_IS_LINUX 
#include <semaphore.h>
#endif

using namespace std;

namespace Aris
{
	namespace Core
	{
		/** \brief 信号量
		*
		* 用于线程之间通讯，每当触发，信号+1，每当等待，信号-1。
		*
		*/
		class SEM
		{
		private:
#ifdef PLATFORM_IS_WINDOWS 
			const static int _SIZE = 4;
#endif
#ifdef PLATFORM_IS_LINUX 
			const static int _SIZE = 32;
#endif
			char _pData[_SIZE];



		public:
			SEM(const SEM & other) = delete;
			SEM &operator=(const SEM& other) = delete;
			SEM(SEM && other) = delete;
			SEM &operator=(SEM&& other) = delete;

			/*! \brief 构造函数，并初始化信号量的值
			* \param iniValue 当信号量被创建时，该信号量的初始值。在Windows平台下该值最大为256。
			*/
			SEM(unsigned int iniValue = 0);
			~SEM();/*!< \brief 析构函数 */
			int Post();/*!< \brief 在某线程中触发信号，该动作使得信号值+1 */
			/*! \brief 在某线程中等待信号，该动作使得信号值-1
			*
			* 当某线程等待信号时，若当前信号值为0，那么本线程阻塞，直到其他线程触发信号，之后信号值-1；
			*若当前信号值大于0，则本线程继续运行，同时信号值-1。
			*/
			int Wait();

		};

		
		std::queue<Aris::Core::MSG> msgq;
		SEM MsgReceived;
		std::mutex MsgMutex,MsgCallbackMutex;
		bool ifSkipLoop = false;
		bool isRunning = false;

		map < int, vector<function<int(Aris::Core::MSG &)> > > Msg_CallBack_Map;
		vector<function<int(Aris::Core::MSG &)> > DefaultCallBacks;

		int PostMsg(const Aris::Core::MSG &InMsg)
		{
			MsgMutex.lock();
			msgq.push(InMsg);
			MsgMutex.unlock();

			MsgReceived.Post();

			return 0;
		}
		int GetMsg(Aris::Core::MSG *Rmsg)
		{
			bool Flag=true;
			while(Flag)
			{
				MsgReceived.Wait();

				MsgMutex.lock();
				if(msgq.empty())
				{
					MsgMutex.unlock();
					continue;
				}
				else
				{
					(*Rmsg).Swap(msgq.front());
					msgq.pop();
					Flag=false;
				}
				MsgMutex.unlock();
			}
			return 0;
		}
		int RegisterMsgCallback(unsigned int message, function<int(Aris::Core::MSG &)> CallBack)
		{
			MsgCallbackMutex.lock();
			
			auto found = Msg_CallBack_Map.find(message);
			if (found == Msg_CallBack_Map.end())
			{
				vector<function<int(Aris::Core::MSG &)> > Msg_CallBacks;
				if (CallBack != nullptr)
				{
					Msg_CallBacks.push_back(CallBack);
				}
				Msg_CallBack_Map[message] = Msg_CallBacks;
			}
			else
			{
				found->second.clear();
				if (CallBack != nullptr)
				{
					found->second.push_back(CallBack);
				}
			}

			MsgCallbackMutex.unlock();

			return 0;
		}
		int RegisterDefaultCallback(std::function<int(Aris::Core::MSG &)> CallBack)
		{
			MsgCallbackMutex.lock();
			
			DefaultCallBacks.clear();
			if (CallBack != nullptr)
			{
				DefaultCallBacks.push_back(CallBack);
			}

			MsgCallbackMutex.unlock();

			return 0;
		}

		int RunMsgLoop()
		{
			if (isRunning)
				return -1;
			else
				isRunning = true;

			Aris::Core::MSG Msg;
			ifSkipLoop = false;

			while ((GetMsg(&Msg) == 0) && (ifSkipLoop==false))
			{
				MsgCallbackMutex.lock();

				auto found = Msg_CallBack_Map.find(Msg.GetMsgID());

				if (found == Msg_CallBack_Map.end())
				{
					for (auto func : DefaultCallBacks)
					{
						if (func != nullptr)
						{
							func(Msg);
						}
					}
				}
				else
				{
					for (auto func : found->second)
					{
						if (func != nullptr)
						{
							func(Msg);
						}
					}
				}
				
				MsgCallbackMutex.unlock();
			}

			isRunning = false;
			return 0;
		}
		int StopMsgLoop()
		{
			if (isRunning)
			{
				ifSkipLoop = true;
				PostMsg(Core::MSG(0, 0));
				return 0;
			}
			else
			{
				return -1;
			}
		}
		int ClearMsgLoop()
		{
			/*清空消息队列*/
			MsgCallbackMutex.lock();
			while (!msgq.empty())
			{
				msgq.pop();
			}
			MsgCallbackMutex.unlock();

			return 0;
		}
	}
}