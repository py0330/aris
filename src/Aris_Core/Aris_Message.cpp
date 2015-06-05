#include "Platform.h"
#include "Aris_Message.h"

#include <mutex>
#include <condition_variable>
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
		class SEM
		{
		private:
			std::mutex _mutex;
			std::condition_variable _cv;
			volatile unsigned int _sig_value;

		public:
			SEM(const SEM & other) = delete;
			SEM &operator=(const SEM& other) = delete;
			SEM(SEM && other) = delete;
			SEM &operator=(SEM&& other) = delete;

			SEM(unsigned int iniValue = 0);
			~SEM();
			int Post();
			int Wait();

		};

		SEM::SEM(unsigned int iniValue)
		{
			std::lock_guard<std::mutex> lck(_mutex);
			_sig_value = iniValue;
		}
		SEM::~SEM()
		{
		}
		int SEM::Post()
		{
			std::lock_guard<std::mutex> lck(_mutex);
			_sig_value++;
			if (_sig_value==1)
			{
				_cv.notify_one();
			}

			return 0;
		}
		int SEM::Wait()
		{
			std::unique_lock<std::mutex> lck(_mutex);
			if (_sig_value == 0)
			{
				_cv.wait(lck);
			}
			_sig_value--;

			return 0;
		}

		
		std::queue<Aris::Core::MSG> msgq;
		SEM MsgReceived;
		std::mutex MsgMutex,MsgCallbackMutex;
		bool ifSkipLoop = false;
		bool isRunning = false;

		map < int, vector<function<int(Aris::Core::MSG &)> > > Msg_CallBack_Map;
		vector<function<int(Aris::Core::MSG &)> > DefaultCallBacks;

		int PostMsg(const Aris::Core::MSG &InMsg)
		{
			std::lock_guard<std::mutex> lck(MsgMutex);
			
			msgq.push(InMsg);
			MsgReceived.Post();

			return 0;
		}
		int GetMsg(Aris::Core::MSG &rMsg)
		{
			bool Flag=true;
			while(Flag)
			{
				MsgReceived.Wait();

				std::lock_guard<std::mutex> lck(MsgMutex);

				if(!msgq.empty())
				{
					rMsg.Swap(msgq.front());
					msgq.pop();
					Flag=false;
				}
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

			while ((GetMsg(Msg) == 0) && (ifSkipLoop==false))
			{
				std::lock_guard<std::mutex> lck(MsgCallbackMutex);

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