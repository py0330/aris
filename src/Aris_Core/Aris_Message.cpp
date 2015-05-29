#include "Platform.h"
#include "Aris_Thread.h"
#include "Aris_Message.h"

#include <queue>
#include <vector>
#include <algorithm>
#include <map>

using namespace std;

namespace Aris
{
	namespace Core
	{
		std::queue<Aris::Core::MSG> msgq;
		SEM MsgReceived;
		MUTEX MsgMutex,MsgCallbackMutex;
		bool ifSkipLoop = false;
		bool isRunning = false;

		map < int, vector<function<int(Aris::Core::MSG &)> > > Msg_CallBack_Map;
		vector<function<int(Aris::Core::MSG &)> > DefaultCallBacks;

		int PostMsg(const Aris::Core::MSG &InMsg)
		{
			MsgMutex.Lock();
			msgq.push(InMsg);
			MsgMutex.Unlock();

			MsgReceived.Post();

			return 0;
		}
		int GetMsg(Aris::Core::MSG *Rmsg)
		{
			bool Flag=true;
			while(Flag)
			{
				MsgReceived.Wait();

				MsgMutex.Lock();
				if(msgq.empty())
				{
					MsgMutex.Unlock();
					continue;
				}
				else
				{
					(*Rmsg).Swap(msgq.front());
					msgq.pop();
					Flag=false;
				}
				MsgMutex.Unlock();
			}
			return 0;
		}
		int RegisterMsgCallback(unsigned int message, function<int(Aris::Core::MSG &)> CallBack)
		{
			MsgCallbackMutex.Lock();
			
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

			MsgCallbackMutex.Unlock();

			return 0;
		}
		int RegisterDefaultCallback(std::function<int(Aris::Core::MSG &)> CallBack)
		{
			MsgCallbackMutex.Lock();
			
			DefaultCallBacks.clear();
			if (CallBack != nullptr)
			{
				DefaultCallBacks.push_back(CallBack);
			}

			MsgCallbackMutex.Unlock();

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
				MsgCallbackMutex.Lock();

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
				
				MsgCallbackMutex.Unlock();
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
			MsgCallbackMutex.Lock();
			while (!msgq.empty())
			{
				msgq.pop();
			}
			MsgCallbackMutex.Unlock();

			return 0;
		}
	}
}