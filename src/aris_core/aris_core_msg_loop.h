#ifndef ARIS_CORE_MSG_LOOP_H_
#define ARIS_CORE_MSG_LOOP_H_

#include <functional>
#include <cstdint>

#include <aris_core_msg.h>

namespace aris
{
	namespace core
	{
		void postMsg(const aris::core::Msg &msg);
		void registerMsgCallback(int msg_id, std::function<int(aris::core::Msg &)> func);
		void registerDefaultCallback(std::function<int(aris::core::Msg &)> func);
		void runMsgLoop();
		void stopMsgLoop();
	}
}
#endif 
