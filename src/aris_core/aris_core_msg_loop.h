#ifndef ARIS_CORE_MSG_LOOP_H_
#define ARIS_CORE_MSG_LOOP_H_

#include <functional>
#include <cstdint>

#include <aris_core_msg.h>

namespace aris
{
	namespace core
	{
		/** \brief 向主线程发送消息
		* \param InMsg 发送的消息
		*/
		void postMsg(const aris::core::Msg &msg);
		/** \brief 注册消息及其对应的回调函数
		* \param message 消息标识，系统此后若碰到该标识的消息，将执行其对应的回调函数
		* \param CallBack 回调函数，该函数应当形如 int CallBack(aris::Message::Msg & msg){};
		*/
		void registerMsgCallback(int msg_id, std::function<int(aris::core::Msg &)> func);
		/** \brief 注册默认回调函数
		* 当系统碰到没有注册过的消息标识时，就执行默认的回调函数。
		* \param CallBack 默认回调函数，该函数应当形如 int CallBack(aris::Message::Msg & msg){};
		*/
		void registerDefaultCallback(std::function<int(aris::core::Msg &)> func);
		/** \brief 主线程开始消息循环
		*
		*/
		void runMsgLoop();
		/** \brief 主线程停止消息循环
		*
		*/
		void stopMsgLoop();
	}
}
#endif /* MESSAGE_H_ */
