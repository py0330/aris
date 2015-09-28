/*
* Message.h
*
*  Created on: 2012-8-28
*      Author: py0330
*/

#ifndef ARIS_MESSAGE_H_
#define ARIS_MESSAGE_H_

#include <functional>
#include <cstdint>

#include <Aris_Core.h>

namespace Aris
{
	namespace Core
	{
		/** \brief 向主线程发送消息
		* \param InMsg 发送的消息
		*/
		void PostMsg(const Aris::Core::MSG &InMsg);
		/** \brief 注册消息及其对应的回调函数
		* \param message 消息标识，系统此后若碰到该标识的消息，将执行其对应的回调函数
		* \param CallBack 回调函数，该函数应当形如 int CallBack(Aris::Message::MSG & msg){};
		*/
		void RegisterMsgCallback(int message, std::function<int(Aris::Core::MSG &)>);
		/** \brief 注册默认回调函数
		* 当系统碰到没有注册过的消息标识时，就执行默认的回调函数。
		* \param CallBack 默认回调函数，该函数应当形如 int CallBack(Aris::Message::MSG & msg){};
		*/
		void RegisterDefaultCallback(std::function<int(Aris::Core::MSG &)>);
		/** \brief 主线程开始消息循环
		*
		*/
		void RunMsgLoop();
		/** \brief 主线程停止消息循环
		*
		*/
		void StopMsgLoop();
	}
}
#endif /* MESSAGE_H_ */
