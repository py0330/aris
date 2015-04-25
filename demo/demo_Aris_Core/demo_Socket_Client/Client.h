#ifndef CLIENT_H_
#define CLIENT_H_

#include "Aris_Message.h"
#include "Aris_Socket.h"

enum ClientMessage
{
	VisualSystemDataNeeded,
	VisualSystemLost,
	ControlSystemLost,
	ControlCommandReceived,
};


int OnVisualSystemLost(Aris::Core::MSG &msg);
int OnVisualSystemDataNeeded(Aris::Core::MSG &msg);
int OnControlSystemLost(Aris::Core::MSG &msg);
int OnControlCommandReceived(Aris::Core::MSG &msg);

int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int OnConnectionLost(Aris::Core::CONN *pConn);

extern Aris::Core::CONN *pVisualSystem, *pControlSystem;



#endif