#include <Platform.h>
#include "Aris_Message.h"
#include "Aris_Socket.h"



enum ClientMessage
{
	VisualSystemConnected,
	VisualSystemDataReceived,
	VisualSystemLost,
	ControlSystemConnected,
	ControlTrajectoryFinished,
	ControlSystemLost,
};

int OnVisualSystemConnected(Aris::Core::MSG &msg);
int OnControlSystemConnected(Aris::Core::MSG &msg);
int OnVisualSystemLost(Aris::Core::MSG &msg);
int OnControlSystemLost(Aris::Core::MSG &msg);
int OnVisualSystemDataReceived(Aris::Core::MSG &msg);
int OnControlTrajectoryFinished(Aris::Core::MSG &msg);

int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int OnConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port);
int OnConnectionLost(Aris::Core::CONN *pConn);

extern Aris::Core::CONN VisualSystem, ControlSystem;