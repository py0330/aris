#ifdef WIN32
#include <ecrt_windows_py.h>//just for IDE vs2015, it does not really work
#endif
#ifdef UNIX
// following for pipe
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <rtdm/rtdm.h>
#include <rtdm/rtipc.h>
#include <mutex>
#endif

#include <mutex>
#include <string>
#include <iostream>

#include "aris_control_ethercat.h"


namespace aris
{
	namespace control
	{

		class PipeBase::Imp
		{
		public:
			Imp(bool isBlock)
			{
				static int port = 0;
				InitRT(port);
				InitNRT(port, isBlock);
				++port;
			}
			
		private:
			void InitRT(int port)
			{
#ifdef UNIX
				if ((FD_RT = rt_dev_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP)) < 0)
				{
					throw std::runtime_error(std::string("RT data communication failed! Port:") + std::to_string(port));
				}

				struct timeval tv;
				tv.tv_sec = 1;  /* 30 Secs Timeout */
				tv.tv_usec = 0;  // Not Init'ing this can cause strange errors
				if (rt_dev_setsockopt(FD_RT, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)))
				{
					throw std::runtime_error(std::string("RT data communication failed! Port:") + std::to_string(port));
				}

				/*
				* Set a local 16k pool for the RT endpoint. Memory needed to
				* convey datagrams will be pulled from this pool, instead of
				* Xenomai's system pool.
				*/
				const std::size_t poolsz = 16384; /* bytes */
				if (rt_dev_setsockopt(FD_RT, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz)))
				{
					throw std::runtime_error(std::string("RT data communication failed! Port:") + std::to_string(port));
				}

				/*
				* Bind the socket to the port, to setup a proxy to channel
				* traffic to/from the Linux domain.
				*
				* saddr.sipc_port specifies the port number to use.
				*/
				struct sockaddr_ipc saddr;
				memset(&saddr, 0, sizeof(saddr));
				saddr.sipc_family = AF_RTIPC;
				saddr.sipc_port = port;
				if (rt_dev_bind(FD_RT, (struct sockaddr *)&saddr, sizeof(saddr)))
				{
					throw std::runtime_error(std::string("RT pipe Init failed! Port:") + std::to_string(port));
				}
#endif
			}
			void InitNRT(int port, bool isBlock)
			{
#ifdef UNIX
				if (asprintf(&FD_NRT_DEVNAME, "/dev/rtp%d", port) < 0)
				{
					throw std::runtime_error("Error in asprintf");
				}
				FD_NRT = open(FD_NRT_DEVNAME, O_RDWR);
				free(FD_NRT_DEVNAME);

				if (FD_NRT < 0)
				{
					throw std::runtime_error(std::string("NRT pipe Init failed! Port:") + std::to_string(port));
				}


				if (isBlock)
				{
					//            set to block
					int flags = fcntl(FD_NRT, F_GETFL, 0);
					fcntl(FD_NRT, F_SETFL, flags &~O_NONBLOCK);
				}
				else
				{
					//            set to nonblock
					int flags = fcntl(FD_NRT, F_GETFL, 0);
					fcntl(FD_NRT, F_SETFL, flags | O_NONBLOCK);
				}
#endif
			}

			int FD_RT;
			int FD_NRT;
			char* FD_NRT_DEVNAME;

			std::recursive_mutex mutexInNRT;

			friend class PipeBase;

		};

		PipeBase::PipeBase(bool isBlock):pImp(new PipeBase::Imp(isBlock)){}
		PipeBase::~PipeBase(){}
		int PipeBase::sendToRTRawData(const void *pData, int size)
		{
#ifdef UNIX
			if (pData == nullptr)
			{
				throw std::runtime_error("SendNRTtoRT:Invalid pointer");
			}
			std::lock_guard<std::recursive_mutex> guard(pImp->mutexInNRT);
			return write(pImp->FD_NRT, pData, size);
#endif
#ifdef WIN32
			return 0;
#endif
		}
		int PipeBase::sendToNrtRawData(const void* pData, int size)
		{
#ifdef UNIX
			if (pData == nullptr)
			{
				throw std::runtime_error("SendRTtoNRT:Invalid pointer");
			}
			return rt_dev_sendto(pImp->FD_RT, pData, size, 0, NULL, 0);
#endif
#ifdef WIN32
			return 0;
#endif
		}
		int PipeBase::recvInRTRawData(void* pData, int size)
		{
#ifdef UNIX
			if (pData == nullptr)
			{
				throw std::runtime_error("RecvRTfromNRT:Invalid pointer");
			}
			return rt_dev_recvfrom(pImp->FD_RT, pData, size, MSG_DONTWAIT, NULL, 0);
#endif
#ifdef WIN32
			return 0;
#endif
		}
		int PipeBase::recvInNrtRawData(void *pData, int size)
		{
#ifdef UNIX
			if (pData == nullptr)
			{
				throw std::runtime_error("RecvNRTfromRT:Invalid pointer");
			}
			std::lock_guard<std::recursive_mutex> guard(pImp->mutexInNRT);
			return read(pImp->FD_NRT, pData, size);
#endif
#ifdef WIN32
			return 0;
#endif
		}
		
		Pipe<aris::core::Msg>::Pipe(bool isBlock) :PipeBase(isBlock)
		{
		}
		int Pipe<aris::core::Msg>::sendToRT(const aris::core::Msg &msg)
		{
			sendToRTRawData(msg.data_, msg.size() + sizeof(aris::core::MsgHeader));
			return msg.size() + sizeof(aris::core::MsgHeader);
		}
		int Pipe<aris::core::Msg>::sendToNrt(const aris::core::MsgRT &msg)
		{
			sendToNrtRawData(msg.data_, msg.size() + sizeof(aris::core::MsgHeader));
			return msg.size() + sizeof(aris::core::MsgHeader);
		}
		int Pipe<aris::core::Msg>::recvInRT(aris::core::MsgRT &msg)
		{
			int length = recvInRTRawData(msg.data_, sizeof(aris::core::MsgHeader)+aris::core::MsgRT::RT_MSG_LENGTH);		
			return length<=0?0:length;			
		}
		int Pipe<aris::core::Msg>::recvInNrt(aris::core::Msg &msg)
		{
			int err = recvInNrtRawData(msg.data_, sizeof(aris::core::MsgHeader));
			msg.resize(msg.size());
			recvInNrtRawData(msg.data(), msg.size());
			return msg.size() + sizeof(aris::core::MsgHeader);
		}
	}
}
