#include <Platform.h>
#ifdef PLATFORM_IS_WINDOWS
#include <ecrt_windows_py.h>//just for IDE vs2015, it does not really work
#endif
#ifdef PLATFORM_IS_LINUX
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

#include <Aris_EtherCat.h>


namespace Aris
{
	namespace Control
	{
#ifdef PLATFORM_IS_LINUX
		class PipeBase::Imp
		{
		public:
			Imp(int port, bool isBlock)
			{
				InitRT(port);
				InitNRT(port, isBlock);
			}
			
		private:
			void InitRT(int port)
			{
				if ((FD_RT = rt_dev_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP)) < 0)
				{
					throw std::runtime_error(std::string("RT data communication failed! Port:") + std::to_string(port));
				}

				struct timeval tv;
				tv.tv_sec = 0;  /* 30 Secs Timeout */
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
			}
			void InitNRT(int port, bool isBlock)
			{
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
			}

			int FD_RT;
			int FD_NRT;
			char* FD_NRT_DEVNAME;

			std::recursive_mutex mutexInNRT;

			friend class PipeBase;
		};

		PipeBase::PipeBase(int port, bool isBlock):pImp(new PipeBase::Imp(port, isBlock)){}
		PipeBase::~PipeBase(){}
		int PipeBase::SendToRT_RawData(const void *pData, int size)
		{
			if (pData == nullptr)
			{
				throw std::runtime_error("SendNRTtoRT:Invalid pointer");
			}
			std::lock_guard<std::recursive_mutex> guard(pImp->mutexInNRT);
			return write(pImp->FD_NRT, pData, size);
		}
		int PipeBase::SendToNRT_RawData(const void* pData, int size)
		{
			if (pData == nullptr)
			{
				throw std::runtime_error("SendRTtoNRT:Invalid pointer");
			}
			return rt_dev_sendto(pImp->FD_RT, pData, size, 0, NULL, 0);;
		}
		int PipeBase::RecvInRT_RawData(void* pData, int size)
		{
			if (pData == nullptr)
			{
				throw std::runtime_error("RecvRTfromNRT:Invalid pointer");
			}
			return rt_dev_recvfrom(pImp->FD_RT, pData, size, MSG_DONTWAIT, NULL, 0);
		}
		int PipeBase::RecvInNRT_RawData(void *pData, int size)
		{
			if (pData == nullptr)
			{
				throw std::runtime_error("RecvNRTfromRT:Invalid pointer");
			}
			std::lock_guard<std::recursive_mutex> guard(pImp->mutexInNRT);
			return read(pImp->FD_NRT, pData, size);
		}
		
		Pipe<Aris::Core::Msg>::Pipe(int port, bool isBlock) :PipeBase(port, isBlock)
		{
		}
		int Pipe<Aris::Core::Msg>::SendToRT(const Aris::Core::Msg &msg)
		{
			SendToRT_RawData(msg._pData, msg.GetLength() + sizeof(Aris::Core::MsgHeader));
			return msg.GetLength() + sizeof(Aris::Core::MsgHeader);
		}
		int Pipe<Aris::Core::Msg>::SendToNRT(const Aris::Core::MsgRT &msg)
		{
			SendToNRT_RawData(msg._pData, msg.GetLength() + sizeof(Aris::Core::MsgHeader));
			return msg.GetLength() + sizeof(Aris::Core::MsgHeader);
		}
		int Pipe<Aris::Core::Msg>::RecvInRT(Aris::Core::MsgRT &msg)
		{
			int length = RecvInRT_RawData(msg._pData, sizeof(Aris::Core::MsgHeader)+Aris::Core::MsgRT::RT_MSG_LENGTH);		
			return length<=0?0:length;			
		}
		int Pipe<Aris::Core::Msg>::RecvInNRT(Aris::Core::Msg &msg)
		{
						
			int err = RecvInNRT_RawData(msg._pData, sizeof(Aris::Core::MsgHeader));
			msg.SetLength(msg.GetLength());
			RecvInNRT_RawData(msg.GetDataAddress(), msg.GetLength());
			return msg.GetLength() + sizeof(Aris::Core::MsgHeader);
		}
#endif
		
	}
}
