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
#include <memory>
#include <atomic>

#include "aris_control_ethercat.h"


namespace aris
{
	namespace control
	{
		/*
		class PipeBase::Imp
		{
		public:
			Imp(bool isBlock)
			{
				static int port = 0;
				InitRT(port);
                InitNrt(port, isBlock);
				++port;
			}
			
		private:
            auto InitRT(int port)->void
			{
#ifdef UNIX
                if ((fd_rt_ = rt_dev_socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP)) < 0)
				{
					throw std::runtime_error(std::string("RT data communication failed! Port:") + std::to_string(port));
				}

				struct timeval tv;
                tv.tv_sec = 1;  // 30 Secs Timeout //
                tv.tv_usec = 0;
                if (rt_dev_setsockopt(fd_rt_, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)))
				{
					throw std::runtime_error(std::string("RT data communication failed! Port:") + std::to_string(port));
				}

                //
                // Set a local 16k pool for the RT endpoint. Memory needed to
                // convey datagrams will be pulled from this pool, instead of
                // Xenomai's system pool.
                //
                const std::size_t poolsz = 16384; // bytes //
                if (rt_dev_setsockopt(fd_rt_, SOL_XDDP, XDDP_POOLSZ, &poolsz, sizeof(poolsz)))
				{
					throw std::runtime_error(std::string("RT data communication failed! Port:") + std::to_string(port));
				}

                //
                // Bind the socket to the port, to setup a proxy to channel
                // traffic to/from the Linux domain.
                //
                // saddr.sipc_port specifies the port number to use.
                //
				struct sockaddr_ipc saddr;
				memset(&saddr, 0, sizeof(saddr));
				saddr.sipc_family = AF_RTIPC;
				saddr.sipc_port = port;
                if (rt_dev_bind(fd_rt_, (struct sockaddr *)&saddr, sizeof(saddr)))
				{
					throw std::runtime_error(std::string("RT pipe Init failed! Port:") + std::to_string(port));
				}
#endif
			}
            auto InitNrt(int port, bool is_block)->void
			{
#ifdef UNIX
                if(fd_nrt_ = open(("/dev/rtp" + std::to_string(port)).c_str(), O_RDWR));
                if (fd_nrt_ < 0)throw std::runtime_error(std::string("NRT pipe Init failed! Port:") + std::to_string(port));

                int flags = fcntl(fd_nrt_, F_GETFL, 0);
                fcntl(fd_nrt_, F_SETFL, is_block?flags &~O_NONBLOCK : flags | O_NONBLOCK);
#endif
			}

            int fd_rt_;
            int fd_nrt_;

            std::recursive_mutex mutex_nrt_;

			friend class PipeBase;
		};
		PipeBase::PipeBase(bool is_block):imp_(new PipeBase::Imp(is_block)){}
		PipeBase::~PipeBase(){}
		auto PipeBase::sendToRTRawData(const void *data, int size)->int
		{
#ifdef UNIX
            std::lock_guard<std::recursive_mutex> guard(imp_->mutex_nrt_);
            return write(imp_->fd_nrt_, data, size);
#endif
#ifdef WIN32
			return 0;
#endif
		}
		auto PipeBase::sendToNrtRawData(const void* data, int size)->int
		{
#ifdef UNIX
            return rt_dev_sendto(imp_->fd_rt_, data, size, 0, NULL, 0);
#endif
#ifdef WIN32
			return 0;
#endif
		}
		auto PipeBase::recvInRTRawData(void* data, int size)->int
		{
#ifdef UNIX
            return rt_dev_recvfrom(imp_->fd_rt_, data, size, MSG_DONTWAIT, NULL, 0);
#endif
#ifdef WIN32
			return 0;
#endif
		}
		auto PipeBase::recvInNrtRawData(void *data, int size)->int
		{
#ifdef UNIX
            std::lock_guard<std::recursive_mutex> guard(imp_->mutex_nrt_);
            return read(imp_->fd_nrt_, data, size);
#endif
#ifdef WIN32
			return 0;
#endif
		}
		
		Pipe<aris::core::Msg>::Pipe(bool is_block) :PipeBase(is_block) {}
		auto Pipe<aris::core::Msg>::sendToRT(const aris::core::Msg &msg)->int
		{
			sendToRTRawData(msg.data_.get(), msg.size() + sizeof(aris::core::MsgHeader));
			return msg.size() + sizeof(aris::core::MsgHeader);
		}
		auto Pipe<aris::core::Msg>::sendToNrt(const aris::core::MsgRT &msg)->int
		{
			sendToNrtRawData(msg.data_, msg.size() + sizeof(aris::core::MsgHeader));
			return msg.size() + sizeof(aris::core::MsgHeader);
		}
		auto Pipe<aris::core::Msg>::recvInRT(aris::core::MsgRT &msg)->int
		{
			int length = recvInRTRawData(msg.data_, sizeof(aris::core::MsgHeader) + aris::core::MsgRT::RT_MSG_SIZE);
			return length <= 0 ? 0 : length;
		}
		auto Pipe<aris::core::Msg>::recvInNrt(aris::core::Msg &msg)->int
		{
			int err = recvInNrtRawData(msg.data_.get(), sizeof(aris::core::MsgHeader));
			msg.resize(msg.size());
			recvInNrtRawData(msg.data(), msg.size());
			return msg.size() + sizeof(aris::core::MsgHeader);
		}
		*/
	}
}
