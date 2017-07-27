#include <string>
#include <iostream>
#include <map>
#include <fstream>
#include <algorithm>
#include <mutex>
#include <thread>
#include <future>

#include "aris_control_rt_timer.h"
#include "aris_control_master_slave.h"


namespace aris
{
	namespace control
	{
		struct RTTimer::Imp { int sample_period_ns_{ 1000000 }; };
		auto RTTimer::saveXml(aris::core::XmlElement &xml_ele) const->void { Object::saveXml(xml_ele); }
		RTTimer::~RTTimer() = default;
		RTTimer::RTTimer(const std::string &name) :Object(name), imp_(new Imp)
		{
		}
		RTTimer::RTTimer(Object &father, const aris::core::XmlElement &xml_ele) : Object(father, xml_ele), imp_(new Imp)
		{
		}
		
		struct RTLogger::Imp
		{
			aris::core::Pipe *log_pipe_;
			aris::core::MsgFix<MAX_LOG_DATA_SIZE> log_msg_;

			std::unique_ptr<aris::core::MsgStream> log_msg_stream_;

			std::thread log_thread_;

			std::mutex mu_running_;
			std::atomic_bool is_running_;

			Imp() :log_msg_(), is_running_(false) { log_msg_stream_.reset(new aris::core::MsgStream(log_msg_)); }
		};
		auto RTLogger::saveXml(aris::core::XmlElement &xml_ele) const->void { Object::saveXml(xml_ele); }
		auto RTLogger::start(const std::string &log_file_name)->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("failed to start RTLogger, because it's running");
			imp_->is_running_ = true;

			aris::core::createLogDir();
			auto file_name = aris::core::logDirPath() + (log_file_name.empty() ? "logdata_" + aris::core::logFileTimeFormat(std::chrono::system_clock::now()) + ".txt" : log_file_name);

			std::promise<void> thread_ready;
			auto fut = thread_ready.get_future();
			imp_->log_thread_ = std::thread([this, file_name](std::promise<void> thread_ready)
			{
				std::fstream file;
				file.open(file_name.c_str(), std::ios::out | std::ios::trunc);

				aris::core::MsgFix<MAX_LOG_DATA_SIZE> recv_msg;

				thread_ready.set_value();

				long long count = 0;
				while (imp_->is_running_)
				{
					if (imp_->log_pipe_->recvMsg(recv_msg))
					{
						file << recv_msg.data();
					}
					else
					{
						std::this_thread::sleep_for(std::chrono::microseconds(10));
					}
				}

				// clean pipe //
				while (imp_->log_pipe_->recvMsg(recv_msg))file << recv_msg.data();
				file.close();
			}, std::move(thread_ready));

			fut.wait();
		}
		auto RTLogger::stop()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (!imp_->is_running_)throw std::runtime_error("failed to stop RTLogger, because it's not running");
			imp_->is_running_ = false;
			imp_->log_thread_.join();
		}
		auto RTLogger::lout()->aris::core::MsgStream & { return *imp_->log_msg_stream_; }
		auto RTLogger::send()->void
		{
			lout().update();
			if (!imp_->log_msg_.empty())
			{
				lout() << '\0';
				lout().update();
				imp_->log_pipe_->sendMsg(imp_->log_msg_);
				imp_->log_msg_.resize(0);
				lout().resetBuf();
			}
		}
		RTLogger::~RTLogger() = default;
		RTLogger::RTLogger(const std::string &name) :Object(name), imp_(new Imp)
		{
			imp_->log_pipe_ = &add<aris::core::Pipe>("pipe", 16384);
		}
		RTLogger::RTLogger(Object &father, const aris::core::XmlElement &xml_ele) : Object(father, xml_ele), imp_(new Imp)
		{
			imp_->log_pipe_ = findOrInsert<aris::core::Pipe>("pipe", 16384);
		}

		struct NSlave::Imp { Size col_id_, blk_col_id_; };
		NSlave::~NSlave() = default;
		NSlave::NSlave(const std::string &name) :Object(name) {}
		NSlave::NSlave(Object &father, const aris::core::XmlElement &xml_ele) :Object(father, xml_ele) {}
		NSlave::NSlave(const NSlave &other) = default;
		NSlave::NSlave(NSlave &&other) = default;
		NSlave& NSlave::operator=(const NSlave &other) = default;
		NSlave& NSlave::operator=(NSlave &&other) = default;

		class NMaster::Imp
		{
		public:
			static auto rt_task_func(void *master)->void
			{
				auto &mst = *reinterpret_cast<NMaster*>(master);

				aris_rt_task_set_periodic(mst.imp_->sample_period_ns_);

				while (mst.imp_->is_running_)
				{
					// rt timer //
					aris_rt_task_wait_period();

					// receive //
					mst.recv();

					// tragectory generator //
					if (mst.imp_->strategy_)mst.imp_->strategy_();

					// sync
					mst.sync();

					// send
					mst.send();
				}
			}

			// slave //
			aris::core::ObjectPool<NSlave, Object> *slave_pool_;

			// for log //
			RTLogger* data_logger_;

			// for msg in and out //
			aris::core::Pipe *pipe_in_;
			aris::core::Pipe *pipe_out_;
			aris::core::MsgFix<MAX_MSG_SIZE> out_msg_, in_msg_;
			std::unique_ptr<aris::core::MsgStream> out_msg_stream_;

			// strategy //
			std::function<void()> strategy_{ nullptr };

			// running flag //
			std::mutex mu_running_;
			std::atomic_bool is_running_{ false };

			const int sample_period_ns_{ 1000000 };

			aris::core::ImpPtr<Handle> rt_task_handle_;
			aris::core::ImpPtr<Handle> ec_handle_;


			Imp() { out_msg_stream_.reset(new aris::core::MsgStream(out_msg_)); }

			friend class NSlave;
			friend class NMaster;
		};
		auto NMaster::loadXml(const aris::core::XmlDocument &xml_doc)->void
		{
			auto root_xml_ele = xml_doc.RootElement()->FirstChildElement("controller");

			if (!root_xml_ele)throw std::runtime_error("can't find controller element in xml file");

			loadXml(*root_xml_ele);
		}
		auto NMaster::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Root::loadXml(xml_ele);

			//imp_->slave_type_pool_ = findByName("slave_type_pool") == children().end() ? &add<aris::core::ObjectPool<SlaveType, Object> >("slave_type_pool") : static_cast<aris::core::ObjectPool<SlaveType, Object> *>(&(*findByName("slave_type_pool")));
			imp_->slave_pool_ = findByName("slave_pool") == children().end() ? &add<aris::core::ObjectPool<NSlave, Object> >("slave_pool") : static_cast<aris::core::ObjectPool<NSlave, Object> *>(&(*findByName("slave_pool")));
			imp_->data_logger_ = findByName("data_logger") == children().end() ? &add<RTLogger>("data_logger") : static_cast<RTLogger*>(&(*findByName("data_logger")));
			imp_->pipe_in_ = findOrInsert<aris::core::Pipe>("msg_pipe_in");
			imp_->pipe_out_ = findOrInsert<aris::core::Pipe>("msg_pipe_out");
		}
		auto NMaster::start()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("master already running, so cannot start");
			imp_->is_running_ = true;

			// lock memory// 
			aris_mlockall();

			// create and start rt task //
			imp_->rt_task_handle_.reset(aris_rt_task_create());
			if (imp_->rt_task_handle_.get() == nullptr) throw std::runtime_error("rt_task_create failed");
			if (aris_rt_task_start(imp_->rt_task_handle_.get(), &Imp::rt_task_func, this))throw std::runtime_error("rt_task_start failed");
		}
		auto NMaster::stop()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (!imp_->is_running_)throw std::runtime_error("master is not running, so can't stop");
			imp_->is_running_ = false;

			if (aris_rt_task_join(rtHandle()))throw std::runtime_error("aris_rt_task_join failed");
			//aris_ecrt_master_stop(ecHandle());
		}
		auto NMaster::setControlStrategy(std::function<void()> strategy)->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("master already running, cannot set control strategy");
			imp_->strategy_ = strategy;
		}
		auto NMaster::rtHandle()->Handle* { return imp_->rt_task_handle_.get(); }
		auto NMaster::msgIn()->aris::core::MsgFix<MAX_MSG_SIZE>& { return imp_->in_msg_; }
		auto NMaster::msgOut()->aris::core::MsgFix<MAX_MSG_SIZE>& { return imp_->out_msg_; }
		auto NMaster::mout()->aris::core::MsgStream & { return *imp_->out_msg_stream_; }
		auto NMaster::sendOut()->void
		{
			if (!imp_->out_msg_.empty())
			{
				imp_->pipe_out_->sendMsg(imp_->out_msg_);
				imp_->out_msg_.resize(0);
				mout().resetBuf();
			}
		}
		auto NMaster::recvOut(aris::core::MsgBase &recv_msg)->int { return imp_->pipe_out_->recvMsg(recv_msg); }
		auto NMaster::sendIn(const aris::core::MsgBase &send_msg)->void { imp_->pipe_in_->sendMsg(send_msg); }
		auto NMaster::recvIn()->int { return imp_->pipe_in_->recvMsg(imp_->in_msg_); }
		auto NMaster::slavePool()->aris::core::ObjectPool<NSlave, aris::core::Object>& { return *imp_->slave_pool_; }
		auto NMaster::dataLogger()->RTLogger& { return *imp_->data_logger_; }
		NMaster::~NMaster() = default;
		NMaster::NMaster() :imp_(new Imp)
		{
			registerChildType<RTTimer>();
			registerChildType<RTLogger>();
			registerChildType<NSlave>();

			imp_->slave_pool_ = &add<aris::core::ObjectPool<NSlave, Object> >("slave_pool");
			imp_->data_logger_ = &add<RTLogger>("data_logger");
			imp_->pipe_in_ = &add<aris::core::Pipe>("msg_pipe_in");
			imp_->pipe_out_ = &add<aris::core::Pipe>("msg_pipe_out");
		}
    }
}
