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
		auto RTTimer::loadXml(const aris::core::XmlElement &xml_ele)->void { Object::loadXml(xml_ele); }
		RTTimer::~RTTimer() = default;
		RTTimer::RTTimer(const std::string &name) :Object(name), imp_(new Imp)
		{
		}
		
		struct DataLogger::Imp
		{
			aris::core::Pipe *log_pipe_;
			aris::core::MsgFix<MAX_LOG_DATA_SIZE> log_msg_;

			std::unique_ptr<aris::core::MsgStream> log_msg_stream_;

			std::thread log_thread_;

			std::mutex mu_running_;
			std::atomic_bool is_running_;

			Imp() :log_msg_(), is_running_(false) { log_msg_stream_.reset(new aris::core::MsgStream(log_msg_)); }
		};
		auto DataLogger::saveXml(aris::core::XmlElement &xml_ele) const->void { Object::saveXml(xml_ele); }
		auto DataLogger::loadXml(const aris::core::XmlElement &xml_ele)->void 
		{ 
			Object::loadXml(xml_ele);
			imp_->log_pipe_ = findOrInsert<aris::core::Pipe>("pipe", 16384);
		}
		auto DataLogger::start(const std::string &log_file_name)->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("failed to start DataLogger, because it's running");
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
		auto DataLogger::stop()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (!imp_->is_running_)throw std::runtime_error("failed to stop DataLogger, because it's not running");
			imp_->is_running_ = false;
			imp_->log_thread_.join();
		}
		auto DataLogger::lout()->aris::core::MsgStream & { return *imp_->log_msg_stream_; }
		auto DataLogger::send()->void
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
		DataLogger::~DataLogger() = default;
		DataLogger::DataLogger(const std::string &name) :Object(name), imp_(new Imp)
		{
			imp_->log_pipe_ = &add<aris::core::Pipe>("pipe", 16384);
		}

		struct Slave::Imp
		{
		public:
			const SlaveType *slave_type_;
			std::uint16_t phy_id_, sla_id_;
		};
		auto Slave::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);
			if(slaveType())	xml_ele.SetAttribute("slave_type", slaveType()->name().c_str());

			xml_ele.SetAttribute("phy_id", std::to_string(phyId()).c_str());
		}
		auto Slave::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Object::loadXml(xml_ele);
			imp_->slave_type_ = nullptr;
			if (!attributeString(xml_ele, "slave_type", "").empty())
			{
				if (root().findByName("slave_type_pool") == root().children().end())throw std::runtime_error("you must insert \"slave_type_pool\" before insert \"slave_pool\" node");
				auto &slave_type_pool = static_cast<aris::core::ObjectPool<SlaveType> &>(*root().findByName("slave_type_pool"));

				if (slave_type_pool.findByName(attributeString(xml_ele, "slave_type")) == slave_type_pool.end())
				{
					throw std::runtime_error("can not find slave_type \"" + attributeString(xml_ele, "slave_type") + "\" in slave \"" + name() + "\"");
				}
				imp_->slave_type_ = &*slave_type_pool.findByName(attributeString(xml_ele, "slave_type"));
			}

			imp_->phy_id_ = attributeUint16(xml_ele, "phy_id");
		}
		auto Slave::slaveType()const->const SlaveType *{ return imp_->slave_type_; }
		auto Slave::phyId()const->std::uint16_t { return imp_->phy_id_; }
		Slave::~Slave() = default;
		Slave::Slave(const std::string &name, const SlaveType *slave_type, std::uint16_t phy_id) :Object(name), imp_(new Imp) 
		{
			imp_->slave_type_ = slave_type;
			imp_->phy_id_ = phy_id;

		}
		Slave::Slave(const Slave &other) = default;
		Slave::Slave(Slave &&other) = default;
		Slave& Slave::operator=(const Slave &other) = default;
		Slave& Slave::operator=(Slave &&other) = default;

		struct Master::Imp
		{
		public:
			static auto rt_task_func(void *master)->void
			{
				auto &mst = *reinterpret_cast<Master*>(master);

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
			aris::core::ObjectPool<SlaveType> *slave_type_pool_;
			aris::core::ObjectPool<Slave> *slave_pool_;
			std::vector<Size> sla_vec_phy2abs_;

			// for log //
			DataLogger* data_logger_;

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

			friend class Slave;
			friend class Master;
		};
		auto Master::loadXml(const aris::core::XmlDocument &xml_doc)->void
		{
			auto root_xml_ele = xml_doc.RootElement()->FirstChildElement("controller");

			if (!root_xml_ele)throw std::runtime_error("can't find controller element in xml file");

			loadXml(*root_xml_ele);
		}
		auto Master::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Root::loadXml(xml_ele);

			imp_->slave_type_pool_ = findByName("slave_type_pool") == children().end() ? &add<aris::core::ObjectPool<SlaveType, Object> >("slave_type_pool") : static_cast<aris::core::ObjectPool<SlaveType, Object> *>(&(*findByName("slave_type_pool")));
			imp_->slave_pool_ = findByName("slave_pool") == children().end() ? &add<aris::core::ObjectPool<Slave, Object> >("slave_pool") : static_cast<aris::core::ObjectPool<Slave, Object> *>(&(*findByName("slave_pool")));
			imp_->data_logger_ = findByName("data_logger") == children().end() ? &add<DataLogger>("data_logger") : static_cast<DataLogger*>(&(*findByName("data_logger")));
			imp_->pipe_in_ = findOrInsert<aris::core::Pipe>("msg_pipe_in");
			imp_->pipe_out_ = findOrInsert<aris::core::Pipe>("msg_pipe_out");
		}
		auto Master::start()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("master already running, so cannot start");
			imp_->is_running_ = true;


			// make mot_vec_phy2abs //
			for (auto &sla : slavePool())
			{
				imp_->sla_vec_phy2abs_.resize(std::max(static_cast<aris::Size>(sla.phyId() + 1), imp_->sla_vec_phy2abs_.size()), -1);
				if (imp_->sla_vec_phy2abs_.at(sla.phyId()) != -1) throw std::runtime_error("invalid Master::Slave phy id:\"" + std::to_string(sla.phyId()) + "\" of slave \"" + sla.name() + "\" already exists");
				imp_->sla_vec_phy2abs_.at(sla.phyId()) = sla.id();
			}

			// init child master //
			init();
			
			// lock memory // 
			aris_mlockall();

			// create and start rt task //
			imp_->rt_task_handle_.reset(aris_rt_task_create());
			if (imp_->rt_task_handle_.get() == nullptr) throw std::runtime_error("rt_task_create failed");
			if (aris_rt_task_start(imp_->rt_task_handle_.get(), &Imp::rt_task_func, this))throw std::runtime_error("rt_task_start failed");
		}
		auto Master::stop()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (!imp_->is_running_)throw std::runtime_error("master is not running, so can't stop");
			imp_->is_running_ = false;

			// join task //
			if (aris_rt_task_join(rtHandle()))throw std::runtime_error("aris_rt_task_join failed");
			
			// release child resources //
			release();
		}
		auto Master::setControlStrategy(std::function<void()> strategy)->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("master already running, cannot set control strategy");
			imp_->strategy_ = strategy;
		}
		auto Master::rtHandle()->Handle* { return imp_->rt_task_handle_.get(); }
		auto Master::msgIn()->aris::core::MsgFix<MAX_MSG_SIZE>& { return imp_->in_msg_; }
		auto Master::msgOut()->aris::core::MsgFix<MAX_MSG_SIZE>& { return imp_->out_msg_; }
		auto Master::mout()->aris::core::MsgStream & { return *imp_->out_msg_stream_; }
		auto Master::sendOut()->void
		{
			if (!imp_->out_msg_.empty())
			{
				imp_->pipe_out_->sendMsg(imp_->out_msg_);
				imp_->out_msg_.resize(0);
				mout().resetBuf();
			}
		}
		auto Master::recvOut(aris::core::MsgBase &recv_msg)->int { return imp_->pipe_out_->recvMsg(recv_msg); }
		auto Master::sendIn(const aris::core::MsgBase &send_msg)->void { imp_->pipe_in_->sendMsg(send_msg); }
		auto Master::recvIn()->int { return imp_->pipe_in_->recvMsg(imp_->in_msg_); }
		auto Master::slaveAtPhy(aris::Size id)->Slave& { return slavePool().at(imp_->sla_vec_phy2abs_.at(id)); }
		auto Master::slaveTypePool()->aris::core::ObjectPool<SlaveType>& { return *imp_->slave_type_pool_; }
		auto Master::slavePool()->aris::core::ObjectPool<Slave, aris::core::Object>& { return *imp_->slave_pool_; }
		auto Master::dataLogger()->DataLogger& { return *imp_->data_logger_; }
		Master::~Master() = default;
		Master::Master(const std::string &name) :imp_(new Imp), Root(name)
		{
			registerChildType<RTTimer>();
			registerChildType<DataLogger>();
			registerChildType<SlaveType>();
			registerChildType<aris::core::ObjectPool<SlaveType> >();
			registerChildType<Slave>();
			registerChildType<aris::core::ObjectPool<Slave> >();

			imp_->slave_type_pool_ = &add<aris::core::ObjectPool<SlaveType> >("slave_type_pool");
			imp_->slave_pool_ = &add<aris::core::ObjectPool<Slave> >("slave_pool");
			imp_->data_logger_ = &add<DataLogger>("data_logger");
			imp_->pipe_in_ = &add<aris::core::Pipe>("msg_pipe_in");
			imp_->pipe_out_ = &add<aris::core::Pipe>("msg_pipe_out");
		}
    }
}
