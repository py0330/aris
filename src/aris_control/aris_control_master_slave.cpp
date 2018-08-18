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
		struct Slave::Imp { std::uint16_t phy_id_, sla_id_; };
		auto Slave::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);
			xml_ele.SetAttribute("phy_id", std::to_string(phyId()).c_str());
		}
		auto Slave::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Object::loadXml(xml_ele);
			imp_->phy_id_ = attributeUint16(xml_ele, "phy_id");
		}
		auto Slave::phyId()const->std::uint16_t { return imp_->phy_id_; }
		Slave::~Slave() = default;
		Slave::Slave(const std::string &name, std::uint16_t phy_id) :Object(name), imp_(new Imp) { imp_->phy_id_ = phy_id; }
		Slave::Slave(const Slave &other) = default;
		Slave::Slave(Slave &&other) = default;
		Slave& Slave::operator=(const Slave &other) = default;
		Slave& Slave::operator=(Slave &&other) = default;

		struct Master::Imp
		{
		public:
			enum { LOG_NEW_FILE = 1 };
			static auto rt_task_func(void *master)->void
			{
				auto &mst = *reinterpret_cast<Master*>(master);

				aris_rt_task_set_periodic(mst.imp_->sample_period_ns_);

				while (mst.imp_->is_rt_thread_running_)
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

					// flush lout
					mst.lout().update();
					if (!mst.imp_->lout_msg_.empty())
					{
						// 补充一个0作为结尾 //
						mst.lout() << '\0';
						mst.lout().update();

						mst.imp_->lout_pipe_->sendMsg(mst.imp_->lout_msg_);
						mst.imp_->lout_msg_.resize(0);
						mst.lout().resetBuf();
					}

					// flush mout
					mst.mout().update();
					if (!mst.imp_->mout_msg_.empty())
					{
						// 补充一个0作为结尾 //
						mst.mout() << '\0';
						mst.mout().update();
						
						mst.imp_->mout_pipe_->sendMsg(mst.imp_->mout_msg_);
						mst.imp_->mout_msg_.resize(0);
						mst.mout().resetBuf();
					}
				}

				mst.imp_->is_mout_thread_running_ = false;
			}

			// slave //
			aris::core::ObjectPool<Slave> *slave_pool_;
			std::vector<Size> sla_vec_phy2abs_;

			// for mout and lout //
			aris::core::Pipe *mout_pipe_, *lout_pipe_;
			aris::core::MsgFix<MAX_MSG_SIZE> mout_msg_, lout_msg_;
			std::unique_ptr<aris::core::MsgStream> mout_msg_stream_, lout_msg_stream_;
			std::thread mout_thread_;

			// strategy //
			std::function<void()> strategy_{ nullptr };

			// running flag //
			std::mutex mu_running_;
			std::atomic_bool is_rt_thread_running_{ false };
			std::atomic_bool is_mout_thread_running_{ false };

			const int sample_period_ns_{ 1000000 };

			aris::core::ImpPtr<Handle> rt_task_handle_;
			aris::core::ImpPtr<Handle> ec_handle_;
			
			Imp() { mout_msg_stream_.reset(new aris::core::MsgStream(mout_msg_)); lout_msg_stream_.reset(new aris::core::MsgStream(lout_msg_));}

			friend class Slave;
			friend class Master;
		};
		auto Master::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Object::loadXml(xml_ele);

			imp_->slave_pool_ = findByName("slave_pool") == children().end() ? &add<aris::core::ObjectPool<Slave, Object> >("slave_pool") : static_cast<aris::core::ObjectPool<Slave, Object> *>(&(*findByName("slave_pool")));
			imp_->mout_pipe_ = findOrInsert<aris::core::Pipe>("mout_pipe");
			imp_->lout_pipe_ = findOrInsert<aris::core::Pipe>("lout_pipe");
		}
		auto Master::start()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_rt_thread_running_)throw std::runtime_error("master already running, so cannot start");
			imp_->is_rt_thread_running_ = true;

			// make vec_phy2abs //
			for (auto &sla : slavePool())
			{
				imp_->sla_vec_phy2abs_.clear();
				imp_->sla_vec_phy2abs_.resize(std::max(static_cast<aris::Size>(sla.phyId() + 1), imp_->sla_vec_phy2abs_.size()), -1);
				if (imp_->sla_vec_phy2abs_.at(sla.phyId()) != -1) throw std::runtime_error("invalid Master::Slave phy id:\"" + std::to_string(sla.phyId()) + "\" of slave \"" + sla.name() + "\" already exists");
				imp_->sla_vec_phy2abs_.at(sla.phyId()) = sla.id();
			}

			// init child master //
			init();
			
			// lock memory // 
			aris_mlockall();

			// create mout & lout thread //
			imp_->is_mout_thread_running_ = true;
			imp_->mout_thread_ = std::thread([this]() 
			{
				// prepair lout //
				auto file_name = aris::core::logDirPath() + "rt_log--" + aris::core::logFileTimeFormat(std::chrono::system_clock::now()) + "--";
				std::fstream file;
				file.open(file_name + ".txt", std::ios::out | std::ios::trunc);

				// start read mout and lout //
				aris::core::Msg msg;
				while (imp_->is_mout_thread_running_)
				{
					if (imp_->lout_pipe_->recvMsg(msg))
					{
						if (msg.msgID() == Imp::LOG_NEW_FILE)
						{
							file.close();
							file.open(file_name + msg.data() + ".txt", std::ios::out | std::ios::trunc);
						}
						else if (!msg.empty()) 
						{
							file << msg.data();
						}
					}
					else if (imp_->mout_pipe_->recvMsg(msg))
					{
						if (!msg.empty())std::cout << msg.data() << std::endl;
					}
					else
					{
						std::this_thread::sleep_for(std::chrono::milliseconds(1));
					}
				}

				// 结束前最后一次接收，此时实时线程已经结束 //
				while (imp_->mout_pipe_->recvMsg(msg))if (!msg.empty())std::cout << msg.data() << std::endl;
				while (imp_->lout_pipe_->recvMsg(msg)) 
				{
					if (msg.msgID() == Imp::LOG_NEW_FILE)
					{
						file.close();
						file.open(file_name + msg.data() + ".txt", std::ios::out | std::ios::trunc);
					}
					else if (!msg.empty())
					{
						file << msg.data();
					}
				}
			});

			// create and start rt task //
			imp_->rt_task_handle_.reset(aris_rt_task_create());
			if (imp_->rt_task_handle_.get() == nullptr) throw std::runtime_error("rt_task_create failed");
			if (aris_rt_task_start(imp_->rt_task_handle_.get(), &Imp::rt_task_func, this))throw std::runtime_error("rt_task_start failed");
		}
		auto Master::stop()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (!imp_->is_rt_thread_running_)throw std::runtime_error("master is not running, so can't stop");
			imp_->is_rt_thread_running_ = false;

			// join rt task //
			if (aris_rt_task_join(rtHandle()))throw std::runtime_error("aris_rt_task_join failed");
			
			// join mout task //
			imp_->mout_thread_.join();

			// release child resources //
			release();
		}
		auto Master::setControlStrategy(std::function<void()> strategy)->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_rt_thread_running_)throw std::runtime_error("master already running, cannot set control strategy");
			imp_->strategy_ = strategy;
		}
		auto Master::rtHandle()->Handle* { return imp_->rt_task_handle_.get(); }
		auto Master::logFile(const char *file_name)->void
		{
			// 将已有的log数据发送过去 //
			if (!imp_->lout_msg_.empty())
			{
				// 补充一个0作为结尾 //
				lout() << '\0';
				lout().update();

				imp_->lout_pipe_->sendMsg(imp_->lout_msg_);
				imp_->lout_msg_.resize(0);
				lout().resetBuf();
			}

			// 发送切换文件的msg //
			imp_->lout_msg_.setMsgID(Imp::LOG_NEW_FILE);
			imp_->lout_msg_.copy(file_name);
			imp_->lout_pipe_->sendMsg(imp_->lout_msg_);

			// 将msg变更回去
			imp_->lout_msg_.setMsgID(0);
			imp_->lout_msg_.resize(0);
		}
		auto Master::lout()->aris::core::MsgStream & { return *imp_->lout_msg_stream_; }
		auto Master::mout()->aris::core::MsgStream & { return *imp_->mout_msg_stream_; }
		auto Master::slaveAtPhy(aris::Size id)->Slave& { return slavePool().at(imp_->sla_vec_phy2abs_.at(id)); }
		auto Master::slavePool()->aris::core::ObjectPool<Slave, aris::core::Object>& { return *imp_->slave_pool_; }
		Master::~Master() = default;
		Master::Master(const std::string &name) :imp_(new Imp), Object(name)
		{
			registerType<Slave>();
			registerType<aris::core::ObjectPool<Slave> >();

			imp_->slave_pool_ = &add<aris::core::ObjectPool<Slave> >("slave_pool");
			imp_->mout_pipe_ = &add<aris::core::Pipe>("mout_pipe");
			imp_->lout_pipe_ = &add<aris::core::Pipe>("lout_pipe");
		}
    }
}
