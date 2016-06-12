#ifdef WIN32
#include <ecrt_windows_py.h>//just for IDE vs2015, it does not really work



#endif
#ifdef UNIX
#include <ecrt.h>
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>
#include <sys/mman.h>
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
#include <sstream>
#include <map>
#include <atomic>
#include <memory>
#include <typeinfo>

#include "aris_control_ethercat.h"

namespace aris
{
	namespace control
	{
		struct Slave::Imp
		{
		public:
			auto read()->void
			{
#ifdef UNIX 
				ecrt_domain_process(domain_);
#endif
			};
			auto write()->void
			{
#ifdef UNIX
				ecrt_domain_queue(domain_);
#endif
			};
			auto init()->void;
			Imp(Slave*slave) :slave_(slave) {};

			std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec_;
			std::vector<ec_pdo_info_t> ec_pdo_info_vec_tx_, ec_pdo_info_vec_rx_;
			ec_sync_info_t ec_sync_info_[5];
			ec_slave_config_t* ec_slave_config_;

			ec_domain_t* domain_;
			std::uint8_t* domain_pd_;

			SlaveType *slave_type_;

			aris::core::ObjectPool<PdoGroup, Element> *pdo_group_pool_;
			aris::core::ObjectPool<Sdo, Element> *sdo_pool_;
			std::map<std::uint16_t, std::map<std::uint8_t, std::pair<int, int> > > pdo_map_;

			Slave *slave_;
			RxType rx_data_;
			TxType tx_data_;

			friend class Master::Imp;
			friend class Slave;
			friend class Master;
		};
		class Master::Imp
		{
		public:
			static auto rt_task_func(void *)->void
			{
			#ifdef UNIX
				auto &mst = *instance;
			
				rt_task_set_periodic(NULL, TM_NOW, mst.imp_->sample_period_ns_);
			
				while (!mst.imp_->is_stopping_)
				{
					rt_task_wait_period(NULL);
			
					mst.imp_->read();//motors and sensors get data
			
					/// tg begin
					mst.controlStrategy();
					/// tg end
			
					mst.imp_->sync(rt_timer_read());
					mst.imp_->write();//motor data write and state machine/mode transition

                    mst.imp_->logToNrt();//sent data to nrt
				}
			#endif
			};
			static auto setInstance(Master *master)->void
			{
				instance = master;
			}
			auto read()->void 
			{ 
				ecrt_master_receive(ec_master_);
				for (auto &sla : *slave_pool_) 
				{
					sla.readUpdate();
					sla.imp_->read();
				}
			};
			auto write()->void 
			{ 
				for (auto &sla : *slave_pool_) 
				{
					sla.writeUpdate();
					sla.imp_->write();
				}
				ecrt_master_send(ec_master_); 
            }

            auto logToNrt()->void
            {
                std::size_t size_count=0;
                for(std::size_t i=0;i<slave_pool_->size();i++)
                {
                    auto &sla=slave_pool_->at(i);
                    auto tx_data=reinterpret_cast<char *>(&tx_data_pool_.at(i));
                    auto rx_data=reinterpret_cast<char *>(&rx_data_pool_.at(i));
                    std::copy(tx_data,tx_data+sla.txTypeSize(),sent_data_.get()+size_count);
                    size_count=size_count+sla.txTypeSize();
                    std::copy(rx_data,rx_data+sla.rxTypeSize(),sent_data_.get()+size_count);
                    size_count=size_count+sla.rxTypeSize();
                }
                record_pipe_->sendToNrt(sent_data_.get(),data_size_);
            }
            auto logToFile()->void
            {
                if(!record_thread_.joinable())
                record_thread_ = std::thread([this]()
                {
                    static std::fstream file;
                    std::string name = aris::core::logFileName();
                    name.replace(name.rfind("log.txt"), std::strlen("data.txt"), "data.txt");
                    file.open(name.c_str(), std::ios::out | std::ios::trunc);

                    long long count = -1;
                    std::unique_ptr<char[]> txdata;
                    std::unique_ptr<char[]> rxdata;
					std::unique_ptr<char[]> receive_data(new char [data_size_]);
                    while (!is_stopping_)
                    {
                        record_pipe_->recvInNrt(receive_data.get(),data_size_);

                        file << ++count << " ";

                        std::size_t size_count=0;
                        for (auto &sla : *slave_pool_)
                        {
							sla.logData(*reinterpret_cast<Slave::TxType *>(receive_data.get() + size_count)
								, *reinterpret_cast<Slave::RxType *>(receive_data.get() + size_count + sla.txTypeSize()), file);
							
							file << " ";

							size_count += sla.txTypeSize() + sla.rxTypeSize();
                        }
                        file << std::endl;
                    }
                    file.close();
                });
            }

            auto sync(uint64_t ns)->void
			{
				ecrt_master_application_time(ec_master_, ns);
				ecrt_master_sync_reference_clock(ec_master_);
				ecrt_master_sync_slave_clocks(ec_master_);
			};

			static Master *instance;

			aris::core::ObjectPool<SlaveType, Element> *slave_type_pool_;
			aris::core::ObjectPool<Slave, Element> *slave_pool_;
			aris::core::RefPool<Slave::TxType> tx_data_pool_;
			aris::core::RefPool<Slave::RxType> rx_data_pool_;

			ec_master_t* ec_master_;
			const int sample_period_ns_ = 1000000;

            //for log
            std::size_t data_size_{0};
            std::unique_ptr<char[]> sent_data_;
            std::unique_ptr<Pipe<void *>> record_pipe_;
            std::thread record_thread_;

			std::atomic_bool is_running_{ false }, is_stopping_{ false };

#ifdef UNIX
			RT_TASK rt_task_;
#endif
			friend class Slave;
			friend class Master;
		};
		Master *Master::Imp::instance{ nullptr };

		auto Element::master()->Master &{return static_cast<Master &>(root());}
		auto Element::master()const->const Master &{ return static_cast<const Master &>(root()); }

        auto DO::slave()->Slave& { return *slave_; }
        auto DO::slave()const->const Slave&{ return *slave_; }

		auto Pdo::read(std::int32_t &value)const->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType()!=INT32) throw std::runtime_error("can not read pdo with wrong data type");
			value = *reinterpret_cast<const std::int32_t*>(slave().imp_->domain_pd_ + offset_);
		}
		auto Pdo::read(std::int16_t &value)const->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT16) throw std::runtime_error("can not read pdo with wrong data type");
			value = *reinterpret_cast<const std::int16_t*>(slave().imp_->domain_pd_ + offset_);
		}
		auto Pdo::read(std::int8_t &value)const->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT8) throw std::runtime_error("can not read pdo with wrong data type");
			value = *reinterpret_cast<const std::int8_t*>(slave().imp_->domain_pd_ + offset_);
		}
		auto Pdo::read(std::uint32_t &value)const->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT32) throw std::runtime_error("can not read pdo with wrong data type");
			value = *reinterpret_cast<const std::uint32_t*>(slave().imp_->domain_pd_ + offset_);
		}
		auto Pdo::read(std::uint16_t &value)const->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT16) throw std::runtime_error("can not read pdo with wrong data type");
			value = *reinterpret_cast<const std::uint16_t*>(slave().imp_->domain_pd_ + offset_);
		}
		auto Pdo::read(std::uint8_t &value)const->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT8) throw std::runtime_error("can not read pdo with wrong data type");
			value = *reinterpret_cast<const std::uint8_t*>(slave().imp_->domain_pd_ + offset_);
		}
		auto Pdo::write(std::int32_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT32) throw std::runtime_error("can not read pdo with wrong data type");
			*reinterpret_cast<std::int32_t*>(slave().imp_->domain_pd_ + offset_) = value;
		}
		auto Pdo::write(std::int16_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT16) throw std::runtime_error("can not read pdo with wrong data type");
			*reinterpret_cast<std::int16_t*>(slave().imp_->domain_pd_ + offset_) = value;
		}
		auto Pdo::write(std::int8_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT8) throw std::runtime_error("can not read pdo with wrong data type");
			*reinterpret_cast<std::int8_t*>(slave().imp_->domain_pd_ + offset_) = value;
		}
		auto Pdo::write(std::uint32_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT32) throw std::runtime_error("can not read pdo with wrong data type");
			*reinterpret_cast<std::uint32_t*>(slave().imp_->domain_pd_ + offset_) = value;
		}
		auto Pdo::write(std::uint16_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT16) throw std::runtime_error("can not read pdo with wrong data type");
			*reinterpret_cast<std::uint16_t*>(slave().imp_->domain_pd_ + offset_) = value;
		}
		auto Pdo::write(std::uint8_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT8) throw std::runtime_error("can not read pdo with wrong data type");
			*reinterpret_cast<std::uint8_t*>(slave().imp_->domain_pd_ + offset_) = value;
		}

		struct Sdo::Imp
		{
			unsigned option_;
			union
			{
				char config_value_[8];
				std::uint32_t config_value_uint32_;
				std::uint16_t config_value_uint16_;
				std::uint8_t config_value_uint8_;
				std::int32_t config_value_int32_;
				std::int16_t config_value_int16_;
				std::int8_t config_value_int8_;
			};
		};
		auto Sdo::readable()const->bool { return (imp_->option_ & READ) != 0; }
		auto Sdo::writeable()const->bool { return (imp_->option_ & WRITE) != 0; }
		auto Sdo::configurable()const->bool { return (imp_->option_ & CONFIG) != 0; }
		auto Sdo::option()const->unsigned { return imp_->option_; }
		auto Sdo::getConfigValue(std::int32_t &value)const->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != INT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int32 type");
			value = imp_->config_value_int32_;
		}
		auto Sdo::getConfigValue(std::int16_t &value)const->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != INT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int16 type");
			value = imp_->config_value_int16_;
		}
		auto Sdo::getConfigValue(std::int8_t &value)const->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != INT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int8 type");
			value = imp_->config_value_int8_;
		}
		auto Sdo::getConfigValue(std::uint32_t &value)const->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != UINT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint32 type");
			value = imp_->config_value_uint32_;
		}
		auto Sdo::getConfigValue(std::uint16_t &value)const->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != UINT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint16 type");
			value = imp_->config_value_uint32_;
		}
		auto Sdo::getConfigValue(std::uint8_t &value)const->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != UINT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint8 type");
			value = imp_->config_value_uint32_;
		}
		auto Sdo::setConfigValue(std::int32_t value)->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != INT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int32 type");
			imp_->config_value_int32_ = value;
		}
		auto Sdo::setConfigValue(std::int16_t value)->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != INT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int16 type");
			imp_->config_value_int16_ = value;
		}
		auto Sdo::setConfigValue(std::int8_t value)->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != INT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int8 type");
			imp_->config_value_int8_ = value;
		}
		auto Sdo::setConfigValue(std::uint32_t value)->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != UINT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint32 type");
			imp_->config_value_uint32_ = value;
		}
		auto Sdo::setConfigValue(std::uint16_t value)->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != UINT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint16 type");
			imp_->config_value_uint16_ = value;
		}
		auto Sdo::setConfigValue(std::uint8_t value)->void
		{
			if (!configurable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not configurable");
			if (dataType() != UINT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint8 type");
			imp_->config_value_uint8_ = value;
		}
		auto Sdo::read(std::int32_t &value)const->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != INT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int32 type");
			
			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_upload(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &real_size, &abort_code);
#endif
		}
		auto Sdo::read(std::int16_t &value)const->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != INT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int16 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_upload(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &real_size, &abort_code);
#endif
		}
		auto Sdo::read(std::int8_t &value)const->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != INT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int8 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_upload(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &real_size, &abort_code);
#endif
		}
		auto Sdo::read(std::uint32_t &value)const->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != UINT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint32 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_upload(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &real_size, &abort_code);
#endif
		}
		auto Sdo::read(std::uint16_t &value)const->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != UINT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint16 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_upload(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &real_size, &abort_code);
#endif
		}
		auto Sdo::read(std::uint8_t &value)const->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != UINT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint8 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_upload(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &real_size, &abort_code);
#endif
		}
		auto Sdo::write(std::int32_t value)->void 
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != INT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int32 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_download(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &abort_code);
#endif
		}
		auto Sdo::write(std::int16_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != INT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int16 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_download(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &abort_code);
#endif
		}
		auto Sdo::write(std::int8_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != INT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int8 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_download(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &abort_code);
#endif
		}
		auto Sdo::write(std::uint32_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != UINT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint32 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_download(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &abort_code);
#endif
		}
		auto Sdo::write(std::uint16_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != UINT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint16 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_download(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &abort_code);
#endif
		}
		auto Sdo::write(std::uint8_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != UINT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint8 type");

			auto *ec_mst = master().imp_->ec_master_;
			std::size_t real_size;
			std::uint32_t abort_code;
#ifdef UNIX
			ecrt_master_sdo_download(ec_mst, slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataSize(), &abort_code);
#endif
		}
		Sdo::Sdo(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :DO(father, id, xml_ele)
		{
			if (attributeBool(xml_ele, "read", true))imp_->option_ |= READ; else imp_->option_ &= ~READ;
			if (attributeBool(xml_ele, "write", true))imp_->option_ |= WRITE; else imp_->option_ &= ~WRITE;
			if (xml_ele.Attribute("config"))
			{
				if (!writeable())throw std::runtime_error("you can't config data in unwriteable sdo, error in \"" + std::string(xml_ele.name()) + "\" sdo");
				imp_->option_ |= CONFIG;
				switch (data_type_)
				{
				case aris::control::DO::INT32:
					imp_->config_value_int32_ = attributeInt32(xml_ele, "config");
					break;
				case aris::control::DO::INT16:
					imp_->config_value_int16_ = attributeInt16(xml_ele, "config");
					break;
				case aris::control::DO::INT8:
					imp_->config_value_int8_ = attributeInt8(xml_ele, "config");
					break;
				case aris::control::DO::UINT32:
					imp_->config_value_uint32_ = attributeUint32(xml_ele, "config");
					break;
				case aris::control::DO::UINT16:
					imp_->config_value_uint16_ = attributeUint16(xml_ele, "config");
					break;
				case aris::control::DO::UINT8:
					imp_->config_value_uint8_ = attributeUint8(xml_ele, "config");
					break;
				default:
					throw std::runtime_error("failed to get sdo config value");
					break;
				}
			}
		}

		struct PdoGroup::Imp
		{
			bool is_tx_;
			std::uint16_t index_;
			std::vector<ec_pdo_entry_info_t> ec_pdo_entry_info_vec_;
		};
		auto PdoGroup::tx()const->bool { return imp_->is_tx_; }
		auto PdoGroup::rx()const->bool { return !imp_->is_tx_; }
		auto PdoGroup::index()const->std::uint16_t { return imp_->index_; }
		PdoGroup::PdoGroup(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ObjectPool(father, id, xml_ele)
		{
			imp_->index_ = attributeUint16(xml_ele, "index");
			imp_->is_tx_ = attributeBool(xml_ele, "is_tx");
		}
		
		struct SlaveType::Imp
		{
			std::uint32_t product_code_, vender_id_;
			std::uint16_t alias_;
			std::uint32_t distributed_clock_;
		};
		auto SlaveType::productCode()const->std::uint32_t { return imp_->product_code_; }
		auto SlaveType::venderID()const->std::uint32_t { return imp_->vender_id_; }
		auto SlaveType::alias()const->std::uint16_t { return imp_->alias_; }
		auto SlaveType::distributedClock()const->std::uint32_t { return imp_->distributed_clock_; }
		SlaveType::SlaveType(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :Element(father, id, xml_ele)
		{
			//load product id...
			imp_->product_code_ = attributeUint32(xml_ele, "product_code");
			imp_->vender_id_ = attributeUint32(xml_ele, "vender_id");
			imp_->alias_ = attributeUint16(xml_ele, "alias");
			imp_->distributed_clock_ = attributeUint32(xml_ele, "distributed_clock", 0);
		}

		auto Slave::Imp::init()->void
		{
			auto &ec_mst =	slave_->master().imp_->ec_master_;

#ifdef UNIX
			for (auto &reg : this->ec_pdo_entry_reg_vec_)	reg.position = slave_->position();

			// Create domain
			if (!(this->domain_ = ecrt_master_create_domain(ec_mst)))throw std::runtime_error("failed to create domain");

			// Get the slave configuration 
			if (!(this->ec_slave_config_ = ecrt_master_slave_config(ec_mst, slave_type_->alias(), slave_->position(), slave_type_->venderID(), slave_type_->productCode())))
			{
				throw std::runtime_error("failed to slave config");
			}

			// Config Sdo
			for (auto &sdo : slave_->sdoPool())
			{
				if (!(sdo.option() & Sdo::CONFIG)) continue;

				switch (sdo.dataSize())
				{
				case 8:		ecrt_slave_config_sdo8(this->ec_slave_config_, sdo.index(), sdo.subindex(), sdo.imp_->config_value_uint8_); break;
				case 16:	ecrt_slave_config_sdo16(this->ec_slave_config_, sdo.index(), sdo.subindex(), sdo.imp_->config_value_uint16_); break;
				case 32:	ecrt_slave_config_sdo32(this->ec_slave_config_, sdo.index(), sdo.subindex(), sdo.imp_->config_value_uint32_); break;
				default:    throw std::runtime_error("invalid size of sdo, it must be 8, 16 or 32");
				}
			}

			// Configure the slave's PDOs and sync masters
			if (ecrt_slave_config_pdos(this->ec_slave_config_, 4, this->ec_sync_info_))throw std::runtime_error("failed to slave config pdos");

			// Configure the slave's domain
			if (ecrt_domain_reg_pdo_entry_list(this->domain_, this->ec_pdo_entry_reg_vec_.data()))throw std::runtime_error("failed domain_reg_pdo_entry");

			// Configure the slave's discrete clock			
			if (slave_type_->distributedClock())ecrt_slave_config_dc(this->ec_slave_config_, slave_type_->distributedClock(), 1000000, 4400000, 0, 0);

#endif
		};
		auto Slave::txData()->TxType& { return imp_->tx_data_; }
		auto Slave::txData()const->const TxType&{ return imp_->tx_data_; }
		auto Slave::rxData()->RxType& { return imp_->rx_data_; }
		auto Slave::rxData()const->const RxType&{ return imp_->rx_data_; }
		auto Slave::pdoGroupPool()->aris::core::ObjectPool<PdoGroup, Element>& { return *imp_->pdo_group_pool_; }
		auto Slave::pdoGroupPool()const->const aris::core::ObjectPool<PdoGroup, Element>&{return *imp_->pdo_group_pool_; }
		auto Slave::sdoPool()->aris::core::ObjectPool<Sdo, Element>& { return *imp_->sdo_pool_; }
		auto Slave::sdoPool()const->const aris::core::ObjectPool<Sdo, Element>&{return *imp_->sdo_pool_; }
		auto Slave::readPdo(int pdo_group_id, int pdo_id, std::int8_t &value) const->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).read(value);
		}
		auto Slave::readPdo(int pdo_group_id, int pdo_id, std::int16_t &value) const->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).read(value);
		}
		auto Slave::readPdo(int pdo_group_id, int pdo_id, std::int32_t &value) const->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).read(value);
		}
		auto Slave::readPdo(int pdo_group_id, int pdo_id, std::uint8_t &value) const->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).read(value);
		}
		auto Slave::readPdo(int pdo_group_id, int pdo_id, std::uint16_t &value) const->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).read(value);
		}
		auto Slave::readPdo(int pdo_group_id, int pdo_id, std::uint32_t &value) const->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).read(value);
		}
		auto Slave::writePdo(int pdo_group_id, int pdo_id, std::int8_t value)->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).write(value);
		}
		auto Slave::writePdo(int pdo_group_id, int pdo_id, std::int16_t value)->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).write(value);
		}
		auto Slave::writePdo(int pdo_group_id, int pdo_id, std::int32_t value)->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).write(value);
		}
		auto Slave::writePdo(int pdo_group_id, int pdo_id, std::uint8_t value)->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).write(value);
		}
		auto Slave::writePdo(int pdo_group_id, int pdo_id, std::uint16_t value)->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).write(value);
		}
		auto Slave::writePdo(int pdo_group_id, int pdo_id, std::uint32_t value)->void
		{
			pdoGroupPool().at(pdo_group_id).at(pdo_id).write(value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t &value)const->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			readPdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t &value)const->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			readPdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t &value)const->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			readPdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t &value)const->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			readPdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t &value)const->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			readPdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t &value)const->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			readPdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			writePdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			writePdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			writePdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			writePdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			writePdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			writePdo(id_pair.first, id_pair.second, value);
		}
		auto Slave::readSdoConfig(int sdoID, std::int8_t &value) const->void { sdoPool().at(sdoID).getConfigValue(value);}
		auto Slave::readSdoConfig(int sdoID, std::int16_t &value) const->void { sdoPool().at(sdoID).getConfigValue(value); }
		auto Slave::readSdoConfig(int sdoID, std::int32_t &value) const->void { sdoPool().at(sdoID).getConfigValue(value); }
		auto Slave::readSdoConfig(int sdoID, std::uint8_t &value) const->void { sdoPool().at(sdoID).getConfigValue(value); }
		auto Slave::readSdoConfig(int sdoID, std::uint16_t &value) const->void { sdoPool().at(sdoID).getConfigValue(value); }
		auto Slave::readSdoConfig(int sdoID, std::uint32_t &value) const->void { sdoPool().at(sdoID).getConfigValue(value); }
		auto Slave::configSdo(int sdoID, std::int8_t value)->void {	sdoPool().at(sdoID).setConfigValue(value);}
		auto Slave::configSdo(int sdoID, std::int16_t value)->void { sdoPool().at(sdoID).setConfigValue(value); }
		auto Slave::configSdo(int sdoID, std::int32_t value)->void { sdoPool().at(sdoID).setConfigValue(value); }
		auto Slave::configSdo(int sdoID, std::uint8_t value)->void { sdoPool().at(sdoID).setConfigValue(value); }
		auto Slave::configSdo(int sdoID, std::uint16_t value)->void { sdoPool().at(sdoID).setConfigValue(value); }
		auto Slave::configSdo(int sdoID, std::uint32_t value)->void { sdoPool().at(sdoID).setConfigValue(value); }
		auto Slave::readSdo(int sdoID, std::int8_t &value) const->void { sdoPool().at(sdoID).read(value); }
		auto Slave::readSdo(int sdoID, std::int16_t &value) const->void { sdoPool().at(sdoID).read(value); }
		auto Slave::readSdo(int sdoID, std::int32_t &value) const->void { sdoPool().at(sdoID).read(value); }
		auto Slave::readSdo(int sdoID, std::uint8_t &value) const->void { sdoPool().at(sdoID).read(value); }
		auto Slave::readSdo(int sdoID, std::uint16_t &value) const->void { sdoPool().at(sdoID).read(value); }
		auto Slave::readSdo(int sdoID, std::uint32_t &value) const->void { sdoPool().at(sdoID).read(value); }
		auto Slave::writeSdo(int sdoID, std::int8_t value)->void { sdoPool().at(sdoID).write(value); }
		auto Slave::writeSdo(int sdoID, std::int16_t value)->void { sdoPool().at(sdoID).write(value); }
		auto Slave::writeSdo(int sdoID, std::int32_t value)->void { sdoPool().at(sdoID).write(value); }
		auto Slave::writeSdo(int sdoID, std::uint8_t value)->void { sdoPool().at(sdoID).write(value); }
		auto Slave::writeSdo(int sdoID, std::uint16_t value)->void { sdoPool().at(sdoID).write(value); }
		auto Slave::writeSdo(int sdoID, std::uint32_t value)->void { sdoPool().at(sdoID).write(value); }
		Slave::~Slave() {}
		Slave::Slave(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :Element(father, id, xml_ele), imp_(new Imp(this))
		{
			auto &slave_type_pool = static_cast<aris::core::ObjectPool<SlaveType, Element> &>(*master().findByName("SlaveType"));

			if (slave_type_pool.findByName(attributeString(xml_ele, "slave_type")) == slave_type_pool.end())
			{
				throw std::runtime_error("can not find slave_type \"" + attributeString(xml_ele, "slave_type") + "\" in slave \"" + name() + "\"");
			}
			imp_->slave_type_ = static_cast<SlaveType*>(&add(std::ref(*slave_type_pool.findByName(attributeString(xml_ele, "slave_type")))));
			imp_->pdo_group_pool_ = static_cast<aris::core::ObjectPool<PdoGroup, Element> *>(&*imp_->slave_type_->findByName("PDO"));
			imp_->sdo_pool_ = static_cast<aris::core::ObjectPool<Sdo, Element> *>(&*imp_->slave_type_->findByName("SDO"));
			

			for (auto &group : pdoGroupPool())
			{
				for (auto &pdo : group)
				{
					pdo.slave_ = this;
				}
			}
			for (auto &sdo : sdoPool())
			{
				sdo.slave_ = this;
			}

			/// make PDO map ///
			for (int i = 0; i < static_cast<int>(pdoGroupPool().size()); ++i)
			{
				auto &group = pdoGroupPool().at(i);
				for (int j = 0; j < static_cast<int>(group.size()); ++j)
				{
					auto &pdo = group.at(j);

					if (imp_->pdo_map_.find(pdo.index_) != imp_->pdo_map_.end())
					{
						imp_->pdo_map_.at(pdo.index_).insert(std::make_pair(pdo.subindex_, std::make_pair(i, j)));
					}
					else
					{
						std::map<std::uint8_t, std::pair<int, int> > subindex_map;
						subindex_map.insert(std::make_pair(pdo.subindex_, std::make_pair(i, j)));
						imp_->pdo_map_.insert(std::make_pair(pdo.index_, subindex_map));
					}
				}
			}

			/// create ecrt structs  ///
			for (auto &pdo_group : pdoGroupPool())
			{
				for (auto &pdo : pdo_group)
				{
					imp_->ec_pdo_entry_reg_vec_.push_back(ec_pdo_entry_reg_t{ imp_->slave_type_->alias(), static_cast<std::uint16_t>(this->id()), imp_->slave_type_->venderID(), imp_->slave_type_->productCode(), pdo.index(), pdo.subindex(), &pdo.offset_ });
					pdo_group.imp_->ec_pdo_entry_info_vec_.push_back(ec_pdo_entry_info_t{ pdo.index(), pdo.subindex(), pdo.dataSize() });
				}
				
				if (pdo_group.tx())
				{
					imp_->ec_pdo_info_vec_tx_.push_back(ec_pdo_info_t{ pdo_group.index(), static_cast<std::uint8_t>(pdo_group.size()), pdo_group.imp_->ec_pdo_entry_info_vec_.data() });
				}
				else
				{
					imp_->ec_pdo_info_vec_rx_.push_back(ec_pdo_info_t{ pdo_group.index(), static_cast<std::uint8_t>(pdo_group.size()), pdo_group.imp_->ec_pdo_entry_info_vec_.data() });
				}
			}
			imp_->ec_pdo_entry_reg_vec_.push_back(ec_pdo_entry_reg_t{});

			imp_->ec_sync_info_[0] = ec_sync_info_t{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE };
			imp_->ec_sync_info_[1] = ec_sync_info_t{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE };
			imp_->ec_sync_info_[2] = ec_sync_info_t{ 2, EC_DIR_OUTPUT, static_cast<unsigned int>(imp_->ec_pdo_info_vec_rx_.size()), imp_->ec_pdo_info_vec_rx_.data(), EC_WD_ENABLE };
			imp_->ec_sync_info_[3] = ec_sync_info_t{ 3, EC_DIR_INPUT, static_cast<unsigned int>(imp_->ec_pdo_info_vec_tx_.size()),imp_->ec_pdo_info_vec_tx_.data(), EC_WD_ENABLE };
			imp_->ec_sync_info_[4] = ec_sync_info_t{ 0xff };
		}

		auto Master::loadXml(const aris::core::XmlDocument &xml_doc)->void
		{
			auto sensor_root_xml_ele = xml_doc.RootElement()->FirstChildElement("Controller");

			if (!sensor_root_xml_ele)throw std::runtime_error("can't find SensorRoot element in xml file");

			loadXml(*sensor_root_xml_ele);
		}
		auto Master::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Root::loadXml(xml_ele);

			imp_->slave_type_pool_ = findByName("SlaveType") == end() ? &add<aris::core::ObjectPool<SlaveType, Element> >("SlaveType") : static_cast<aris::core::ObjectPool<SlaveType, Element> *>(&(*findByName("SlaveType")));
			imp_->slave_pool_ = findByName("Slave") == end() ? &add<aris::core::ObjectPool<Slave, Element> >("Slave") : static_cast<aris::core::ObjectPool<Slave, Element> *>(&(*findByName("Slave")));
			imp_->tx_data_pool_.clear();
			imp_->rx_data_pool_.clear();
			for (auto &slave : slavePool())
			{
				imp_->tx_data_pool_.push_back_ptr(&slave.txData());
				imp_->rx_data_pool_.push_back_ptr(&slave.rxData());
			}
		}
		auto Master::start()->void
		{
			if (imp_->is_running_)throw std::runtime_error("master already running");
			imp_->is_running_ = true;
			imp_->setInstance(this);

			/// init begin ///
#ifdef UNIX
			if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) { throw std::runtime_error("lock failed"); }

			imp_->ec_master_ = ecrt_request_master(0);
			if (!imp_->ec_master_) { throw std::runtime_error("master request failed!"); }

			/// init each slave and update tx & rx data pool ///
			txDataPool().clear();
			rxDataPool().clear();
			for (auto &slave : slavePool())
			{
				slave.imp_->init();
				slave.init();
				txDataPool().push_back_ptr(&slave.txData());
				rxDataPool().push_back_ptr(&slave.rxData());
                imp_->data_size_= imp_->data_size_+slave.txTypeSize();
                imp_->data_size_= imp_->data_size_+slave.rxTypeSize();
			}
            imp_->sent_data_.reset(new char[imp_->data_size_]);
            imp_->record_pipe_.reset(new Pipe<void *>());

			ecrt_master_activate(imp_->ec_master_);

			for (auto &slave : slavePool())
			{
				slave.imp_->domain_pd_ = ecrt_domain_data(slave.imp_->domain_);
			}

            imp_->logToFile();
			/// init end ///
			rt_print_auto_init(1);
			const int priority = 99;

			rt_task_create(&imp_->rt_task_, "realtime core", 0, priority, T_FPU);
			rt_task_start(&imp_->rt_task_, &Master::Imp::rt_task_func, NULL);
#endif
		};
		auto Master::stop()->void
		{
			if (!imp_->is_running_)throw std::runtime_error("master is not running, so can't stop");

			imp_->is_stopping_ = true;
#ifdef UNIX
			rt_task_delete(&imp_->rt_task_);

			ecrt_master_deactivate(imp_->ec_master_);
			ecrt_release_master(imp_->ec_master_);
#endif
			imp_->is_stopping_ = false;
			imp_->is_running_ = false;
		}
		auto Master::slaveTypePool()->aris::core::ObjectPool<SlaveType, Element>&{ return *imp_->slave_type_pool_; }
		auto Master::slaveTypePool()const->const aris::core::ObjectPool<SlaveType, Element>&{ return *imp_->slave_type_pool_; }
		auto Master::slavePool()->aris::core::ObjectPool<Slave, Element>& { return *imp_->slave_pool_; }
		auto Master::slavePool()const->const aris::core::ObjectPool<Slave, Element>&{ return *imp_->slave_pool_; }
		auto Master::txDataPool()->aris::core::RefPool<Slave::TxType> &{return imp_->tx_data_pool_;}
		auto Master::txDataPool()const->const aris::core::RefPool<Slave::TxType> &{return imp_->tx_data_pool_; }
		auto Master::rxDataPool()->aris::core::RefPool<Slave::RxType> & { return imp_->rx_data_pool_; }
		auto Master::rxDataPool()const->const aris::core::RefPool<Slave::RxType> &{return imp_->rx_data_pool_; }
		Master::~Master() {}
		Master::Master() :imp_(new Imp)
		{
			registerChildType<Pdo>();
			registerChildType<Sdo>();
			registerChildType<PdoGroup>();
			registerChildType<aris::core::ObjectPool<Sdo, Element> >();
			registerChildType<aris::core::ObjectPool<PdoGroup, Element> >();
			registerChildType<aris::core::ObjectPool<SlaveType, Element> >();

			registerChildType<SlaveType>();
			registerChildType<Slave, false, false, false, false>();
			registerChildType<aris::core::ObjectPool<Slave, Element> >();
		}
	}
}
