#include <mutex>
#include <string>
#include <iostream>
#include <sstream>
#include <map>
#include <atomic>
#include <memory>
#include <typeinfo>
#include <thread>
#include <chrono>
#include <future>

#include "aris_control_kernel.h"
#include "aris_control_ethercat.h"

namespace aris
{
	namespace control
	{
		auto Element::master()->Master & { return static_cast<Master &>(root()); }
		auto Element::master()const->const Master &{ return static_cast<const Master &>(root()); }

		struct DataLogger::Imp
		{
			aris::core::Pipe *log_pipe_;
			aris::core::MsgFix<MAX_LOG_DATA_SIZE> log_data_msg_;
			std::size_t log_data_size_{ 0 };
			std::atomic_bool is_receiving_{ false }, is_sending_{ false }, is_prepaired_{ false };
			std::mutex mu_prepair_, mu_running_;
		};
		auto DataLogger::prepair(const std::string &log_file_name)->void
		{
			std::unique_lock<std::mutex> prepair_lck(imp_->mu_prepair_);
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_, std::try_to_lock);
			if (!running_lck.owns_lock())throw std::runtime_error("failed to prepair pipe, because it's started already");
			running_lck.unlock();
			running_lck.release();

			auto file_name = aris::core::logDirPath() + (log_file_name.empty() ? "logdata_" + aris::core::logFileTimeFormat(std::chrono::system_clock::now()) + ".txt" : log_file_name);

			std::promise<void> thread_ready;
			auto fut = thread_ready.get_future();

			std::thread([this, file_name](std::promise<void> thread_ready)
			{
				std::unique_lock<std::mutex> running_lck(imp_->mu_running_);

				std::fstream file;
				file.open(file_name.c_str(), std::ios::out | std::ios::trunc);

				imp_->log_data_size_ = 0;
				for (auto &sla : master().slavePool()) imp_->log_data_size_ += sla.txTypeSize() + sla.rxTypeSize();
				imp_->log_data_msg_.resize(imp_->log_data_size_);

				aris::core::MsgFix<MAX_LOG_DATA_SIZE> recv_msg;
				while (imp_->log_pipe_->recvMsg(recv_msg));
				imp_->is_receiving_ = true;
				thread_ready.set_value();

				long long count = 0;
				while (imp_->is_receiving_)
				{
					if (imp_->log_pipe_->recvMsg(recv_msg))
					{
						file << count++ << " ";

						std::size_t size_count = 0;
						for (auto &sla : master().slavePool())
						{
							sla.logData(*reinterpret_cast<Slave::TxType *>(recv_msg.data() + size_count)
								, *reinterpret_cast<Slave::RxType *>(recv_msg.data() + size_count + sla.txTypeSize()), file);

							file << " ";

							size_count += sla.txTypeSize() + sla.rxTypeSize();
						}
						file << std::endl;
					}
					else
					{
						std::this_thread::sleep_for(std::chrono::microseconds(10));
					}
				}
				file.close();
			}, std::move(thread_ready)).detach();

			fut.wait();

			imp_->is_prepaired_ = true;
		}
		auto DataLogger::start()->void
		{
			if (imp_->is_prepaired_)
			{
				imp_->is_sending_ = true;
				imp_->is_prepaired_ = false;
			}
		}
		auto DataLogger::stop()->void
		{
			imp_->is_sending_ = false;
			imp_->is_receiving_ = false;
		}
		auto DataLogger::logDataRT()->void
		{
			if (imp_->is_sending_)
			{
				std::size_t size_count = 0;
				for (auto &sla : master().slavePool())
				{
					auto tx_data_char = reinterpret_cast<char *>(&sla.txData());
					auto rx_data_char = reinterpret_cast<char *>(&sla.rxData());

					std::copy(tx_data_char, tx_data_char + sla.txTypeSize(), imp_->log_data_msg_.data() + size_count);
					size_count += sla.txTypeSize();
					std::copy(rx_data_char, rx_data_char + sla.rxTypeSize(), imp_->log_data_msg_.data() + size_count);
					size_count += sla.rxTypeSize();
				}

				imp_->log_pipe_->sendMsg(imp_->log_data_msg_);
			}
		}
		DataLogger::~DataLogger() = default;
		DataLogger::DataLogger(const std::string &name) :Element(name), imp_(new Imp)
		{
			imp_->log_pipe_ = &add<aris::core::Pipe>("pipe", 16384);
		}
		DataLogger::DataLogger(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele), imp_(new Imp)
		{
			imp_->log_pipe_ = findOrInsert<aris::core::Pipe>("pipe", 16384);
		}

		struct DO::Imp
		{
			DataType data_type_;
			std::uint16_t index_;
			std::uint8_t subindex_;
			std::uint8_t data_bit_size_;
			Slave *slave_;
		};
		auto DO::slave()->Slave& { return *imp_->slave_; }
		auto DO::slave()const->const Slave&{ return *imp_->slave_; }
		auto DO::index()const->std::uint16_t { return imp_->index_; }
		auto DO::subindex()const->std::uint8_t { return imp_->subindex_; }
		auto DO::dataBit()const->std::uint8_t { return imp_->data_bit_size_; }
		auto DO::dataType()const->DataType { return imp_->data_type_; }
		DO::~DO() = default;
		DO::DO(Object &father, const aris::core::XmlElement &xml_ele) :Element(father, xml_ele)
		{
			imp_->index_ = attributeUint16(xml_ele, "index");
			imp_->subindex_ = attributeUint8(xml_ele, "subindex");

			if (!xml_ele.Attribute("datatype"))throw std::runtime_error("Data Object in slave must have \"datatype\" attribute");
			else if (xml_ele.Attribute("datatype", "int32"))
			{
				imp_->data_type_ = INT32;
				imp_->data_bit_size_ = 32;
			}
			else if (xml_ele.Attribute("datatype", "int16"))
			{
				imp_->data_type_ = INT16;
				imp_->data_bit_size_ = 16;
			}
			else if (xml_ele.Attribute("datatype", "int8"))
			{
				imp_->data_type_ = INT8;
				imp_->data_bit_size_ = 8;
			}
			else if (xml_ele.Attribute("datatype", "uint32"))
			{
				imp_->data_type_ = UINT32;
				imp_->data_bit_size_ = 32;
			}
			else if (xml_ele.Attribute("datatype", "uint16"))
			{
				imp_->data_type_ = UINT16;
				imp_->data_bit_size_ = 16;
			}
			else if (xml_ele.Attribute("datatype", "uint8"))
			{
				imp_->data_type_ = UINT8;
				imp_->data_bit_size_ = 8;
			}
			else
			{
				throw std::runtime_error("Data Object in slave has invalid \"datatype\" attribute");
			}
		}
		DO::DO(const DO &) = default;
		DO::DO(DO &&) = default;
		DO& DO::operator=(const DO &) = default;
		DO& DO::operator=(DO &&) = default;

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
		auto Sdo::configBuffer()->char* { return imp_->config_value_; };
		auto Sdo::configValueInt32()const->std::int32_t { return imp_->config_value_int32_; }
		auto Sdo::configValueInt16()const->std::int16_t { return imp_->config_value_int16_; }
		auto Sdo::configValueInt8()const->std::int8_t { return imp_->config_value_int8_; }
		auto Sdo::configValueUint32()const->std::uint32_t { return imp_->config_value_uint32_; }
		auto Sdo::configValueUint16()const->std::uint16_t { return imp_->config_value_uint16_; }
		auto Sdo::configValueUint8()const->std::uint8_t { return imp_->config_value_uint8_; }
		auto Sdo::getConfigValue(std::int32_t &value)const->void { value = imp_->config_value_int32_; }
		auto Sdo::getConfigValue(std::int16_t &value)const->void { value = imp_->config_value_int16_; }
		auto Sdo::getConfigValue(std::int8_t &value)const->void { value = imp_->config_value_int8_; }
		auto Sdo::getConfigValue(std::uint32_t &value)const->void { value = imp_->config_value_uint32_; }
		auto Sdo::getConfigValue(std::uint16_t &value)const->void { value = imp_->config_value_uint16_; }
		auto Sdo::getConfigValue(std::uint8_t &value)const->void { value = imp_->config_value_uint8_; }
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
		auto Sdo::read(std::int32_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != INT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int32 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &real_size, &abort_code);
		}
		auto Sdo::read(std::int16_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != INT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int16 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &real_size, &abort_code);
		}
		auto Sdo::read(std::int8_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != INT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int8 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &real_size, &abort_code);
		}
		auto Sdo::read(std::uint32_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != UINT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint32 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &real_size, &abort_code);
		}
		auto Sdo::read(std::uint16_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != UINT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint16 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &real_size, &abort_code);
		}
		auto Sdo::read(std::uint8_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != UINT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint8 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &real_size, &abort_code);
		}
		auto Sdo::write(std::int32_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != INT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int32 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &abort_code);
		}
		auto Sdo::write(std::int16_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != INT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int16 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &abort_code);
		}
		auto Sdo::write(std::int8_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != INT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int8 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &abort_code);
		}
		auto Sdo::write(std::uint32_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != UINT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint32 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &abort_code);
		}
		auto Sdo::write(std::uint16_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != UINT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint16 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &abort_code);
		}
		auto Sdo::write(std::uint8_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != UINT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint8 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBit(), &abort_code);
		}
		Sdo::~Sdo() = default;
		Sdo::Sdo(Object &father, const aris::core::XmlElement &xml_ele) :DO(father, xml_ele)
		{
			if (attributeBool(xml_ele, "read", true))imp_->option_ |= READ; else imp_->option_ &= ~READ;
			if (attributeBool(xml_ele, "write", true))imp_->option_ |= WRITE; else imp_->option_ &= ~WRITE;
			if (xml_ele.Attribute("config"))
			{
				if (!writeable())throw std::runtime_error("you can't config data in unwriteable sdo, error in \"" + std::string(xml_ele.name()) + "\" sdo");
				imp_->option_ |= CONFIG;
				switch (dataType())
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
		Sdo::Sdo(const Sdo &) = default;
		Sdo::Sdo(Sdo &&) = default;
		Sdo& Sdo::operator=(const Sdo &) = default;
		Sdo& Sdo::operator=(Sdo &&) = default;

		struct Pdo::Imp
		{
			aris::core::ImpPtr<Handle> ec_handle_;
		};
		auto Pdo::ecHandle()->Handle* { return imp_->ec_handle_.get(); };
		auto Pdo::ecHandle()const->const Handle*{ return imp_->ec_handle_.get(); };
		auto Pdo::read(std::int32_t &value)->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT32) throw std::runtime_error("can not read pdo with wrong data type");
			value = aris_ecrt_pdo_read_uint32(slave().ecHandle(), ecHandle());
		}
		auto Pdo::read(std::int16_t &value)->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT16) throw std::runtime_error("can not read pdo with wrong data type");
			value = aris_ecrt_pdo_read_int16(slave().ecHandle(), ecHandle());
		}
		auto Pdo::read(std::int8_t &value)->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT8) throw std::runtime_error("can not read pdo with wrong data type");
			value = aris_ecrt_pdo_read_int8(slave().ecHandle(), ecHandle());
		}
		auto Pdo::read(std::uint32_t &value)->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT32) throw std::runtime_error("can not read pdo with wrong data type");
			value = aris_ecrt_pdo_read_uint32(slave().ecHandle(), ecHandle());
		}
		auto Pdo::read(std::uint16_t &value)->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT16) throw std::runtime_error("can not read pdo with wrong data type");
			value = aris_ecrt_pdo_read_uint16(slave().ecHandle(), ecHandle());
		}
		auto Pdo::read(std::uint8_t &value)->void
		{
			if (!static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT8) throw std::runtime_error("can not read pdo with wrong data type");
			value = aris_ecrt_pdo_read_uint8(slave().ecHandle(), ecHandle());
		}
		auto Pdo::write(std::int32_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT32) throw std::runtime_error("can not read pdo with wrong data type");
			aris_ecrt_pdo_write_int32(slave().ecHandle(), ecHandle(), value);
		}
		auto Pdo::write(std::int16_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT16) throw std::runtime_error("can not read pdo with wrong data type");
			aris_ecrt_pdo_write_int16(slave().ecHandle(), ecHandle(), value);
		}
		auto Pdo::write(std::int8_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != INT8) throw std::runtime_error("can not read pdo with wrong data type");
			aris_ecrt_pdo_write_int8(slave().ecHandle(), ecHandle(), value);
		}
		auto Pdo::write(std::uint32_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT32) throw std::runtime_error("can not read pdo with wrong data type");
			aris_ecrt_pdo_write_uint32(slave().ecHandle(), ecHandle(), value);
		}
		auto Pdo::write(std::uint16_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT16) throw std::runtime_error("can not read pdo with wrong data type");
			aris_ecrt_pdo_write_uint16(slave().ecHandle(), ecHandle(), value);
		}
		auto Pdo::write(std::uint8_t value)->void
		{
			if (static_cast<const PdoGroup &>(father()).tx()) throw std::runtime_error("can not read pdo with rx type");
			if (dataType() != UINT8) throw std::runtime_error("can not read pdo with wrong data type");
			aris_ecrt_pdo_write_uint8(slave().ecHandle(), ecHandle(), value);
		}
		Pdo::~Pdo() = default;
		Pdo::Pdo(Object &father, const aris::core::XmlElement &xml_ele) :DO(father, xml_ele) {}
		Pdo::Pdo(const Pdo &) = default;
		Pdo::Pdo(Pdo &&) = default;
		Pdo& Pdo::operator=(const Pdo &) = default;
		Pdo& Pdo::operator=(Pdo &&) = default;

		struct PdoGroup::Imp
		{
			aris::core::ImpPtr<Handle> handle_;
			bool is_tx_;
			std::uint16_t index_;
		};
		auto PdoGroup::ecHandle()->Handle* { return imp_->handle_.get(); };
		auto PdoGroup::ecHandle()const->const Handle*{ return imp_->handle_.get(); };
		auto PdoGroup::tx()const->bool { return imp_->is_tx_; }
		auto PdoGroup::rx()const->bool { return !imp_->is_tx_; }
		auto PdoGroup::index()const->std::uint16_t { return imp_->index_; }
		PdoGroup::~PdoGroup() = default;
		PdoGroup::PdoGroup(Object &father, const aris::core::XmlElement &xml_ele) :ObjectPool(father, xml_ele)
		{
			imp_->index_ = attributeUint16(xml_ele, "index");
			imp_->is_tx_ = attributeBool(xml_ele, "is_tx");
		}
		PdoGroup::PdoGroup(const PdoGroup &) = default;
		PdoGroup::PdoGroup(PdoGroup &&) = default;
		PdoGroup& PdoGroup::operator=(const PdoGroup &) = default;
		PdoGroup& PdoGroup::operator=(PdoGroup &&) = default;

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
		SlaveType::~SlaveType() = default;
		SlaveType::SlaveType(Object &father, const aris::core::XmlElement &xml_ele) :Element(father, xml_ele)
		{
			imp_->product_code_ = attributeUint32(xml_ele, "product_code");
			imp_->vender_id_ = attributeUint32(xml_ele, "vender_id");
			imp_->alias_ = attributeUint16(xml_ele, "alias");
			imp_->distributed_clock_ = attributeUint32(xml_ele, "distributed_clock", 0);
		}
		SlaveType::SlaveType(const SlaveType &) = default;
		SlaveType::SlaveType(SlaveType &&) = default;
		SlaveType& SlaveType::operator=(const SlaveType &) = default;
		SlaveType& SlaveType::operator=(SlaveType &&) = default;

		struct Slave::Imp
		{
		public:
			Imp(Slave*slave) :slave_(slave) {}

			aris::core::ImpPtr<Handle> ec_handle_;

			SlaveType *slave_type_;

			aris::core::ObjectPool<PdoGroup, Element> *pdo_group_pool_;
			aris::core::ObjectPool<Sdo, Element> *sdo_pool_;
			std::map<std::uint16_t, std::map<std::uint8_t, std::pair<int, int> > > pdo_map_;
			std::map<std::uint16_t, std::map<std::uint8_t, int>> sdo_map_;

			Slave *slave_;
			RxType rx_data_;
			TxType tx_data_;

			friend class Master::Imp;
			friend class Slave;
			friend class Master;
		};
		auto Slave::ecHandle()->Handle* { return imp_->ec_handle_.get(); }
		auto Slave::ecHandle()const->const Handle*{ return imp_->ec_handle_.get(); }
		auto Slave::productCode()const->std::uint32_t { return imp_->slave_type_->productCode(); }
		auto Slave::venderID()const->std::uint32_t { return imp_->slave_type_->venderID(); }
		auto Slave::alias()const->std::uint16_t { return imp_->slave_type_->alias(); }
		auto Slave::distributedClock()const->std::uint32_t { return imp_->slave_type_->distributedClock(); }
		auto Slave::txData()->TxType& { return imp_->tx_data_; }
		auto Slave::txData()const->const TxType&{ return imp_->tx_data_; }
		auto Slave::rxData()->RxType& { return imp_->rx_data_; }
		auto Slave::rxData()const->const RxType&{ return imp_->rx_data_; }
		auto Slave::pdoGroupPool()->aris::core::ObjectPool<PdoGroup, Element>& { return *imp_->pdo_group_pool_; }
		auto Slave::pdoGroupPool()const->const aris::core::ObjectPool<PdoGroup, Element>&{return *imp_->pdo_group_pool_; }
		auto Slave::sdoPool()->aris::core::ObjectPool<Sdo, Element>& { return *imp_->sdo_pool_; }
		auto Slave::sdoPool()const->const aris::core::ObjectPool<Sdo, Element>&{return *imp_->sdo_pool_; }
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t &value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).read(value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t &value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).read(value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t &value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).read(value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t &value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).read(value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t &value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).read(value);
		}
		auto Slave::readPdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t &value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).read(value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).write(value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).write(value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).write(value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).write(value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).write(value);
		}
		auto Slave::writePdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t value)->void
		{
			auto id_pair = imp_->pdo_map_.at(index).at(subindex);
			pdoGroupPool().at(id_pair.first).at(id_pair.second).write(value);
		}
		auto Slave::readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t &value)const->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).getConfigValue(value);
		}
		auto Slave::readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t &value)const->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).getConfigValue(value);
		}
		auto Slave::readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t &value)const->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).getConfigValue(value);
		}
		auto Slave::readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t &value)const->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).getConfigValue(value);
		}
		auto Slave::readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t &value)const->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).getConfigValue(value);
		}
		auto Slave::readSdoConfigIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t &value)const->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).getConfigValue(value);
		}
		auto Slave::configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).setConfigValue(value);
		}
		auto Slave::configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).setConfigValue(value);
		}
		auto Slave::configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).setConfigValue(value);
		}
		auto Slave::configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).setConfigValue(value);
		}
		auto Slave::configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).setConfigValue(value);
		}
		auto Slave::configSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).setConfigValue(value);
		}
		auto Slave::readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t &value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).read(value);
		}
		auto Slave::readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t &value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).read(value);
		}
		auto Slave::readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t &value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).read(value);
		}
		auto Slave::readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t &value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).read(value);
		}
		auto Slave::readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t &value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).read(value);
		}
		auto Slave::readSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t &value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).read(value);
		}
		auto Slave::writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int8_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).write(value);
		}
		auto Slave::writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int16_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).write(value);
		}
		auto Slave::writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::int32_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).write(value);
		}
		auto Slave::writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint8_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).write(value);
		}
		auto Slave::writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint16_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).write(value);
		}
		auto Slave::writeSdoIndex(std::uint16_t index, std::uint8_t subindex, std::uint32_t value)->void
		{
			int sdo_ID = imp_->sdo_map_.at(index).at(subindex);
			sdoPool().at(sdo_ID).write(value);
		}
		Slave::~Slave() = default;
		Slave::Slave(Object &father, const aris::core::XmlElement &xml_ele) :Element(father, xml_ele), imp_(new Imp(this))
		{
			if (master().findByName("slave_type_pool") == master().children().end())
			{
				throw std::runtime_error("you must insert \"slave_type_pool\" before insert \"slave_pool\" node");
			}
			auto &slave_type_pool = static_cast<aris::core::ObjectPool<SlaveType, Element> &>(*master().findByName("slave_type_pool"));

			if (slave_type_pool.findByName(attributeString(xml_ele, "slave_type")) == slave_type_pool.end())
			{
				throw std::runtime_error("can not find slave_type \"" + attributeString(xml_ele, "slave_type") + "\" in slave \"" + name() + "\"");
			}
			imp_->slave_type_ = &add<SlaveType>(*slave_type_pool.findByName(attributeString(xml_ele, "slave_type")));
			imp_->pdo_group_pool_ = static_cast<aris::core::ObjectPool<PdoGroup, Element> *>(&*imp_->slave_type_->findByName("pdo_group_pool"));
			imp_->sdo_pool_ = static_cast<aris::core::ObjectPool<Sdo, Element> *>(&*imp_->slave_type_->findByName("sdo_pool"));

			for (auto &group : pdoGroupPool())for (auto &pdo : group)pdo.DO::imp_->slave_ = this;
			for (auto &sdo : sdoPool())sdo.DO::imp_->slave_ = this;

			// make PDO map //
			for (int i = 0; i < static_cast<int>(pdoGroupPool().size()); ++i)
			{
				auto &group = pdoGroupPool().at(i);
				for (int j = 0; j < static_cast<int>(group.size()); ++j)
				{
					auto &pdo = group.at(j);

					if (imp_->pdo_map_.find(pdo.index()) != imp_->pdo_map_.end())
					{
						imp_->pdo_map_.at(pdo.index()).insert(std::make_pair(pdo.subindex(), std::make_pair(i, j)));
					}
					else
					{
						std::map<std::uint8_t, std::pair<int, int> > subindex_map;
						subindex_map.insert(std::make_pair(pdo.subindex(), std::make_pair(i, j)));
						imp_->pdo_map_.insert(std::make_pair(pdo.index(), subindex_map));
					}
				}
			}

			// make SDO map //
			for (int i = 0; i < static_cast<int>(sdoPool().size()); ++i)
			{
				auto &sdo = sdoPool().at(i);
				if (imp_->sdo_map_.find(sdo.index()) != imp_->sdo_map_.end())
				{
					imp_->sdo_map_.at(sdo.index()).insert(std::make_pair(sdo.subindex(), i));
				}
				else
				{
					std::map<std::uint8_t, int > subindex_map;
					subindex_map.insert(std::make_pair(sdo.subindex(), i));
					imp_->sdo_map_.insert(std::make_pair(sdo.index(), subindex_map));
				}
			}
		}

		class Master::Imp
		{
		public:
			static auto rt_task_func(void *master)->void
			{
				auto &mst = *reinterpret_cast<Master*>(master);

				aris_rt_set_periodic(mst.imp_->sample_period_ns_);

				while (!mst.imp_->is_stopping_)
				{
					aris_rt_wait_period();

					// receive pdo data
					aris_ecrt_master_receive(mst.ecHandle());
					for (auto &sla : mst.slavePool())
					{
						aris_ecrt_slave_receive(sla.ecHandle());
						sla.readUpdate();
					}

					// tg begin
					mst.controlStrategy();
					// tg end

					// sync
					aris_ecrt_master_sync(mst.ecHandle(), aris_rt_timer_read());

					// send pdo data
					for (auto &sla : mst.slavePool())
					{
						sla.writeUpdate();
						aris_ecrt_slave_send(sla.ecHandle());
					}
					aris_ecrt_master_send(mst.ecHandle());

					// log data
					mst.dataLogger().logDataRT();
				}
			};

			aris::core::ObjectPool<SlaveType, Element> *slave_type_pool_;
			aris::core::ObjectPool<Slave, Element> *slave_pool_;
			aris::core::RefPool<Slave::TxType> tx_data_pool_;
			aris::core::RefPool<Slave::RxType> rx_data_pool_;

			//for log
			DataLogger* data_logger_;
			std::atomic_bool is_running_{ false }, is_stopping_{ false };

			const int sample_period_ns_{ 1000000 };

			aris::core::ImpPtr<Handle> rt_task_handle_;
			aris::core::ImpPtr<Handle> ec_handle_;


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

			imp_->data_logger_ = findByName("data_logger") == children().end() ? &add<DataLogger>("data_logger") : static_cast<DataLogger*>(&(*findByName("data_logger")));
			imp_->slave_type_pool_ = findByName("slave_type_pool") == children().end() ? &add<aris::core::ObjectPool<SlaveType, Element> >("slave_type_pool") : static_cast<aris::core::ObjectPool<SlaveType, Element> *>(&(*findByName("slave_type_pool")));
			imp_->slave_pool_ = findByName("slave_pool") == children().end() ? &add<aris::core::ObjectPool<Slave, Element> >("slave_pool") : static_cast<aris::core::ObjectPool<Slave, Element> *>(&(*findByName("slave_pool")));
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

			// init each slave and update tx & rx data pool //
			txDataPool().clear();
			rxDataPool().clear();
			for (auto &slave : slavePool())
			{
				slave.init();
				txDataPool().push_back_ptr(&slave.txData());
				rxDataPool().push_back_ptr(&slave.rxData());
			}

			// init ethercat master, slave, pdo group, and pdo
			imp_->ec_handle_.reset(aris_ecrt_master_init());
			for (auto &sla : slavePool())
			{
				sla.imp_->ec_handle_.reset(aris_ecrt_slave_init());

				for (auto &pdo_group : sla.pdoGroupPool())
				{
					pdo_group.imp_->handle_.reset(aris_ecrt_pdo_group_init());
					for (auto &pdo : pdo_group)
					{
						pdo.imp_->ec_handle_.reset(aris_ecrt_pdo_init());
					}
				}
			}

			// config ethercat master, slave, pdo group, and pdo
			for (auto &sla : slavePool())
			{
				for (auto &pdo_group : sla.pdoGroupPool())
				{
					for (auto &pdo : pdo_group)
					{
						aris_ecrt_pdo_config(sla.ecHandle(), pdo_group.ecHandle(), pdo.ecHandle(), pdo.index(), pdo.subindex(), pdo.dataBit());
					}
					aris_ecrt_pdo_group_config(sla.ecHandle(), pdo_group.ecHandle(), pdo_group.index(), pdo_group.tx());
				}
				aris_ecrt_slave_config(ecHandle(), sla.ecHandle(), sla.alias(), sla.position(), sla.venderID(), sla.productCode(), sla.distributedClock());
			}
			aris_ecrt_master_config(ecHandle());

			//config ethercat sdo
			for (auto &sla : slavePool())for (auto &sdo : sla.sdoPool())
				aris_ecrt_sdo_config(ecHandle(), sla.ecHandle(), sdo.index(), sdo.subindex(), sdo.configBuffer(), sdo.dataBit());


			// start ethercat master and slave
			aris_ecrt_master_start(ecHandle());
			for (auto &sla : slavePool())aris_ecrt_slave_start(sla.ecHandle());

			imp_->rt_task_handle_.reset(aris_rt_task_start(&Imp::rt_task_func, this));
		};
		auto Master::stop()->void
		{
			if (!imp_->is_running_)throw std::runtime_error("master is not running, so can't stop");

			imp_->is_stopping_ = true;

			aris_rt_task_stop(rtHandle());
			aris_ecrt_master_stop(ecHandle());

			imp_->is_stopping_ = false;
			imp_->is_running_ = false;
		}
		auto Master::ecHandle()const->const Handle*{ return imp_->ec_handle_.get(); };
		auto Master::ecHandle()->Handle* { return imp_->ec_handle_.get(); };
		auto Master::rtHandle()const->const Handle*{ return imp_->rt_task_handle_.get(); };
		auto Master::rtHandle()->Handle* { return imp_->rt_task_handle_.get(); };
		auto Master::slaveTypePool()->aris::core::ObjectPool<SlaveType, Element>& { return *imp_->slave_type_pool_; }
		auto Master::slaveTypePool()const->const aris::core::ObjectPool<SlaveType, Element>&{ return *imp_->slave_type_pool_; }
		auto Master::slavePool()->aris::core::ObjectPool<Slave, Element>& { return *imp_->slave_pool_; }
		auto Master::slavePool()const->const aris::core::ObjectPool<Slave, Element>&{ return *imp_->slave_pool_; }
		auto Master::txDataPool()->aris::core::RefPool<Slave::TxType> & { return imp_->tx_data_pool_; }
		auto Master::txDataPool()const->const aris::core::RefPool<Slave::TxType> &{return imp_->tx_data_pool_; }
		auto Master::rxDataPool()->aris::core::RefPool<Slave::RxType> & { return imp_->rx_data_pool_; }
		auto Master::rxDataPool()const->const aris::core::RefPool<Slave::RxType> &{return imp_->rx_data_pool_; }
		auto Master::dataLogger()->DataLogger& { return std::ref(*imp_->data_logger_); }
		auto Master::dataLogger()const->const DataLogger&{ return std::ref(*imp_->data_logger_); }
		Master::~Master() = default;
		Master::Master() :imp_(new Imp)
		{
			registerChildType<DataLogger>();

			registerChildType<Pdo>();
			registerChildType<Sdo>();
			registerChildType<PdoGroup>();
			registerChildType<aris::core::ObjectPool<Sdo, Element> >();
			registerChildType<aris::core::ObjectPool<PdoGroup, Element> >();
			registerChildType<aris::core::ObjectPool<SlaveType, Element> >();

			registerChildType<SlaveType>();
			registerChildType<Slave>();
			registerChildType<aris::core::ObjectPool<Slave, Element> >();
		}
	}
}
