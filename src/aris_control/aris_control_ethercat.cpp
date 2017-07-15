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
		auto Element::saveXml(aris::core::XmlElement &xml_ele) const->void{	Object::saveXml(xml_ele); }
		auto Element::master()->Master & { return static_cast<Master &>(root()); }
		auto Element::master()const->const Master &{ return static_cast<const Master &>(root()); }

		struct DataLogger::Imp
		{
			aris::core::Pipe *log_pipe_;
			aris::core::MsgFix<MAX_LOG_DATA_SIZE> log_msg_;
			
			std::unique_ptr<aris::core::MsgStream> log_msg_stream_;
			
			std::thread log_thread_;
			
			std::mutex mu_running_;
			std::atomic_bool is_running_;

			Imp() :log_msg_(), is_running_(false) { log_msg_stream_.reset(new aris::core::MsgStream(log_msg_)); };
		};
		auto DataLogger::saveXml(aris::core::XmlElement &xml_ele) const->void{	Element::saveXml(xml_ele);	}
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
				while(imp_->log_pipe_->recvMsg(recv_msg))file << recv_msg.data();
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

			Imp(DataType data_type = DO::INT32, std::uint16_t index = 0, std::uint8_t subindex = 0):data_type_(data_type), index_(index), subindex_(subindex), slave_(nullptr){}
		};
		auto DO::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);

			std::stringstream s;
			s << "0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(index());
			xml_ele.SetAttribute("index", s.str().c_str());

			s = std::stringstream();
			s << "0x" << std::setfill('0') << std::setw(sizeof(std::int8_t) * 2) << std::hex << static_cast<std::uint32_t>(subindex());
			xml_ele.SetAttribute("subindex", s.str().c_str());

			switch (dataType())
			{
			case INT32:xml_ele.SetAttribute("datatype", "int32"); break;
			case INT16:xml_ele.SetAttribute("datatype", "int16"); break;
			case INT8:xml_ele.SetAttribute("datatype", "int8"); break;
			case UINT32:xml_ele.SetAttribute("datatype", "uint32"); break;
			case UINT16:xml_ele.SetAttribute("datatype", "uint16"); break;
			case UINT8:xml_ele.SetAttribute("datatype", "uint8"); break;
			}
		}
		auto DO::slave()->Slave& { return *imp_->slave_; }
		auto DO::slave()const->const Slave&{ return *imp_->slave_; }
		auto DO::index()const->std::uint16_t { return imp_->index_; }
		auto DO::subindex()const->std::uint8_t { return imp_->subindex_; }
		auto DO::dataBitSize()const->std::uint8_t { return imp_->data_bit_size_; }
		auto DO::dataType()const->DataType { return imp_->data_type_; }
		DO::~DO() = default;
		DO::DO(const std::string &name, DO::DataType data_type, std::uint16_t index, std::uint8_t subindex):Element(name), imp_(new Imp(data_type, index, subindex))
		{
			switch (data_type)
			{
			case aris::control::DO::INT32:	imp_->data_bit_size_ = 32; break;
			case aris::control::DO::INT16:imp_->data_bit_size_ = 16; break;
			case aris::control::DO::INT8:imp_->data_bit_size_ = 8; break;
			case aris::control::DO::UINT32:imp_->data_bit_size_ = 32; break;
			case aris::control::DO::UINT16:imp_->data_bit_size_ = 16; break;
			case aris::control::DO::UINT8:imp_->data_bit_size_ = 8;	break;
			default:throw std::runtime_error("wrong type in DO"); break;
			}
		}
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

			Imp(unsigned opt = Sdo::READ | Sdo::WRITE | Sdo::CONFIG):option_(opt) {}
		};
		auto Sdo::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			DO::saveXml(xml_ele);

			xml_ele.SetAttribute("read", option() & READ ? "true" : "false");
			xml_ele.SetAttribute("write", option() & WRITE ? "true" : "false");
			if (option() & CONFIG) 
			{
				switch (dataType())
				{
				case aris::control::DO::INT32:xml_ele.SetAttribute("config", imp_->config_value_int32_); break;
				case aris::control::DO::INT16:xml_ele.SetAttribute("config", imp_->config_value_int16_); break;
				case aris::control::DO::INT8:xml_ele.SetAttribute("config", imp_->config_value_int8_); break;
				case aris::control::DO::UINT32:xml_ele.SetAttribute("config", imp_->config_value_uint32_); break;
				case aris::control::DO::UINT16:xml_ele.SetAttribute("config", imp_->config_value_uint16_); break;
				case aris::control::DO::UINT8:xml_ele.SetAttribute("config", imp_->config_value_uint8_); break;
				}
			}
		}
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
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &real_size, &abort_code);
		}
		auto Sdo::read(std::int16_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != INT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int16 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &real_size, &abort_code);
		}
		auto Sdo::read(std::int8_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != INT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int8 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &real_size, &abort_code);
		}
		auto Sdo::read(std::uint32_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != UINT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint32 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &real_size, &abort_code);
		}
		auto Sdo::read(std::uint16_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != UINT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint16 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &real_size, &abort_code);
		}
		auto Sdo::read(std::uint8_t &value)->void
		{
			if (!readable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not readable");
			if (dataType() != UINT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint8 type");

			std::size_t real_size;
			std::uint32_t abort_code;
			aris_ecrt_sdo_read(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &real_size, &abort_code);
		}
		auto Sdo::write(std::int32_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != INT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int32 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &abort_code);
		}
		auto Sdo::write(std::int16_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != INT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int16 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &abort_code);
		}
		auto Sdo::write(std::int8_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != INT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not int8 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &abort_code);
		}
		auto Sdo::write(std::uint32_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != UINT32)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint32 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &abort_code);
		}
		auto Sdo::write(std::uint16_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != UINT16)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint16 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &abort_code);
		}
		auto Sdo::write(std::uint8_t value)->void
		{
			if (!writeable())throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not writeable");
			if (dataType() != UINT8)throw std::runtime_error("sdo " + std::to_string(index()) + "-" + std::to_string(subindex()) + " is not uint8 type");

			std::uint32_t abort_code;
			aris_ecrt_sdo_write(master().ecHandle(), slave().position(), index(), subindex(), reinterpret_cast<std::uint8_t *>(&value), dataBitSize(), &abort_code);
		}
		Sdo::~Sdo() = default;
		Sdo::Sdo(const std::string &name, DO::DataType data_type, std::uint16_t index, std::uint8_t sub_index, unsigned opt, std::int32_t config_value) :DO(name, data_type, index, sub_index), imp_(new Imp(opt)) 
		{
			if (opt & Sdo::CONFIG)
			{
				if(!(opt & Sdo::WRITE)) throw std::runtime_error("you can't config data in unwriteable sdo, error in \"" + name + "\" sdo");
				switch (dataType())
				{
				case aris::control::DO::INT32:
					imp_->config_value_int32_ = static_cast<std::int32_t>(config_value);
					break;
				case aris::control::DO::INT16:
					imp_->config_value_int16_ = static_cast<std::int16_t>(config_value);
					break;
				case aris::control::DO::INT8:
					imp_->config_value_int8_ = static_cast<std::int8_t>(config_value);
					break;
				case aris::control::DO::UINT32:
					imp_->config_value_uint32_ = static_cast<std::uint32_t>(config_value);
					break;
				case aris::control::DO::UINT16:
					imp_->config_value_uint16_ = static_cast<std::uint16_t>(config_value);
					break;
				case aris::control::DO::UINT8:
					imp_->config_value_uint8_ = static_cast<std::uint8_t>(config_value);
					break;
				default:
					throw std::runtime_error("failed to get sdo config value");
					break;
				}

			}
		}
		Sdo::Sdo(Object &father, const aris::core::XmlElement &xml_ele) :DO(father, xml_ele)
		{
			if (attributeBool(xml_ele, "read", true))imp_->option_ |= READ; else imp_->option_ &= ~READ;
			if (attributeBool(xml_ele, "write", true))imp_->option_ |= WRITE; else imp_->option_ &= ~WRITE;
			if (xml_ele.Attribute("config"))
			{
				if (!writeable())throw std::runtime_error("you can't config data in unwriteable sdo, error in \"" + std::string(xml_ele.Name()) + "\" sdo");
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

		struct Pdo::Imp { aris::core::ImpPtr<Handle> ec_handle_; };
		auto Pdo::saveXml(aris::core::XmlElement &xml_ele) const->void{	DO::saveXml(xml_ele);}
		auto Pdo::ecHandle()->Handle* { return imp_->ec_handle_.get(); }
		auto Pdo::ecHandle()const->const Handle*{ return imp_->ec_handle_.get(); }
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
		Pdo::Pdo(const std::string &name, DO::DataType data_type, std::uint16_t index, std::uint8_t sub_index):DO(name, data_type, index, sub_index){}
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

			Imp(std::uint16_t index = 0, bool is_tx = true):is_tx_(is_tx), index_(index) {}
		};
		auto PdoGroup::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);

			std::stringstream s;
			s << "0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(index());
			xml_ele.SetAttribute("index", s.str().c_str());

			xml_ele.SetAttribute("is_tx", tx());
		}
		auto PdoGroup::ecHandle()->Handle* { return imp_->handle_.get(); };
		auto PdoGroup::ecHandle()const->const Handle*{ return imp_->handle_.get(); };
		auto PdoGroup::tx()const->bool { return imp_->is_tx_; }
		auto PdoGroup::rx()const->bool { return !imp_->is_tx_; }
		auto PdoGroup::index()const->std::uint16_t { return imp_->index_; }
		PdoGroup::~PdoGroup() = default;
		PdoGroup::PdoGroup(const std::string &name, std::uint16_t index, bool is_tx):aris::core::ObjectPool<Pdo, Element>(name), imp_(new Imp(index, is_tx)){}
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

			Imp(std::uint32_t product_code = 0, std::uint32_t vender_id = 0, std::uint16_t alias = 0, std::uint32_t distributed_clock = 0)
				:product_code_(product_code), vender_id_(vender_id), alias_(alias), distributed_clock_(distributed_clock){}
		};
		auto SlaveType::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);

			std::stringstream s;
			s << "0x" << std::setfill('0') << std::setw(sizeof(decltype(productCode())) * 2) << std::hex << productCode();
			xml_ele.SetAttribute("product_code", s.str().c_str());

			s = std::stringstream();
			s << "0x" << std::setfill('0') << std::setw(sizeof(decltype(venderID())) * 2) << std::hex << venderID();
			xml_ele.SetAttribute("vender_id", s.str().c_str());

			s = std::stringstream();
			s << "0x" << std::setfill('0') << std::setw(sizeof(decltype(alias())) * 2) << std::hex << alias();
			xml_ele.SetAttribute("alias", s.str().c_str());

			s = std::stringstream();
			s << "0x" << std::setfill('0') << std::setw(sizeof(decltype(distributedClock())) * 2) << std::hex << distributedClock();
			xml_ele.SetAttribute("distributed_clock", s.str().c_str());
		}
		auto SlaveType::productCode()const->std::uint32_t { return imp_->product_code_; }
		auto SlaveType::venderID()const->std::uint32_t { return imp_->vender_id_; }
		auto SlaveType::alias()const->std::uint16_t { return imp_->alias_; }
		auto SlaveType::distributedClock()const->std::uint32_t { return imp_->distributed_clock_; }
		SlaveType::~SlaveType() = default;
		SlaveType::SlaveType(const std::string &name, std::uint32_t product_code, std::uint32_t vender_id, std::uint16_t alias, std::uint32_t distributed_clock)
			:Element(name), imp_(new Imp(product_code, vender_id, alias, distributed_clock)){}
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
			Imp(Slave*slave, const SlaveType *st = nullptr) :slave_(slave), slave_type_(st) {}

			aris::core::ImpPtr<Handle> ec_handle_;

			const SlaveType *slave_type_;

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
		auto Slave::saveXml(aris::core::XmlElement &xml_ele) const->void{Element::saveXml(xml_ele);}
		auto Slave::init()->void
		{
			// make PDO map and upd pdo's slave ptr //
			imp_->pdo_map_.clear();
			for (int i = 0; i < static_cast<int>(pdoGroupPool().size()); ++i)
			{
				auto &group = pdoGroupPool().at(i);
				for (int j = 0; j < static_cast<int>(group.size()); ++j)
				{
					auto &pdo = group.at(j);
					pdo.DO::imp_->slave_ = this;
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

			// make SDO map and upd pdo's slave ptr //
			imp_->sdo_map_.clear();
			for (int i = 0; i < static_cast<int>(sdoPool().size()); ++i)
			{
				auto &sdo = sdoPool().at(i);
				sdo.DO::imp_->slave_ = this;
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
		Slave::Slave(const std::string &name, const SlaveType &slave_type) :Element(name), imp_(new Imp(this, &slave_type))
		{
			imp_->pdo_group_pool_ = &add<aris::core::ObjectPool<PdoGroup, Element> >("pdo_group_pool");
			imp_->sdo_pool_ = &add<aris::core::ObjectPool<Sdo, Element> >("sdo_pool");
		}
		Slave::Slave(Object &father, const aris::core::XmlElement &xml_ele) :Element(father, xml_ele), imp_(new Imp(this))
		{
			if (master().findByName("slave_type_pool") == master().children().end())throw std::runtime_error("you must insert \"slave_type_pool\" before insert \"slave_pool\" node");
			auto &slave_type_pool = static_cast<aris::core::ObjectPool<SlaveType, Element> &>(*master().findByName("slave_type_pool"));

			if (slave_type_pool.findByName(attributeString(xml_ele, "slave_type")) == slave_type_pool.end())
			{
				throw std::runtime_error("can not find slave_type \"" + attributeString(xml_ele, "slave_type") + "\" in slave \"" + name() + "\"");
			}
			imp_->slave_type_ = &*slave_type_pool.findByName(attributeString(xml_ele, "slave_type"));
			imp_->pdo_group_pool_ = findOrInsert<aris::core::ObjectPool<PdoGroup, Element> >("pdo_group_pool");
			imp_->sdo_pool_ = findOrInsert<aris::core::ObjectPool<Sdo, Element> >("sdo_pool");
		}

		class Master::Imp
		{
		public:
			static auto rt_task_func(void *master)->void
			{
				auto &mst = *reinterpret_cast<Master*>(master);

				aris_rt_task_set_periodic(mst.imp_->sample_period_ns_);

				while (mst.imp_->is_running_)
				{
					aris_rt_task_wait_period();

					// receive pdo data
					aris_ecrt_master_receive(mst.ecHandle());
					for (auto &sla : mst.slavePool())
					{
						aris_ecrt_slave_receive(sla.ecHandle());
						sla.readUpdate();
					}

					// tg begin
					if (mst.imp_->strategy_)mst.imp_->strategy_();
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
				}
			};

			// slave type and slave //
			aris::core::ObjectPool<SlaveType, Element> *slave_type_pool_;
			aris::core::ObjectPool<Slave, Element> *slave_pool_;
			aris::core::RefPool<Slave::TxType> tx_data_pool_;
			aris::core::RefPool<Slave::RxType> rx_data_pool_;
			
			// for log //
			DataLogger* data_logger_;

			// for msg in and out //
			aris::core::Pipe *pipe_in_;
			aris::core::Pipe *pipe_out_;

			// strategy //
			std::function<void()> strategy_{ nullptr };

			// is running //
			std::mutex mu_running_;
			std::atomic_bool is_running_{ false };

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

			imp_->slave_type_pool_ = findByName("slave_type_pool") == children().end() ? &add<aris::core::ObjectPool<SlaveType, Element> >("slave_type_pool") : static_cast<aris::core::ObjectPool<SlaveType, Element> *>(&(*findByName("slave_type_pool")));
			imp_->slave_pool_ = findByName("slave_pool") == children().end() ? &add<aris::core::ObjectPool<Slave, Element> >("slave_pool") : static_cast<aris::core::ObjectPool<Slave, Element> *>(&(*findByName("slave_pool")));
			imp_->tx_data_pool_.clear();
			imp_->rx_data_pool_.clear();
			imp_->data_logger_ = findByName("data_logger") == children().end() ? &add<DataLogger>("data_logger") : static_cast<DataLogger*>(&(*findByName("data_logger")));
			imp_->pipe_in_ = findOrInsert<aris::core::Pipe>("msg_pipe_in");
			imp_->pipe_out_ = findOrInsert<aris::core::Pipe>("msg_pipe_out");
			for (auto &slave : slavePool())
			{
				imp_->tx_data_pool_.push_back_ptr(&slave.txData());
				imp_->rx_data_pool_.push_back_ptr(&slave.rxData());
			}
		}
		auto Master::start()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("master already running, so cannot start");
			imp_->is_running_ = true;

			// init each slave //
			for (auto &slave : slavePool())slave.init();

			// init ethercat master, slave, pdo group, and pdo //
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

			// config ethercat master, slave, pdo group, and pdo //
			for (auto &sla : slavePool())
			{
				for (auto &pdo_group : sla.pdoGroupPool())
				{
					for (auto &pdo : pdo_group)
					{
						aris_ecrt_pdo_config(sla.ecHandle(), pdo_group.ecHandle(), pdo.ecHandle(), pdo.index(), pdo.subindex(), pdo.dataBitSize());
					}
					aris_ecrt_pdo_group_config(sla.ecHandle(), pdo_group.ecHandle(), pdo_group.index(), pdo_group.tx());
				}
				aris_ecrt_slave_config(ecHandle(), sla.ecHandle(), sla.alias(), sla.position(), sla.venderID(), sla.productCode(), sla.distributedClock());
			}
			aris_ecrt_master_config(ecHandle());

			// config ethercat sdo //
			for (auto &sla : slavePool())for (auto &sdo : sla.sdoPool())
				aris_ecrt_sdo_config(ecHandle(), sla.ecHandle(), sdo.index(), sdo.subindex(), sdo.configBuffer(), sdo.dataBitSize());


			// start ethercat master and slave //
			aris_ecrt_master_start(ecHandle());
			for (auto &sla : slavePool())aris_ecrt_slave_start(sla.ecHandle());

			// create and start rt thread //
			imp_->rt_task_handle_.reset(aris_rt_task_create());
			if (imp_->rt_task_handle_.get() == nullptr) throw std::runtime_error("rt_task_create failed");
			if (aris_rt_task_start(imp_->rt_task_handle_.get(), &Imp::rt_task_func, this))throw std::runtime_error("rt_task_start failed");
		};
		auto Master::stop()->void
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (!imp_->is_running_)throw std::runtime_error("master is not running, so can't stop");
			imp_->is_running_ = false;

			if (aris_rt_task_join(rtHandle()))throw std::runtime_error("aris_rt_task_join failed");
			aris_ecrt_master_stop(ecHandle());
		}
		auto Master::setControlStrategy(std::function<void()> strategy)->void 
		{
			std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("master already running, cannot set control strategy");
			imp_->strategy_ = strategy;
		}
		auto Master::ecHandle()->Handle* { return imp_->ec_handle_.get(); };
		auto Master::rtHandle()->Handle* { return imp_->rt_task_handle_.get(); };
		auto Master::pipeIn()->aris::core::Pipe& { return *imp_->pipe_in_; }
		auto Master::pipeOut()->aris::core::Pipe& { return *imp_->pipe_out_; }
		auto Master::slaveTypePool()->aris::core::ObjectPool<SlaveType, Element>& { return *imp_->slave_type_pool_; }
		auto Master::slavePool()->aris::core::ObjectPool<Slave, Element>& { return *imp_->slave_pool_; }
		auto Master::dataLogger()->DataLogger& { return *imp_->data_logger_; }
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

			imp_->slave_type_pool_ = &add<aris::core::ObjectPool<SlaveType, Element> >("slave_type_pool");
			imp_->slave_pool_ = &add<aris::core::ObjectPool<Slave, Element> >("slave_pool");
			imp_->data_logger_ = &add<DataLogger>("date_logger");
			imp_->pipe_in_ = &add<aris::core::Pipe>("msg_pipe_in");
			imp_->pipe_out_ = &add<aris::core::Pipe>("msg_pipe_out");
		}
	}
}
