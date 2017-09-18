#ifndef ARIS_CONTROL_MASTER_SLAVE_H
#define ARIS_CONTROL_MASTER_SLAVE_H

#include <functional>
#include <thread>
#include <atomic>

#include <aris_core.h>
#include <aris_control_rt_timer.h>

namespace aris
{
	namespace control
	{
		class RTTimer : public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("RTTimer"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;

			virtual ~RTTimer();
			explicit RTTimer(const std::string &name = "rt_timer");
			RTTimer(const RTTimer &) = delete;
			RTTimer(RTTimer &&) = delete;
			RTTimer& operator=(const RTTimer &) = delete;
			RTTimer& operator=(RTTimer &&) = delete;

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class DataLogger :public aris::core::Object
		{
		public:
			enum { MAX_LOG_DATA_SIZE = 8192 };
			static auto Type()->const std::string &{ static const std::string type("DataLogger"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto start(const std::string &log_file_name = std::string())->void;
			auto stop()->void;
			// use in rt thread //
			auto send()->void;
			auto lout()->aris::core::MsgStream &;
			auto lout()const->const aris::core::MsgStream &{ return const_cast<DataLogger*>(this)->lout(); };

			virtual ~DataLogger();
			explicit DataLogger(const std::string &name = "data_logger");
			DataLogger(const DataLogger &) = delete;
			DataLogger(DataLogger &&) = delete;
			DataLogger& operator=(const DataLogger &) = delete;
			DataLogger& operator=(DataLogger &&) = delete;

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class SlaveType :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("SlaveType"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }

			virtual ~SlaveType() = default;
			explicit SlaveType(const std::string &name = "slave_type") :Object(name) {};
			SlaveType(const SlaveType &) = default;
			SlaveType(SlaveType &&) = default;
			SlaveType& operator=(const SlaveType &) = default;
			SlaveType& operator=(SlaveType &&) = default;
		};
		class Slave : public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Slave"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual send()->void {}
			auto virtual recv()->void {}
			auto slaveType()const->const SlaveType *;
			auto phyId()const->std::uint16_t;
			auto slaId()const->std::uint16_t { return static_cast<std::uint16_t>(id()); }

			virtual ~Slave();
			explicit Slave(const std::string &name = "slave", const SlaveType *slave_type = nullptr, std::uint16_t phy_id = 0);
			Slave(const Slave &other);
			Slave(Slave &&other);
			Slave& operator=(const Slave &other);
			Slave& operator=(Slave &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class Master : public aris::core::Root
		{
		public:
			enum { MAX_MSG_SIZE = 8192 };
			using Root::loadXml;
			auto virtual loadXml(const aris::core::XmlDocument &xml_doc)->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			// used only in non-rt thread //
			auto start()->void;
			auto stop()->void;
			auto setControlStrategy(std::function<void()> strategy)->void;
			
			// used in rt thread //
			auto mout()->aris::core::MsgStream &;
			auto msgIn()->aris::core::MsgFix<MAX_MSG_SIZE>&;
			auto msgOut()->aris::core::MsgFix<MAX_MSG_SIZE>&;
			auto sendOut()->void;
			auto recvOut(aris::core::MsgBase &recv_msg)->int;
			auto sendIn(const aris::core::MsgBase &send_msg)->void;
			auto recvIn()->int;
			auto slaveAtAbs(aris::Size id)->Slave& { return slavePool().at(id); }
			auto slaveAtAbs(aris::Size id)const->const Slave& { return const_cast<std::decay_t<decltype(*this)> *>(this)->slaveAtAbs(id); }
			auto slaveAtPhy(aris::Size id)->Slave&;
			auto slaveAtPhy(aris::Size id)const->const Slave&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->slaveAtPhy(id); }
			auto slaveTypePool()->aris::core::ObjectPool<SlaveType>&;
			auto slaveTypePool()const->const aris::core::ObjectPool<SlaveType>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->slaveTypePool(); }
			auto slavePool()->aris::core::ObjectPool<Slave>&;
			auto slavePool()const->const aris::core::ObjectPool<Slave>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->slavePool(); }
			auto dataLogger()->DataLogger&;
			auto dataLogger()const->const DataLogger&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->dataLogger(); }
			auto rtHandle()->Handle*;
			auto rtHandle()const->const Handle*{ return const_cast<std::decay_t<decltype(*this)> *>(this)->rtHandle(); }

			virtual ~Master();
			explicit Master(const std::string &name = "master");
			Master(const Master &other) = delete;
			Master(Master &&other) = delete;
			Master& operator=(const Master &other) = delete;
			Master& operator=(Master &&other) = delete;

		protected:
			auto virtual init()->void {}
			auto virtual send()->void { for (auto &s : slavePool())s.send(); }
			auto virtual recv()->void { for (auto &s : slavePool())s.recv(); }
			auto virtual sync()->void {}
			auto virtual release()->void {}

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Slave;
			friend class Sdo;
			friend class Pdo;
		};
	}
}

#endif
