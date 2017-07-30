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
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;

			virtual ~RTTimer();
			RTTimer(const std::string &name);
			RTTimer(Object &father, const aris::core::XmlElement &xml_ele);
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
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto start(const std::string &log_file_name = std::string())->void;
			auto stop()->void;
			auto lout()->aris::core::MsgStream &;
			auto lout()const->const aris::core::MsgStream &{ return const_cast<DataLogger*>(this)->lout(); };
			auto send()->void;

			virtual ~DataLogger();
			DataLogger(const std::string &name);
			DataLogger(Object &father, const aris::core::XmlElement &xml_ele);
			DataLogger(const DataLogger &) = delete;
			DataLogger(DataLogger &&) = delete;
			DataLogger& operator=(const DataLogger &) = delete;
			DataLogger& operator=(DataLogger &&) = delete;

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class Slave : public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Slave"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual send()->void {}
			auto virtual recv()->void {}

			virtual ~Slave();
			explicit Slave(const std::string &name);
			explicit Slave(Object &father, const aris::core::XmlElement &xml_ele);
			Slave(const Slave &other);
			Slave(Slave &&other);
			Slave& operator=(const Slave &other);
			Slave& operator=(Slave &&other);
		};
		class Master : public aris::core::Root
		{
		public:
			enum { MAX_MSG_SIZE = 8192 };
			using Root::loadXml;
			auto virtual loadXml(const aris::core::XmlDocument &xml_doc)->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto start()->void;
			auto stop()->void;
			auto setControlStrategy(std::function<void()> strategy)->void;
			auto rtHandle()->Handle*;
			auto rtHandle()const->const Handle*{ return const_cast<std::decay_t<decltype(*this)> *>(this)->rtHandle(); }
			auto msgIn()->aris::core::MsgFix<MAX_MSG_SIZE>&;
			auto msgIn()const->const aris::core::MsgFix<MAX_MSG_SIZE>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->msgIn(); }
			auto msgOut()->aris::core::MsgFix<MAX_MSG_SIZE>&;
			auto msgOut()const->const aris::core::MsgFix<MAX_MSG_SIZE>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->msgOut(); }
			auto mout()->aris::core::MsgStream &;
			auto mout()const->const aris::core::MsgStream &{ return const_cast<std::decay_t<decltype(*this)> *>(this)->mout(); };
			auto sendOut()->void;
			auto recvOut(aris::core::MsgBase &recv_msg)->int;
			auto sendIn(const aris::core::MsgBase &send_msg)->void;
			auto recvIn()->int;
			auto slavePool()->aris::core::ObjectPool<Slave, Object>&;
			auto slavePool()const->const aris::core::ObjectPool<Slave, Object>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->slavePool(); }
			auto dataLogger()->DataLogger&;
			auto dataLogger()const->const DataLogger&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->dataLogger(); }
			virtual ~Master();
			Master();
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

		class EthercatMotionBase : public virtual Slave
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("EthercatMotionBase"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto maxPos()->double;
			auto minPos()->double;
			auto maxVel()->double;
			auto posOffset()->double;
			auto posFactor()->double;

			auto virtual modeOfOperation()const->std::uint8_t = 0;
			auto virtual targetPos()const->double = 0;
			auto virtual targetVel()const->double = 0;
			auto virtual targetCur()const->double = 0;
			auto virtual offsetVel()const->double = 0;
			auto virtual offsetCur()const->double = 0;

			auto virtual setModeOfOperation(std::uint8_t mode)->void = 0;
			auto virtual setTargetPos(double pos)->void = 0;
			auto virtual setTargetVel(double vel)->void = 0;
			auto virtual setTargetCur(double cur)->void = 0;
			auto virtual setOffsetVel(double vel)->void = 0;
			auto virtual setOffsetCur(double cur)->void = 0;

			auto virtual modeOfDisplay()->std::uint8_t = 0;
			auto virtual actualPos()->double = 0;
			auto virtual actualVel()->double = 0;
			auto virtual actualCur()->double = 0;

			auto virtual disable()->int = 0;
			auto virtual enable()->int = 0;
			auto virtual home()->int = 0;
			auto virtual mode(std::uint8_t md)->int = 0;

			virtual ~EthercatMotionBase();
			explicit EthercatMotionBase(const std::string &name, std::int32_t pos_factor, double max_pos, double min_pos, double max_vel, double home_pos = 0, double pos_offset = 0);
			explicit EthercatMotionBase(Object &father, const aris::core::XmlElement &xml_ele);
			EthercatMotionBase(const EthercatMotionBase &other) = delete;
			EthercatMotionBase(EthercatMotionBase &&other) = delete;
			EthercatMotionBase& operator=(const EthercatMotionBase &other) = delete;
			EthercatMotionBase& operator=(EthercatMotionBase &&other) = delete;

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class Controller : public virtual Master
		{
		public:
			auto motionPool()->aris::core::RefPool<EthercatMotionBase>&;
			auto motionPool()const->const aris::core::RefPool<EthercatMotionBase>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->motionPool(); }

			virtual ~Controller();
			Controller();
			Controller(const Controller &other) = delete;
			Controller(Controller &&other) = delete;
			Controller& operator=(const Controller &other) = delete;
			Controller& operator=(Controller &&other) = delete;

		private:
			auto virtual init()->void override;
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

		};
	}
}

#endif
