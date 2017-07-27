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
		class RTLogger :public aris::core::Object
		{
		public:
			enum { MAX_LOG_DATA_SIZE = 8192 };
			static auto Type()->const std::string &{ static const std::string type("RTLogger"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto start(const std::string &log_file_name = std::string())->void;
			auto stop()->void;
			auto lout()->aris::core::MsgStream &;
			auto lout()const->const aris::core::MsgStream &{ return const_cast<RTLogger*>(this)->lout(); };
			auto send()->void;

			virtual ~RTLogger();
			RTLogger(const std::string &name);
			RTLogger(Object &father, const aris::core::XmlElement &xml_ele);
			RTLogger(const RTLogger &) = delete;
			RTLogger(RTLogger &&) = delete;
			RTLogger& operator=(const RTLogger &) = delete;
			RTLogger& operator=(RTLogger &&) = delete;

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class NSlave : public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("NSlave"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual send()->void {}
			auto virtual recv()->void {}

			virtual ~NSlave();
			explicit NSlave(const std::string &name);
			explicit NSlave(Object &father, const aris::core::XmlElement &xml_ele);
			NSlave(const NSlave &other);
			NSlave(NSlave &&other);
			NSlave& operator=(const NSlave &other);
			NSlave& operator=(NSlave &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class NMaster : public aris::core::Root
		{
		public:
			enum { MAX_MSG_SIZE = 8192 };
			using Root::loadXml;
			auto virtual loadXml(const aris::core::XmlDocument &xml_doc)->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual start()->void;
			auto virtual stop()->void;
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
			auto slavePool()->aris::core::ObjectPool<NSlave, Object>&;
			auto slavePool()const->const aris::core::ObjectPool<NSlave, Object>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->slavePool(); }
			auto dataLogger()->RTLogger&;
			auto dataLogger()const->const RTLogger&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->dataLogger(); }
			virtual ~NMaster();
			NMaster();
			NMaster(const NMaster &other) = delete;
			NMaster(NMaster &&other) = delete;
			NMaster& operator=(const NMaster &other) = delete;
			NMaster& operator=(NMaster &&other) = delete;

		protected:
			auto virtual send()->void { for (auto &s : slavePool())s.send(); }
			auto virtual recv()->void { for (auto &s : slavePool())s.recv(); }
			auto virtual sync()->void {}

		private:
			class Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class NSlave;
			friend class Sdo;
			friend class Pdo;
		};
	}
}

#endif
