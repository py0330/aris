#ifndef ARIS_CONTROL_MASTER_SLAVE_H_
#define ARIS_CONTROL_MASTER_SLAVE_H_

#include <any>

#include <aris_lib_export.h>
#include <aris/core/object.hpp>
#include <aris/core/basic_type.hpp>
#include <aris/core/msg.hpp>

namespace aris::control
{
	class Master;
	
	class ARIS_API Slave:public aris::core::NamedObject{
	public:
		auto virtual send()->void {}
		auto virtual recv()->void {}
		auto virtual master()->Master*;
		auto virtual master()const->const Master* { return const_cast<std::decay_t<decltype(*this)>*>(this)->master(); }
		auto phyId()const->std::uint16_t; // 为 -1 时，说明是虚拟轴
		auto setPhyId(std::uint16_t phy_id)->void;
		auto isVirtual()const->bool;
		auto setVirtual(bool is_virtual = true)->void;
		auto slaId()const->std::uint16_t { return static_cast<std::uint16_t>(id()); }
		auto id()const->std::uint16_t;

		virtual ~Slave();
		explicit Slave(const std::string &name = "slave", std::uint16_t phy_id = 0);
		ARIS_DECLARE_BIG_FOUR_NOEXCEPT(Slave);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
		friend class Master;
	};
	class ARIS_API Master :public aris::core::NamedObject{
	public:
		struct RtStasticsData{
			double avg_time_consumed;
			std::int64_t max_time_consumed;
			std::int64_t max_time_occur_count;
			std::int64_t min_time_consumed;
			std::int64_t min_time_occur_count;
			std::int64_t total_count;
			std::int64_t overrun_count;
		};
		enum { MAX_MSG_SIZE = 8192 };

		// used only in non-rt thread //
		auto virtual init()->void;
		auto virtual start()->void;
		auto virtual stop()->void;
		auto setControlStrategy(std::function<void()> strategy)->void;
		auto setSamplePeriodNs(int period_ns)->void;
		auto samplePeriodNs()const ->int;

		// used in rt thread //
		auto logFile(const char *file_name)->void;
		auto logFileRawName(const char *raw_file_name)->void;
		auto lout()->aris::core::MsgStream &;
		auto mout()->aris::core::MsgStream &;
		auto slaveAtAbs(aris::Size id)->Slave& { return slavePool().at(id); }
		auto slaveAtAbs(aris::Size id)const->const Slave& { return const_cast<std::decay_t<decltype(*this)> *>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(aris::Size id)->Slave&;
		auto slaveAtPhy(aris::Size id)const->const Slave& { return const_cast<std::decay_t<decltype(*this)> *>(this)->slaveAtPhy(id); }
		auto resetSlavePool(aris::core::PointerArray<Slave> *pool);
		auto slavePool()->aris::core::PointerArray<Slave>&;
		auto slavePool()const->const aris::core::PointerArray<Slave>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->slavePool(); }
		auto rtHandle()->std::any&;
		auto rtHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)> *>(this)->rtHandle(); }
		auto resetRtStasticData(RtStasticsData *stastics, bool is_new_data_include_this_count = false)->void;

		virtual ~Master();
		explicit Master(const std::string &name = "master");
		Master(const Master &other) = delete;
		Master(Master &&other) = delete;
		Master& operator=(const Master &other) = delete;
		Master& operator=(Master &&other) = delete;

	protected:
		auto virtual send()->void { for (auto &s : slavePool())s.send(); }
		auto virtual recv()->void { for (auto &s : slavePool())s.recv(); }
		auto virtual release()->void {}

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		friend class Slave;
		friend class PdoEntry;
	};
}

#endif
