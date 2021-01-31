#ifndef ARIS_CONTROL_ETHERCAT_H_
#define ARIS_CONTROL_ETHERCAT_H_

#include <filesystem>

#include <aris_lib_export.h>
#include <aris/control/controller_motion.hpp>

namespace aris::control
{
	class ARIS_API PdoEntry:public aris::core::NamedObject
	{
	public:
		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }
		auto setIndex(std::uint16_t index)->void;
		auto index()const->std::uint16_t;
		auto setSubindex(std::uint8_t subindex)->void;
		auto subindex()const->std::uint8_t;
		auto setBitSize(Size subindex)->void;
		auto bitSize()const->aris::Size;

		virtual ~PdoEntry();
		explicit PdoEntry(const std::string &name = "entry", std::uint16_t index = 0x0000, std::uint8_t subindex = 0x00, aris::Size bit_size = 8);
		ARIS_DECLARE_BIG_FOUR(PdoEntry);

	public:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API Pdo :public std::vector<PdoEntry>, public aris::core::NamedObject
	{
	public:
		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }
		auto setIndex(std::uint16_t index)->void;
		auto index()const->std::uint16_t;

		virtual ~Pdo();
		explicit Pdo(const std::string &name = "pdo", std::uint16_t index = 0x0000);
		ARIS_DECLARE_BIG_FOUR(Pdo);

	public:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API SyncManager :public std::vector<Pdo>, public aris::core::NamedObject
	{
	public:
		auto tx()const->bool;
		auto rx()const->bool;
		auto setTx(bool is_tx)->void;

		virtual ~SyncManager();
		explicit SyncManager(const std::string &name = "sm", bool is_tx = true);
		ARIS_DECLARE_BIG_FOUR(SyncManager);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class ARIS_API EthercatMaster;
	class ARIS_API EthercatSlave : virtual public Slave
	{
	public:
		auto ecMaster()->EthercatMaster*;
		auto ecMaster()const->const EthercatMaster* { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecMaster(); }

		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }

		// attribute //
		auto smPool()->std::vector<SyncManager>&;
		auto smPool()const->const std::vector<SyncManager>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->smPool(); }
		auto setSmPool(std::vector<SyncManager> *sm_pool)->void;
		auto vendorID()const->std::uint32_t;
		auto setVendorID(std::uint32_t vendor_id)->void;
		auto productCode()const->std::uint32_t;
		auto setProductCode(std::uint32_t product_code)->void;
		auto revisionNum()const->std::uint32_t;
		auto setRevisionNum(std::uint32_t revision_num)->void;
		auto dcAssignActivate()const->std::uint32_t;
		auto setDcAssignActivate(std::uint32_t dc_assign_activate)->void;
		auto sync0ShiftNs()const->std::int32_t;
		auto setSync0ShiftNs(std::int32_t sync0_shift_ns)->void;
		auto findPdoEntry(std::uint16_t index, std::uint8_t subindex)const->const PdoEntry* { return const_cast<std::decay_t<decltype(*this)> *>(this)->findPdoEntry(index, subindex); }
		auto findPdoEntry(std::uint16_t index, std::uint8_t subindex)->PdoEntry*;

		// scan //
		auto scanInfoForCurrentSlave()->void;
		auto scanPdoForCurrentSlave()->void;
		
		// pdo manipulation //
		template<typename ValueType>
		auto readPdo(std::uint16_t index, std::uint8_t subindex, ValueType &value)const->int { return readPdo(index, subindex, &value, sizeof(ValueType) * 8); }
		auto readPdo(std::uint16_t index, std::uint8_t subindex, void *value, aris::Size bit_size)const->int;
		template<typename ValueType>
		auto writePdo(std::uint16_t index, std::uint8_t subindex, const ValueType &value)->int { return writePdo(index, subindex, &value, sizeof(ValueType) * 8); }
		auto writePdo(std::uint16_t index, std::uint8_t subindex, const void *value, aris::Size bit_size)->int;
		template<typename ValueType>
		auto readSdo(std::uint16_t index, std::uint8_t subindex, ValueType &value)->void { readSdo(index, subindex, &value, sizeof(ValueType)); }
		auto readSdo(std::uint16_t index, std::uint8_t subindex, void *value, aris::Size byte_size)->void;
		template<typename ValueType>
		auto writeSdo(std::uint16_t index, std::uint8_t subindex, const ValueType &value)->void { writeSdo(index, subindex, &value, sizeof(ValueType)); }
		auto writeSdo(std::uint16_t index, std::uint8_t subindex, const void *value, aris::Size byte_size)->void;

		virtual ~EthercatSlave();
		explicit EthercatSlave(const std::string &name = "ethercat_slave", std::uint16_t phy_id = 0, std::uint32_t vendor_id = 0, std::uint32_t product_code = 0, std::uint32_t revision_num = 0, std::uint32_t dc_assign_activate = 0, std::int32_t sync0_shift_ns = 650'000);
		EthercatSlave(const EthercatSlave &other) = delete;
		EthercatSlave(EthercatSlave &&other) = delete;
		EthercatSlave& operator=(const EthercatSlave &other) = delete;
		EthercatSlave& operator=(EthercatSlave &&other) = delete;

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		friend class EthercatMaster;
	};
	class ARIS_API EthercatMaster : virtual public Master
	{
	public:
		typedef struct {
			unsigned int online : 1; /**< The slave is online. */
			unsigned int operational : 1; /**< The slave was brought into \a OP state
										  using the specified configuration. */
			unsigned int al_state : 4; /**< The application-layer state of the slave.
									   - 1: \a INIT
									   - 2: \a PREOP
									   - 4: \a SAFEOP
									   - 8: \a OP

									   Note that each state is coded in a different
									   bit! */
		} SlaveLinkState;
		typedef struct {
			unsigned int slaves_responding; /**< Sum of responding slaves on the given
											link. */
			unsigned int al_states : 4; /**< Application-layer states of the slaves on
										the given link.  The states are coded in the
										lower 4 bits.  If a bit is set, it means
										that at least one slave in the bus is in the
										corresponding state:
										- Bit 0: \a INIT
										- Bit 1: \a PREOP
										- Bit 2: \a SAFEOP
										- Bit 3: \a OP */
			unsigned int link_up : 1; /**< \a true, if the given Ethernet link is up.
									  */
		} MasterLinkState;

		auto esiDirStr()->std::string;
		auto setEsiDirStr(const std::string &str)->void;

		auto slavePool()->aris::core::ChildRefPool<EthercatSlave, aris::core::PointerArray<Slave>>&;
		auto slavePool()const->const aris::core::ChildRefPool<EthercatSlave, aris::core::PointerArray<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(aris::Size id)->EthercatSlave& { return dynamic_cast<EthercatSlave&>(Master::slaveAtAbs(id)); }
		auto slaveAtAbs(aris::Size id)const->const EthercatSlave& { return const_cast<std::decay_t<decltype(*this)> *>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(aris::Size id)->EthercatSlave& { return dynamic_cast<EthercatSlave&>(Master::slaveAtPhy(id)); }
		auto slaveAtPhy(aris::Size id)const->const EthercatSlave& { return const_cast<std::decay_t<decltype(*this)> *>(this)->slaveAtPhy(id); }
		
		auto getLinkState(MasterLinkState *master_state, SlaveLinkState *slave_state)->void; // only for rt

		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)> *>(this)->ecHandle(); }
		auto scan()->void;
		auto scanInfoForCurrentSlaves()->void;
		auto scanPdoForCurrentSlaves()->void;

		auto setEsiDirs(std::vector<std::filesystem::path> esi_dirs)->void;
		auto updateDeviceList()->void;
		auto getDeviceList()->std::string;
		auto getPdoList(int vendor_id, int product_code, int revision_num)->std::string;

		auto virtual init()->void override;
		auto virtual start()->void override;
		
		virtual ~EthercatMaster();
		EthercatMaster(const std::string &name = "ethercat_master");
		EthercatMaster(const EthercatMaster &other) = delete;
		EthercatMaster(EthercatMaster &&other) = delete;
		EthercatMaster& operator=(const EthercatMaster &other) = delete;
		EthercatMaster& operator=(EthercatMaster &&other) = delete;

	protected:
		auto virtual send()->void override;
		auto virtual recv()->void override;
		auto virtual release()->void override;

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		friend class PdoEntry;
	};

	class ARIS_API EthercatMotor :public Motor{
	public:
		auto virtual controlWord()const->std::uint16_t override;
		auto virtual modeOfOperation()const->std::uint8_t override;
		auto virtual targetPos()const->double override;
		auto virtual targetVel()const->double override;
		auto virtual targetToq()const->double override;
		auto virtual offsetVel()const->double override;
		auto virtual offsetCur()const->double override;
		// require pdo 0x6040 //
		auto virtual setControlWord(std::uint16_t control_word)->void override;
		// require pdo 0x6060 //
		auto virtual setModeOfOperation(std::uint8_t mode)->void override;
		// require pdo 0x607A //
		auto virtual setTargetPos(double pos)->void override;
		// require pdo 0x60FF //
		auto virtual setTargetVel(double vel)->void override;
		// require pdo 0x6071 //
		auto virtual setTargetToq(double toq)->void override;
		// require pdo 0x60B1 //
		auto virtual setOffsetVel(double vel)->void override;
		// require pdo 0x60B2 //
		auto virtual setOffsetToq(double toq)->void override;
		// require pdo 0x6041 //
		auto virtual statusWord()const->std::uint16_t override;
		// require pdo 0x6061 //
		auto virtual modeOfDisplay()const->std::uint8_t override;
		// require pdo 0x6064 //
		auto virtual actualPos()const->double override;
		// require pdo 0x606C //
		auto virtual actualVel()const->double override;
		// require pdo 0x6077 //
		auto virtual actualToq()const->double override;
		// require pdo 0x6078 //
		auto virtual actualCur()const->double override;

		// require pdo 0x6040 0x6041 // 
		auto virtual disable()->int override;
		// require pdo 0x6040 0x6041 //
		auto virtual enable()->int override;
		// require pdo 0x6040 0x6041 0x6060 0x6061 //
		auto virtual home()->int override;
		// require pdo 0x6060 0x6061 //
		auto virtual mode(std::uint8_t md)->int override;

		auto slave()->EthercatSlave*;
		auto setSlave(EthercatSlave*)->void;

		virtual ~EthercatMotor();
		EthercatMotor(EthercatSlave* slave = nullptr, const std::string &name = "ethercat_motor"
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0);
		EthercatMotor(const EthercatMotor &other) = delete;
		EthercatMotor(EthercatMotor &&other) = delete;
		EthercatMotor& operator=(const EthercatMotor &other) = delete;
		EthercatMotor& operator=(EthercatMotor &&other) = delete;

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif