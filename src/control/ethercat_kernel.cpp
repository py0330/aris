#ifdef ARIS_USE_ETHERLAB
extern "C"
{
#include <ecrt.h>
}
#endif

#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#include <algorithm>

#include "aris/control/ethercat_kernel.hpp"
#include "aris/control/ethercat.hpp"
#include "aris/control/rt_timer.hpp"
#include "aris/core/log.hpp"

namespace aris::control
{
	const unsigned char FF = std::uint8_t(0xff);
	
	// 没有 offset 的版本
	void read_bit2(char *data, int bit_size, const char *pd, int bit_position)
	{
		// data:
		//            bit_size                                                
		//   7 6 5 4     3      2 1 0 | 7 6 5 4 3 2 1 0 | ... | 7 6 5 4 3 2 1 0 |
		// 
		// pd:
		//                   offset          bit_position  
		//   0 1 2 3 4 5 6 7   |    0 1 2 3       4       5 6 7 | ...  
		//   


		// 注意 >>在某些编译器下，是补符号位，因此必须先转换成uint8
		for (int i = 0; i < bit_size / 8; ++i)
		{
			data[i] = (pd[i] >> bit_position) | (std::uint8_t(pd[i + 1]) << (8 - bit_position));
		}

		if (bit_size % 8)
		{
			// 先将还没弄好的位置零 //
			data[bit_size / 8] &= FF << bit_size % 8;
			data[bit_size / 8] |= (pd[bit_size / 8] >> bit_position) & (0xff >> (8 - bit_size % 8));
			if (bit_size % 8 > 8 - bit_position)
				data[bit_size / 8] |= (std::uint8_t(pd[bit_size / 8 + 1]) << (8 - bit_position)) & (0xff >> (8 - bit_size % 8));

		}
	}
	void write_bit2(const char *data, int bit_size, char *pd, int bit_position)
	{
		for (int i = 0; i < bit_size / 8; ++i)
		{
			pd[i] &= FF >> (8 - bit_position);
			pd[i] |= std::uint8_t(data[i]) << bit_position;
			pd[i + 1] &= FF << bit_position;
			pd[i + 1] |= std::uint8_t(data[i]) >> (8 - bit_position);
		}

		if (bit_size % 8)
		{
			if (bit_size % 8 > 8 - bit_position)
			{
				pd[bit_size / 8] &= FF >> (8 - bit_position);
				pd[bit_size / 8] |= std::uint8_t(data[bit_size / 8]) << bit_position;
				pd[bit_size / 8 + 1] &= FF << ((bit_size % 8) - (8 - bit_position));
				pd[bit_size / 8 + 1] |= std::uint8_t(data[bit_size / 8] & (0xff >> (8 - bit_size % 8))) >> (8 - bit_position);
			}
			else
			{
				pd[bit_size / 8] &= ~(std::uint8_t(FF << (8 - bit_position - (bit_size % 8)) >> (8 - (bit_size % 8))) << bit_position);
				pd[bit_size / 8] |= std::uint8_t((FF >> (8 - (bit_size % 8))) & data[bit_size / 8]) << bit_position;
			}
		}
	}

	void read_bit2(char *data, int bit_size, const char *pd, int offset, int bit_position)
	{
		// data:
		//            bit_size                                                
		//   7 6 5 4     3      2 1 0 | 7 6 5 4 3 2 1 0 | ... | 7 6 5 4 3 2 1 0 |
		// 
		// pd:
		//                   offset          bit_position  
		//   0 1 2 3 4 5 6 7   |    0 1 2 3       4       5 6 7 | ...  
		//   


		// 注意 >>在某些编译器下，是补符号位，因此必须先转换成uint8
		for (int i = 0; i < bit_size / 8; ++i)
		{
			data[i] = (pd[offset + i] >> bit_position) | (std::uint8_t(pd[offset + i + 1]) << (8 - bit_position));
		}

		if (bit_size % 8)
		{
			// 先将还没弄好的位置零 //
			data[bit_size / 8] &= FF << bit_size % 8;
			data[bit_size / 8] |= (pd[offset + bit_size / 8] >> bit_position) & (0xff >> (8 - bit_size % 8));
			if (bit_size % 8 > 8 - bit_position)
				data[bit_size / 8] |= (std::uint8_t(pd[offset + bit_size / 8 + 1]) << (8 - bit_position)) & (0xff >> (8 - bit_size % 8));

		}
	}
	void write_bit2(const char *data, int bit_size, char *pd, int offset, int bit_position)
	{
		for (int i = 0; i < bit_size / 8; ++i)
		{
			pd[offset + i] &= FF >> (8 - bit_position);
			pd[offset + i] |= std::uint8_t(data[i]) << bit_position;
			pd[offset + i + 1] &= FF << bit_position;
			pd[offset + i + 1] |= std::uint8_t(data[i]) >> (8 - bit_position);
		}

		if (bit_size % 8)
		{
			if (bit_size % 8 > 8 - bit_position)
			{
				pd[offset + bit_size / 8] &= FF >> (8 - bit_position);
				pd[offset + bit_size / 8] |= std::uint8_t(data[bit_size / 8]) << bit_position;
				pd[offset + bit_size / 8 + 1] &= FF << ((bit_size % 8) - (8 - bit_position));
				pd[offset + bit_size / 8 + 1] |= std::uint8_t(data[bit_size / 8] & (0xff >> (8 - bit_size % 8))) >> (8 - bit_position);
			}
			else
			{
				pd[offset + bit_size / 8] &= ~(std::uint8_t(FF << (8 - bit_position - (bit_size % 8)) >> (8 - (bit_size % 8))) << bit_position);
				pd[offset + bit_size / 8] |= std::uint8_t((FF >> (8 - (bit_size % 8))) & data[bit_size / 8]) << bit_position;
			}
		}
	}

	void ARIS_API read_bit(char *data, int bit_size, const char *pd, int offset, int bit_position)
	{
		// data:
		//                                                   bit_size
		//   0 1 2 3 4 5 6 7 | 0 1 2 3 4 5 6 7 | ... | 0 1 2    3      ... |
		// 
		// pd:
		//                   offset          bit_position  
		//   0 1 2 3 4 5 6 7   |    0 1 2 3       4       5 6 7 | ...  
 		//   
		
		
		// 注意 >>在某些编译器下，是补符号位，因此必须先转换成uint8
		for (int i = 0; i < bit_size / 8; ++i)
		{
			data[i] = (pd[offset + i] << bit_position) | (std::uint8_t(pd[offset + i + 1]) >> (8 - bit_position));
		}

		if (bit_size % 8)
		{
			// 先将还没弄好的位置零 //
			data[bit_size / 8] &= FF >> bit_size % 8;
			data[bit_size / 8] |= (pd[offset + bit_size / 8] << bit_position) & (0xff << (8 - bit_size % 8));
			if (bit_size % 8 > 8 - bit_position)
				data[bit_size / 8] |= (std::uint8_t(pd[offset + bit_size / 8 + 1]) >> (8 - bit_position)) & (0xff << (8 - bit_size % 8));
		}
	}
	void ARIS_API write_bit(const char *data, int bit_size, char *pd, int offset, int bit_position)
	{
		for (int i = 0; i < bit_size / 8; ++i)
		{
			pd[offset + i] &= FF << (8 - bit_position);
			pd[offset + i] |= std::uint8_t(data[i]) >> bit_position;
			pd[offset + i + 1] &= FF >> bit_position;
			pd[offset + i + 1] |= std::uint8_t(data[i]) << (8 - bit_position);
		}

		if (bit_size % 8)
		{
			if (bit_size % 8 > 8 - bit_position)
			{
				pd[offset + bit_size / 8] &= FF << (8 - bit_position);
				pd[offset + bit_size / 8] |= std::uint8_t(data[bit_size / 8]) >> bit_position;
				pd[offset + bit_size / 8 + 1] &= FF >> ((bit_size % 8) - (8 - bit_position));
				pd[offset + bit_size / 8 + 1] |= std::uint8_t(data[bit_size / 8] & (0xff << (8 - bit_size % 8))) << (8 - bit_position);
			}
			else
			{
				pd[offset + bit_size / 8] &= ~(std::uint8_t(FF >> (8 - bit_position - (bit_size % 8)) << (8 - (bit_size % 8))) >> bit_position);
				pd[offset + bit_size / 8] |= std::uint8_t((FF << (8 - (bit_size % 8))) & data[bit_size / 8]) >> bit_position;
			}
		}
	}

#ifdef ARIS_USE_ETHERLAB
	auto aris_ecrt_scan(EthercatMaster *master)->int
	{
		ec_master_t* ec_master;
		if (!(ec_master = ecrt_request_master(0))) THROW_FILE_LINE("master request failed!");

		ec_master_info_t ec_master_info;
		if (ecrt_master(ec_master, &ec_master_info)) THROW_FILE_LINE("master info failed!");

		std::vector<ec_slave_info_t> ec_slave_info_vec(ec_master_info.slave_count);
		std::vector<std::vector<ec_sync_info_t> > ec_sync_info_vec_vec(ec_master_info.slave_count);
		std::vector<std::vector<std::vector<ec_pdo_info_t> > > ec_pdo_info_vec_vec_vec(ec_master_info.slave_count);
		std::vector<std::vector<std::vector<std::vector<ec_pdo_entry_info_t> > > > ec_pdo_entry_info_vec_vec_vec_vec(ec_master_info.slave_count);
		for (uint16_t sla_pos = 0; sla_pos < ec_master_info.slave_count; ++sla_pos)
		{
			if (ecrt_master_get_slave(ec_master, sla_pos, ec_slave_info_vec.data() + sla_pos))THROW_FILE_LINE("slave info failed!");

			ec_sync_info_vec_vec[sla_pos].resize(ec_slave_info_vec[sla_pos].sync_count);
			ec_pdo_info_vec_vec_vec[sla_pos].resize(ec_slave_info_vec[sla_pos].sync_count);
			ec_pdo_entry_info_vec_vec_vec_vec[sla_pos].resize(ec_slave_info_vec[sla_pos].sync_count);
			for (uint8_t sync_pos = 0; sync_pos < ec_slave_info_vec[sla_pos].sync_count; ++sync_pos)
			{
				if (ecrt_master_get_sync_manager(ec_master, sla_pos, sync_pos, ec_sync_info_vec_vec[sla_pos].data() + sync_pos))THROW_FILE_LINE("sync info failed!");

				ec_pdo_info_vec_vec_vec[sla_pos][sync_pos].resize(ec_sync_info_vec_vec[sla_pos][sync_pos].n_pdos);
				ec_pdo_entry_info_vec_vec_vec_vec[sla_pos][sync_pos].resize(ec_sync_info_vec_vec[sla_pos][sync_pos].n_pdos);
				for (unsigned int pdo_pos = 0; pdo_pos < ec_sync_info_vec_vec[sla_pos][sync_pos].n_pdos; ++pdo_pos)
				{
					if (ecrt_master_get_pdo(ec_master, sla_pos, sync_pos, pdo_pos, ec_pdo_info_vec_vec_vec[sla_pos][sync_pos].data() + pdo_pos))THROW_FILE_LINE("pdo info failed!");
					ec_pdo_entry_info_vec_vec_vec_vec[sla_pos][sync_pos][pdo_pos].resize(ec_pdo_info_vec_vec_vec[sla_pos][sync_pos][pdo_pos].n_entries);

					for (unsigned int entry_pos = 0; entry_pos < ec_pdo_info_vec_vec_vec[sla_pos][sync_pos][pdo_pos].n_entries; ++entry_pos)
					{
						if (ecrt_master_get_pdo_entry(ec_master, sla_pos, sync_pos, pdo_pos, entry_pos, ec_pdo_entry_info_vec_vec_vec_vec[sla_pos][sync_pos][pdo_pos].data() + entry_pos))THROW_FILE_LINE("entry info failed!");
					}
				}
			}
		}
		// 释放master //
		ecrt_release_master(ec_master);

		master->slavePool().clear();
		for (uint16_t sla_pos = 0; sla_pos < ec_master_info.slave_count; ++sla_pos)
		{
			auto &info = ec_slave_info_vec[sla_pos];
			auto &sla = master->slavePool().add<EthercatSlave>(info.name, sla_pos, info.vendor_id, info.product_code, info.revision_number);

			for (uint8_t sync_pos = 0; sync_pos < ec_slave_info_vec[sla_pos].sync_count; ++sync_pos)
			{
				auto &info = ec_sync_info_vec_vec[sla_pos][sync_pos];
				sla.smPool().push_back(SyncManager("sm", info.dir == EC_DIR_INPUT));
				
				for (unsigned int pdo_pos = 0; pdo_pos < ec_sync_info_vec_vec[sla_pos][sync_pos].n_pdos; ++pdo_pos)
				{
					auto &pdo_info = ec_pdo_info_vec_vec_vec[sla_pos][sync_pos][pdo_pos];
					sla.smPool()[sync_pos].push_back(Pdo("pdo", pdo_info.index));
					
					for (unsigned int entry_pos = 0; entry_pos < ec_pdo_info_vec_vec_vec[sla_pos][sync_pos][pdo_pos].n_entries; ++entry_pos)
					{
						auto &info = ec_pdo_entry_info_vec_vec_vec_vec[sla_pos][sync_pos][pdo_pos][entry_pos];
						sla.smPool()[sync_pos][pdo_pos].push_back(PdoEntry("entry", info.index, info.subindex, info.bit_length));
					}
				}
			}
		}

		return 0;
	}

	struct MasterHandle
	{
		ec_master_t* ec_master_;
		ec_domain_t* domain_;
		std::uint8_t* domain_pd_;
	};
	struct SlaveHandle
	{
		ec_slave_config_t* ec_slave_config_;
	};
	struct PdoEntryHandle
	{
		union 
		{
			std::uint32_t offset_;
			char *data_;
		};
		std::uint32_t bit_position;
	};

	auto aris_ecrt_master_request(EthercatMaster *master)->void
	{
		// check if product code and vendor id is paired
		aris::control::EthercatMaster local_mst;
		if (aris_ecrt_scan(&local_mst))THROW_FILE_LINE("scan slaves failed!");
		for (int i = 0; i< master->slavePool().size();++i)
		{
			auto &slave = master->slavePool()[i];
			
			if (auto ec_slave = dynamic_cast<aris::control::EthercatSlave*>(&slave))
			{
				if (slave.isVirtual())continue;
				
				if(slave.phyId() > local_mst.slavePool().size()) THROW_FILE_LINE("wrong physical id!");
				
				auto compared_slave = dynamic_cast<aris::control::EthercatSlave*>(&local_mst.slavePool().at(slave.phyId()));
				if (ec_slave->productCode() != compared_slave->productCode()) THROW_FILE_LINE(":wrong product code of slave " + std::to_string(i));
				if (ec_slave->vendorID() != compared_slave->vendorID()) THROW_FILE_LINE(":wrong vendor id of slave " + std::to_string(i));
			}
		}
		// check finished

		// make subfunction， which start the master
		auto start_ethercat = [](EthercatMaster *master) 
		{
			MasterHandle m_handle{ nullptr, nullptr, nullptr };

			// request master //
			if (!(m_handle.ec_master_ = ecrt_request_master(0)))THROW_FILE_LINE("master request failed!");

			// create domain //
			if (!(m_handle.domain_ = ecrt_master_create_domain(m_handle.ec_master_)))THROW_FILE_LINE("failed to create domain");

			// make slaves //
			std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec;
			for (auto &slave : master->slavePool())
			{
				if (slave.isVirtual())continue;
				
				std::vector<ec_sync_info_t> ec_sync_info_vec;
				std::vector<std::vector<ec_pdo_info_t> > ec_pdo_info_vec_vec;
				std::vector<std::vector<std::vector<ec_pdo_entry_info_t> > > ec_pdo_entry_info_vec_vec_vec;

				for (int i = 0; i< slave.smPool().size();++i)
				{
					auto &sm = slave.smPool()[i];
					
					ec_pdo_info_vec_vec.push_back(std::vector<ec_pdo_info_t>());
					ec_pdo_entry_info_vec_vec_vec.push_back(std::vector<std::vector<ec_pdo_entry_info_t> >());

					for (auto &pdo : sm)
					{
						ec_pdo_entry_info_vec_vec_vec.back().push_back(std::vector<ec_pdo_entry_info_t>());
						for (auto &entry : pdo)
						{
							entry.ecHandle() = PdoEntryHandle();
							auto &pe_handle = std::any_cast<PdoEntryHandle&>(entry.ecHandle());

							//etherlab 会根据index是否为0来判断是否结束
							if (entry.index())ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{ 0x00, slave.phyId(), slave.vendorID(), slave.productCode(), entry.index(), entry.subindex(), &pe_handle.offset_, &pe_handle.bit_position });
							ec_pdo_entry_info_vec_vec_vec.back().back().push_back(ec_pdo_entry_info_t{ entry.index(), entry.subindex(), static_cast<std::uint8_t>(entry.bitSize()) });
						}

						ec_pdo_info_vec_vec.back().push_back(ec_pdo_info_t{ pdo.index(),
							static_cast<std::uint8_t>(ec_pdo_entry_info_vec_vec_vec.back().back().size()), ec_pdo_entry_info_vec_vec_vec.back().back().data() });
					}

					ec_sync_info_vec.push_back(ec_sync_info_t{ static_cast<std::uint8_t>(i), sm.tx() ? EC_DIR_INPUT : EC_DIR_OUTPUT,
						static_cast<unsigned int>(ec_pdo_info_vec_vec.back().size()), ec_pdo_info_vec_vec.back().data(), EC_WD_DEFAULT });
				}


				SlaveHandle s_handle;

				// Get the slave configuration 
				if (!(s_handle.ec_slave_config_ = ecrt_master_slave_config(m_handle.ec_master_, 0x00, slave.phyId(), slave.vendorID(), slave.productCode()))) { THROW_FILE_LINE("failed to slave config"); }

				// Configure the slave's PDOs and sync masters
				if (ecrt_slave_config_pdos(s_handle.ec_slave_config_, ec_sync_info_vec.size(), ec_sync_info_vec.data()))THROW_FILE_LINE("failed to slave config pdos");

				// Configure the slave's distributed clock
				if (slave.dcAssignActivate())ecrt_slave_config_dc(s_handle.ec_slave_config_, slave.dcAssignActivate(), master->samplePeriodNs(), 500000, 0, 0);

				slave.ecHandle() = s_handle;
			}

			// configure domain
			ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{});
			if (ecrt_domain_reg_pdo_entry_list(m_handle.domain_, ec_pdo_entry_reg_vec.data()))THROW_FILE_LINE("failed domain_reg_pdo_entry");

			// activate master
			if (ecrt_master_activate(m_handle.ec_master_)) { THROW_FILE_LINE("failed activate master, perhaps pdo map is wrong"); }
			if (!(m_handle.domain_pd_ = ecrt_domain_data(m_handle.domain_)))THROW_FILE_LINE("failed ecrt_domain_data");

			// set handle
			master->ecHandle() = m_handle;

			// make pdo init value to zero
			for (auto &slave : master->slavePool()){
				for (auto &sm : slave.smPool()){
					for (auto &pdo : sm){
						for (auto &entry : pdo){
							if (entry.index()){
								auto &pe_handle = std::any_cast<PdoEntryHandle&>(entry.ecHandle());
								pe_handle.data_ = (char*)std::any_cast<MasterHandle&>(master->ecHandle()).domain_pd_ + pe_handle.offset_;
								
								std::vector<char> value(entry.bitSize() / 8 + 1, 0);
								aris_ecrt_pdo_write(&entry, value.data());
							}
						}
					}
				}
			}
		};
		

		// check pdos 
		/*
		aris::control::EthercatMaster check_master_pdos;
		check_master_pdos.slavePool() = master->slavePool();

		for (auto &sla : check_master_pdos.slavePool())
			if (dynamic_cast<EthercatSlave*>(&sla))
				check_master_pdos.slavePool().push_back_ptr(dynamic_cast<EthercatSlave*>(&sla));


		start_ethercat(&check_master_pdos);
		aris_ecrt_master_stop(&check_master_pdos);

		std::cout << check_master_pdos.xmlString() <<std::endl;

		for (auto &slave : master->slavePool())
		{
			if (auto ec_slave = dynamic_cast<aris::control::EthercatSlave*>(&slave))
			{
				if (slave.phyId() > local_mst.slavePool().size()) THROW_FILE_LINE(":wrong physical id!");

				// check product code and vendor id
				auto compared_slave = dynamic_cast<aris::control::EthercatSlave*>(&local_mst.slavePool().at(slave.phyId()));
				if (ec_slave->productCode() != compared_slave->productCode()) THROW_FILE_LINE(":wrong product code of slave " + std::to_string(ec_slave->id());
				if (ec_slave->vendorID() != compared_slave->vendorID()) THROW_FILE_LINE(":wrong vendor id of slave " + std::to_string(ec_slave->id());

				for (int i = 0; i<ec_slave->smPool().size(); ++i)
				{
					if (i >= compared_slave->smPool().size()) THROW_FILE_LINE("map pdo failed: sm num not correct");
					
					auto &sm = ec_slave->smPool()[i];
					auto &compared_sm = compared_slave->smPool()[i];
					
					// check if sm rx & tx valid 
					if(sm.tx() != compared_sm.tx()) THROW_FILE_LINE("map pdo failed: sm tx or rx not correct");

					for (int j = 0; j<sm.size(); ++j)
					{
						if (j >= compared_sm.size()) THROW_FILE_LINE("map pdo failed: pdo num not correct");
						
						auto &pdo = sm[j];
						auto &compared_pdo = compared_sm[j];
						
						// check pdo index valid 
						if (pdo.index() != compared_pdo.index()) THROW_FILE_LINE("map pdo failed: pdo index not correct");

						for (int k = 0; k<pdo.size(); ++k)
						{
							if (k >= compared_pdo.size()) THROW_FILE_LINE("map pdo failed: entry num not correct");
							
							auto &entry = pdo[k];
							auto &compared_entry = compared_pdo[k];

							if ((entry.index() != compared_entry.index()) 
								|| (entry.subindex() != compared_entry.subindex())
								|| (entry.bitSize() != compared_entry.bitSize())
								)
							{
								THROW_FILE_LINE("map pdo failed: entry info not correct");
							}
						}
					}
				}
			}
		}
		*/
		// check pdos finished
		





		// finally start the master
		start_ethercat(master);
	}
	auto aris_ecrt_master_stop(EthercatMaster *master)->void
	{
		ecrt_master_deactivate(std::any_cast<MasterHandle&>(master->ecHandle()).ec_master_);
		ecrt_release_master(std::any_cast<MasterHandle&>(master->ecHandle()).ec_master_);
	}
	auto aris_ecrt_master_recv(EthercatMaster *mst)->void
	{
		auto &m_handle = std::any_cast<MasterHandle&>(mst->ecHandle());
		
		ec_master_state_t ms;
		ecrt_master_state(m_handle.ec_master_, &ms);

		if (ms.link_up)
		{
			ecrt_master_receive(m_handle.ec_master_);
			ecrt_domain_process(m_handle.domain_);
		}
	}
	auto aris_ecrt_master_send(EthercatMaster *mst)->void
	{
		auto &m_handle = std::any_cast<MasterHandle&>(mst->ecHandle());
		
		ec_master_state_t ms;
		ecrt_master_state(m_handle.ec_master_, &ms);

		if (ms.link_up)
		{
			ecrt_domain_queue(m_handle.domain_);

			ecrt_master_application_time(m_handle.ec_master_, aris_rt_timer_read());
			ecrt_master_sync_reference_clock(m_handle.ec_master_);
			ecrt_master_sync_slave_clocks(m_handle.ec_master_);

			ecrt_master_send(m_handle.ec_master_);
		}
	}

	auto aris_ecrt_master_link_state(EthercatMaster* mst, EthercatMaster::MasterLinkState *ms, EthercatMaster::SlaveLinkState *ss)->void 
	{
		ecrt_master_state(std::any_cast<MasterHandle&>(mst->ecHandle()).ec_master_, reinterpret_cast<ec_master_state_t*>(ms));
		for (int i = 0; i < mst->slavePool().size(); ++i)
		{
			ecrt_slave_config_state(std::any_cast<SlaveHandle&>(mst->slavePool()[i].ecHandle()).ec_slave_config_, reinterpret_cast<ec_slave_config_state_t*>(ss + i));
		}
	}

	auto aris_ecrt_pdo_read(const PdoEntry *entry, void *data)->void
	{
		auto &pe_handle = std::any_cast<const PdoEntryHandle&>(entry->ecHandle());
		read_bit2(reinterpret_cast<char*>(data), entry->bitSize(), pe_handle.data_, pe_handle.bit_position);
	}
	auto aris_ecrt_pdo_write(PdoEntry *entry, const void *data)->void
	{
		auto &pe_handle = std::any_cast<PdoEntryHandle&>(entry->ecHandle());
		write_bit2(reinterpret_cast<const char*>(data), entry->bitSize(), pe_handle.data_, pe_handle.bit_position);
	}

	auto aris_ecrt_sdo_config(std::any& master, std::any& slave, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *buffer, std::size_t byte_size)->void
	{
		auto &ec_slave_config = std::any_cast<SlaveHandle&>(slave).ec_slave_config_;
		ecrt_slave_config_sdo(ec_slave_config, index, subindex, buffer, byte_size * 8);
	}
	auto aris_ecrt_sdo_read(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::size_t *result_size, std::uint32_t *abort_code)->int
	{
		return ecrt_master_sdo_upload(std::any_cast<MasterHandle&>(master).ec_master_, slave_position, index, subindex, to_buffer, buffer_size, result_size, abort_code);
	}
	auto aris_ecrt_sdo_write(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::uint32_t *abort_code)->int
	{
		return ecrt_master_sdo_download(std::any_cast<MasterHandle&>(master).ec_master_, slave_position, index, subindex, to_buffer, buffer_size, abort_code);
	}
#else
	auto aris_ecrt_scan(EthercatMaster *master)->int { return 0; }
	auto aris_ecrt_master_request(EthercatMaster *master)->void {}
	auto aris_ecrt_master_stop(EthercatMaster *master)->void {}
	auto aris_ecrt_master_recv(EthercatMaster *master)->void {}
	auto aris_ecrt_master_send(EthercatMaster *master)->void {}
	auto aris_ecrt_master_link_state(EthercatMaster* mst, EthercatMaster::MasterLinkState *ms, EthercatMaster::SlaveLinkState *ss)->void {}

	auto aris_ecrt_pdo_read(const PdoEntry *entry, void *data)->void {}
	auto aris_ecrt_pdo_write(PdoEntry *entry, const void *data)->void {}
	auto aris_ecrt_sdo_read(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::size_t *result_size, std::uint32_t *abort_code) ->int {
		return 0;
	}
	auto aris_ecrt_sdo_write(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::uint32_t *abort_code) ->int {
		return 0;
	}
	auto aris_ecrt_sdo_config(std::any& master, std::any& slave, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *buffer, std::size_t bit_size)->void {}
#endif
}
