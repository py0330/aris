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

namespace aris::control
{
	const unsigned char FF = std::uint8_t(0xff);
	
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

	void read_bit(char *data, int bit_size, const char *pd, int offset, int bit_position)
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
	void write_bit(const char *data, int bit_size, char *pd, int offset, int bit_position)
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
		if (!(ec_master = ecrt_request_master(0))) throw std::runtime_error("master request failed!");

		ec_master_info_t ec_master_info;
		if (ecrt_master(ec_master, &ec_master_info)) throw std::runtime_error("master info failed!");

		std::vector<ec_slave_info_t> ec_slave_info_vec(ec_master_info.slave_count);
		std::vector<std::vector<ec_sync_info_t> > ec_sync_info_vec_vec(ec_master_info.slave_count);
		std::vector<std::vector<std::vector<ec_pdo_info_t> > > ec_pdo_info_vec_vec_vec(ec_master_info.slave_count);
		std::vector<std::vector<std::vector<std::vector<ec_pdo_entry_info_t> > > > ec_pdo_entry_info_vec_vec_vec_vec(ec_master_info.slave_count);
		for (uint16_t sla_pos = 0; sla_pos < ec_master_info.slave_count; ++sla_pos)
		{
			if (ecrt_master_get_slave(ec_master, sla_pos, ec_slave_info_vec.data() + sla_pos))throw std::runtime_error("slave info failed!");

			ec_sync_info_vec_vec[sla_pos].resize(ec_slave_info_vec[sla_pos].sync_count);
			ec_pdo_info_vec_vec_vec[sla_pos].resize(ec_slave_info_vec[sla_pos].sync_count);
			ec_pdo_entry_info_vec_vec_vec_vec[sla_pos].resize(ec_slave_info_vec[sla_pos].sync_count);
			for (uint8_t sync_pos = 0; sync_pos < ec_slave_info_vec[sla_pos].sync_count; ++sync_pos)
			{
				if (ecrt_master_get_sync_manager(ec_master, sla_pos, sync_pos, ec_sync_info_vec_vec[sla_pos].data() + sync_pos))throw std::runtime_error("sync info failed!");

				ec_pdo_info_vec_vec_vec[sla_pos][sync_pos].resize(ec_sync_info_vec_vec[sla_pos][sync_pos].n_pdos);
				ec_pdo_entry_info_vec_vec_vec_vec[sla_pos][sync_pos].resize(ec_sync_info_vec_vec[sla_pos][sync_pos].n_pdos);
				for (unsigned int pdo_pos = 0; pdo_pos < ec_sync_info_vec_vec[sla_pos][sync_pos].n_pdos; ++pdo_pos)
				{
					if (ecrt_master_get_pdo(ec_master, sla_pos, sync_pos, pdo_pos, ec_pdo_info_vec_vec_vec[sla_pos][sync_pos].data() + pdo_pos))throw std::runtime_error("pdo info failed!");
					ec_pdo_entry_info_vec_vec_vec_vec[sla_pos][sync_pos][pdo_pos].resize(ec_pdo_info_vec_vec_vec[sla_pos][sync_pos][pdo_pos].n_entries);

					for (unsigned int entry_pos = 0; entry_pos < ec_pdo_info_vec_vec_vec[sla_pos][sync_pos][pdo_pos].n_entries; ++entry_pos)
					{
						if (ecrt_master_get_pdo_entry(ec_master, sla_pos, sync_pos, pdo_pos, entry_pos, ec_pdo_entry_info_vec_vec_vec_vec[sla_pos][sync_pos][pdo_pos].data() + entry_pos))throw std::runtime_error("entry info failed!");
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
				sla.smPool().add<SyncManager>("sm", info.dir == EC_DIR_INPUT);
				
				for (unsigned int pdo_pos = 0; pdo_pos < ec_sync_info_vec_vec[sla_pos][sync_pos].n_pdos; ++pdo_pos)
				{
					auto &pdo_info = ec_pdo_info_vec_vec_vec[sla_pos][sync_pos][pdo_pos];
					sla.smPool()[sync_pos].add<Pdo>("pdo", pdo_info.index);
					
					for (unsigned int entry_pos = 0; entry_pos < ec_pdo_info_vec_vec_vec[sla_pos][sync_pos][pdo_pos].n_entries; ++entry_pos)
					{
						auto &info = ec_pdo_entry_info_vec_vec_vec_vec[sla_pos][sync_pos][pdo_pos][entry_pos];
						sla.smPool()[sync_pos][pdo_pos].add<PdoEntry>("entry", info.index, info.subindex, info.bit_length);
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
		std::uint32_t offset;
		std::uint32_t bit_position;
	};

	auto aris_ecrt_master_request(EthercatMaster *master)->void
	{
		// check if product code and vendor id is paired
		aris::control::EthercatMaster local_mst;
		if(aris_ecrt_scan(&local_mst))throw std::runtime_error((std::string(__FILE__) + std::to_string(__LINE__) + ":scan slaves failed!").c_str());
		for (auto &slave : master->slavePool())
		{
			if (auto ec_slave = dynamic_cast<aris::control::EthercatSlave*>(&slave))
			{
				if(slave.phyId() > local_mst.slavePool().size()) throw std::runtime_error((std::string(__FILE__) + std::to_string(__LINE__) + ":wrong physical id!").c_str());
				
				auto compared_slave = dynamic_cast<aris::control::EthercatSlave*>(&local_mst.slavePool().at(slave.phyId()));
				if (ec_slave->productCode() != compared_slave->productCode()) throw std::runtime_error((std::string(__FILE__) + std::to_string(__LINE__) + ":wrong product code of slave " + std::to_string(ec_slave->id())).c_str());
				if (ec_slave->vendorID() != compared_slave->vendorID()) throw std::runtime_error((std::string(__FILE__) + std::to_string(__LINE__) + ":wrong vendor id of slave " + std::to_string(ec_slave->id())).c_str());
			}
		}
		// check finished

		

		// make subfunction
		auto start_ethercat = [](EthercatMaster *master) 
		{
			MasterHandle m_handle{ nullptr, nullptr, nullptr };

			// request master //
			if (!(m_handle.ec_master_ = ecrt_request_master(0)))throw std::runtime_error((std::string(__FILE__) + std::to_string(__LINE__) + ":master request failed!").c_str());

			// create domain //
			if (!(m_handle.domain_ = ecrt_master_create_domain(m_handle.ec_master_)))throw std::runtime_error(std::string(__FILE__) + std::to_string(__LINE__) + "failed to create domain");

			// make slaves //
			std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec;
			for (auto &slave : master->ecSlavePool())
			{
				std::vector<ec_sync_info_t> ec_sync_info_vec;
				std::vector<std::vector<ec_pdo_info_t> > ec_pdo_info_vec_vec;
				std::vector<std::vector<std::vector<ec_pdo_entry_info_t> > > ec_pdo_entry_info_vec_vec_vec;

				for (auto &sm : slave.smPool())
				{
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
							if (entry.index())ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{ 0x00, slave.phyId(), slave.vendorID(), slave.productCode(), entry.index(), entry.subindex(), &pe_handle.offset, &pe_handle.bit_position });
							ec_pdo_entry_info_vec_vec_vec.back().back().push_back(ec_pdo_entry_info_t{ entry.index(), entry.subindex(), static_cast<std::uint8_t>(entry.bitSize()) });
						}

						ec_pdo_info_vec_vec.back().push_back(ec_pdo_info_t{ pdo.index(),
							static_cast<std::uint8_t>(ec_pdo_entry_info_vec_vec_vec.back().back().size()), ec_pdo_entry_info_vec_vec_vec.back().back().data() });
					}

					ec_sync_info_vec.push_back(ec_sync_info_t{ static_cast<std::uint8_t>(sm.id()), sm.tx() ? EC_DIR_INPUT : EC_DIR_OUTPUT,
						static_cast<unsigned int>(ec_pdo_info_vec_vec.back().size()), ec_pdo_info_vec_vec.back().data(), EC_WD_DEFAULT });
				}


				SlaveHandle s_handle;

				// Get the slave configuration 
				if (!(s_handle.ec_slave_config_ = ecrt_master_slave_config(m_handle.ec_master_, 0x00, slave.phyId(), slave.vendorID(), slave.productCode()))) { throw std::runtime_error("failed to slave config"); }

				// Configure the slave's PDOs and sync masters
				if (ecrt_slave_config_pdos(s_handle.ec_slave_config_, ec_sync_info_vec.size(), ec_sync_info_vec.data()))throw std::runtime_error("failed to slave config pdos");

				// Configure the slave's distributed clock
				if (slave.dcAssignActivate())ecrt_slave_config_dc(s_handle.ec_slave_config_, slave.dcAssignActivate(), 1000000, 4400000, 0, 0);

				slave.ecHandle() = s_handle;
			}

			// configure domain
			ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{});
			if (ecrt_domain_reg_pdo_entry_list(m_handle.domain_, ec_pdo_entry_reg_vec.data()))throw std::runtime_error("failed domain_reg_pdo_entry");

			// activate master
			if (ecrt_master_activate(m_handle.ec_master_)) { throw std::runtime_error("failed activate master, perhaps pdo map is wrong"); }
			if (!(m_handle.domain_pd_ = ecrt_domain_data(m_handle.domain_)))throw std::runtime_error("failed ecrt_domain_data");

			// set handle
			master->ecHandle() = m_handle;

			// make pdo init value to zero
			for (auto &slave : master->ecSlavePool())
			{
				for (auto &sm : slave.smPool())
				{
					for (auto &pdo : sm)
					{
						for (auto &entry : pdo)
						{
							if (entry.index())
							{
								std::vector<char> value(entry.bitSize() / 8 + 1, 0);
								aris_ecrt_pdo_write(&entry, value.data(), entry.bitSize());
							}
						}
					}
				}
			}
		};
		

		// check pdos 
		aris::control::EthercatMaster check_master_pdos;
		check_master_pdos.slavePool() = master->slavePool();

		start_ethercat(&check_master_pdos);
		aris_ecrt_master_stop(&check_master_pdos);

		std::cout << check_master_pdos.xmlString() <<std::endl;
		// check pdos finished



		// finally start the master
		start_ethercat(master);
	}
	auto aris_ecrt_master_stop(EthercatMaster *master)->void
	{
		ecrt_master_deactivate(std::any_cast<MasterHandle&>(master->ecHandle()).ec_master_);
		ecrt_release_master(std::any_cast<MasterHandle&>(master->ecHandle()).ec_master_);
	}
	auto aris_ecrt_master_sync(EthercatMaster *master, std::uint64_t ns)->void
	{
		ecrt_master_application_time(std::any_cast<MasterHandle&>(master->ecHandle()).ec_master_, ns);
		ecrt_master_sync_reference_clock(std::any_cast<MasterHandle&>(master->ecHandle()).ec_master_);
		ecrt_master_sync_slave_clocks(std::any_cast<MasterHandle&>(master->ecHandle()).ec_master_);
	}
	auto aris_ecrt_master_recv(EthercatMaster *master)->void
	{
		ecrt_master_receive(std::any_cast<MasterHandle&>(master->ecHandle()).ec_master_);
		ecrt_domain_process(std::any_cast<MasterHandle&>(master->ecHandle()).domain_);
	}
	auto aris_ecrt_master_send(EthercatMaster *master)->void
	{
		ecrt_domain_queue(std::any_cast<MasterHandle&>(master->ecHandle()).domain_);
		ecrt_master_send(std::any_cast<MasterHandle&>(master->ecHandle()).ec_master_);
	}

	auto aris_ecrt_pdo_read(PdoEntry *entry, void *data, int bit_size)->void
	{
		auto pd = std::any_cast<MasterHandle&>(entry->ancestor<EthercatMaster>()->ecHandle()).domain_pd_;
		auto &pe_handle = std::any_cast<PdoEntryHandle&>(entry->ecHandle());

		read_bit2(reinterpret_cast<char*>(data), bit_size, reinterpret_cast<const char*>(pd), pe_handle.offset, pe_handle.bit_position);
	}
	auto aris_ecrt_pdo_write(PdoEntry *entry, const void *data, int bit_size)->void
	{
		auto pd = std::any_cast<MasterHandle&>(entry->ancestor<EthercatMaster>()->ecHandle()).domain_pd_;
		auto &pe_handle = std::any_cast<PdoEntryHandle&>(entry->ecHandle());

		write_bit2(reinterpret_cast<const char*>(data), bit_size, reinterpret_cast<char*>(pd), pe_handle.offset, pe_handle.bit_position);
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
	auto aris_ecrt_master_sync(EthercatMaster *master, std::uint64_t ns)->void {}
	auto aris_ecrt_master_recv(EthercatMaster *master)->void {}
	auto aris_ecrt_master_send(EthercatMaster *master)->void {}

	auto aris_ecrt_pdo_read(PdoEntry *entry, void *data, int byte_size)->void {}
	auto aris_ecrt_pdo_write(PdoEntry *entry, const void *data, int byte_size)->void {}
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
