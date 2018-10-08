#ifdef USE_ETHERLAB
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

#include "aris_control_ethercat_kernel.h"
#include "aris_control_ethercat.h"

namespace aris::control
{
#ifndef USE_ETHERLAB
	auto aris_ecrt_scan(EthercatMaster *master)->int { return 0;}
	auto aris_ecrt_master_request(EthercatMaster *master)->void {}
	
	auto aris_ecrt_master_stop(std::any& master)->void {}
	auto aris_ecrt_master_sync(std::any& master, std::uint64_t ns)->void {}
	auto aris_ecrt_master_receive(std::any& master)->void {}
	auto aris_ecrt_master_send(std::any& master)->void {}
	
	auto aris_ecrt_slave_send(std::any& slave)->void {}
	auto aris_ecrt_slave_receive(std::any& slave)->void {}
	
	auto aris_ecrt_pdo_read(std::any& slave, std::any& pdo, void *data, int byte_size)->void {	}
	auto aris_ecrt_pdo_write(std::any& slave, std::any& pdo, const void *data, int byte_size)->void { }
	auto aris_ecrt_sdo_read(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::size_t *result_size, std::uint32_t *abort_code) ->int { return 0; }
	auto aris_ecrt_sdo_write(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::uint32_t *abort_code) ->int { return 0; }
	auto aris_ecrt_sdo_config(std::any& master, std::any& slave, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *buffer, std::size_t bit_size)->void {}
	
#endif


	


#ifdef USE_ETHERLAB
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
			auto &sla = master->slavePool().add<EthercatSlave>("slave_" + std::to_string(sla_pos), sla_pos, info.vendor_id, info.product_code, info.revision_number);

			for (uint8_t sync_pos = 0; sync_pos < ec_slave_info_vec[sla_pos].sync_count; ++sync_pos)
			{
				auto &info = ec_sync_info_vec_vec[sla_pos][sync_pos];
				sla.smPool().add<SyncManager>("sm", info.dir == EC_DIR_INPUT);
				
				for (unsigned int pdo_pos = 0; pdo_pos < ec_sync_info_vec_vec[sla_pos][sync_pos].n_pdos; ++pdo_pos)
				{
					auto &pdo_info = ec_pdo_info_vec_vec_vec[sla_pos][sync_pos][pdo_pos];
					sla.smPool()[sync_pos].add<Pdo>("pdo", pdo_info.index, info.dir == EC_DIR_INPUT);
					
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

	using MasterHandle = ec_master_t*;
	struct SlaveHandle
	{
		ec_domain_t* domain_;
		std::uint8_t* domain_pd_;
		ec_slave_config_t* ec_slave_config_;
	};
	using PdoEntryHandle = std::uint32_t;


	auto aris_ecrt_master_request(EthercatMaster *master)->void
	{
		auto mst = ecrt_request_master(0);
		if(mst == nullptr)throw std::runtime_error((std::string(__FILE__) + std::to_string(__LINE__) + ":master request failed!").c_str());
		master->ecHandle() = mst;

		for (auto &slave : master->ecSlavePool())
		{
			std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec;
			std::vector<ec_sync_info_t> ec_sync_info_vec;

			for (auto &sm : slave.smPool())
			{
				std::vector<ec_pdo_info_t> ec_pdo_info_vec;
				for (auto &pdo : sm)
				{
					std::vector<ec_pdo_entry_info_t> ec_pdo_entry_info_vec;
					for (auto &entry : pdo)
					{
						entry.ecHandle() = PdoEntryHandle();
						ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{ 0x00, slave.phyId(), slave.vendorID(), slave.productCode(), entry.index(), entry.subindex(), &(std::any_cast<PdoEntryHandle&>(entry.ecHandle())) });
						ec_pdo_entry_info_vec.push_back(ec_pdo_entry_info_t{ entry.index(), entry.subindex(), static_cast<std::uint8_t>(entry.size() * 8) });
					}

					ec_pdo_info_vec.push_back(ec_pdo_info_t{ pdo.index(), static_cast<std::uint8_t>(ec_pdo_entry_info_vec.size()), ec_pdo_entry_info_vec.data() });
				}

				ec_sync_info_vec.push_back(ec_sync_info_t{ static_cast<std::uint8_t>(sm.id()), sm.tx() ? EC_DIR_INPUT : EC_DIR_OUTPUT, static_cast<unsigned int>(ec_pdo_info_vec.size()), ec_pdo_info_vec.data(), EC_WD_DEFAULT });
			}
			ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{});
			ec_sync_info_vec.push_back(ec_sync_info_t{ 0xff });


			SlaveHandle sla;

			// Create domain
			if (!(sla.domain_ = ecrt_master_create_domain(mst)))throw std::runtime_error("failed to create domain");

			// Get the slave configuration 
			if (!(sla.ec_slave_config_ = ecrt_master_slave_config(mst, 0x00, slave.phyId(), slave.vendorID(), slave.productCode()))) { throw std::runtime_error("failed to slave config"); }

			// Configure the slave's PDOs and sync masters
			if (ecrt_slave_config_pdos(sla.ec_slave_config_, ec_sync_info_vec.size() - 1, ec_sync_info_vec.data()))throw std::runtime_error("failed to slave config pdos");

			// Configure the slave's domain
			if (ecrt_domain_reg_pdo_entry_list(sla.domain_, ec_pdo_entry_reg_vec.data()))throw std::runtime_error("failed domain_reg_pdo_entry");

			// Configure the slave's discrete clock
			ecrt_slave_config_dc(sla.ec_slave_config_, slave.dcAssignActivate(), 1000000, 4400000, 0, 0);

			slave.ecHandle() = sla;
		}

		ecrt_master_activate(mst);
		for (auto &slave : master->ecSlavePool()) 
		{
			auto &domain_pd = std::any_cast<SlaveHandle&>(slave.ecHandle()).domain_pd_;
			if (!(domain_pd = ecrt_domain_data(std::any_cast<SlaveHandle&>(slave.ecHandle()).domain_)))throw std::runtime_error("failed ecrt_domain_data");
		}
	}






	auto aris_ecrt_sdo_config(std::any& master, std::any& slave, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *buffer, std::size_t byte_size)->void
	{
		auto &ec_slave_config = std::any_cast<SlaveHandle&>(slave).ec_slave_config_;
		ecrt_slave_config_sdo(ec_slave_config, index, subindex, buffer, byte_size * 8);
	}


	////////////////////////////////////////////////////////////////////////////////////////
	auto aris_ecrt_master_stop(std::any& master)->void
	{
		ecrt_master_deactivate(std::any_cast<MasterHandle>(master));
		ecrt_release_master(std::any_cast<MasterHandle>(master));
	}


	/////////////////////////////////////////////////////////////////////////////////////////
	auto aris_ecrt_master_sync(std::any& master, std::uint64_t ns)->void
	{
		ecrt_master_application_time(std::any_cast<MasterHandle>(master), ns);
		ecrt_master_sync_reference_clock(std::any_cast<MasterHandle>(master));
		ecrt_master_sync_slave_clocks(std::any_cast<MasterHandle>(master));
	}
	auto aris_ecrt_master_receive(std::any& master)->void
	{
		ecrt_master_receive(std::any_cast<MasterHandle>(master));
	}
	auto aris_ecrt_master_send(std::any& master)->void
	{
		ecrt_master_send(std::any_cast<MasterHandle>(master));
	}

	auto aris_ecrt_slave_send(std::any& slave)->void
	{
		ecrt_domain_queue(std::any_cast<SlaveHandle&>(slave).domain_);
	}
	auto aris_ecrt_slave_receive(std::any& slave)->void
	{
		ecrt_domain_process(std::any_cast<SlaveHandle&>(slave).domain_);
	}
	auto aris_ecrt_pdo_read(std::any& slave, std::any& pdo, void *data, int byte_size)->void
	{
		std::copy_n(std::any_cast<SlaveHandle&>(slave).domain_pd_ + std::any_cast<PdoEntryHandle&>(pdo), byte_size, static_cast<char *>(data));
	}
	auto aris_ecrt_pdo_write(std::any& slave, std::any& pdo, const void *data, int byte_size)->void
	{
		std::copy_n(static_cast<const char *>(data), byte_size, std::any_cast<SlaveHandle&>(slave).domain_pd_ + std::any_cast<PdoEntryHandle&>(pdo));
	}
	auto aris_ecrt_sdo_read(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::size_t *result_size, std::uint32_t *abort_code)->int
	{
		return ecrt_master_sdo_upload(std::any_cast<MasterHandle>(master), slave_position, index, subindex, to_buffer, buffer_size, result_size, abort_code);
	}
	auto aris_ecrt_sdo_write(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::uint32_t *abort_code)->int
	{
		return ecrt_master_sdo_download(std::any_cast<MasterHandle>(master), slave_position, index, subindex, to_buffer, buffer_size, abort_code);
	}

#endif
}
