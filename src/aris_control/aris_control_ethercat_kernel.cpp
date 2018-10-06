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

namespace aris::control
{
#ifndef USE_ETHERLAB
	auto aris_ecrt_master_init()->std::any { return std::any(); }
	auto aris_ecrt_master_config(std::any& master)->void {}
	auto aris_ecrt_master_start(std::any& master)->void {}
	auto aris_ecrt_master_stop(std::any& master)->void {}
	auto aris_ecrt_master_sync(std::any& master, std::uint64_t ns)->void {}
	auto aris_ecrt_master_receive(std::any& master)->void {}
	auto aris_ecrt_master_send(std::any& master)->void {}
	auto aris_ecrt_slave_init()->std::any { return std::any(); }
	auto aris_ecrt_slave_config(std::any& master, std::any& slave, std::uint16_t alias, std::uint16_t position, std::uint32_t vendor_id, std::uint32_t product_code, std::uint32_t dc_assign_activate)->void {}
	auto aris_ecrt_slave_start(std::any& slave)->void {}
	auto aris_ecrt_slave_send(std::any& slave)->void {}
	auto aris_ecrt_slave_receive(std::any& slave)->void {}
	auto aris_ecrt_pdo_group_init()->std::any { return std::any(); }
	auto aris_ecrt_pdo_group_config(std::any& slave, std::any& pdo_group, std::uint16_t index, bool is_tx)->void {}
	auto aris_ecrt_pdo_init()->std::any { return std::any(); }
	auto aris_ecrt_pdo_config(std::any& slave, std::any& pdo_group, std::any& pdo, std::uint16_t index, std::uint8_t subindex, std::uint8_t bit_length)->void {}
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
	struct EcMasterHandle
	{
		ec_master_t* ec_master_;
	};
	struct EcSlaveHandle
	{
		ec_domain_t* domain_;
		std::uint8_t* domain_pd_;

		std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec_;
		std::vector<ec_pdo_info_t> ec_pdo_info_vec_tx_, ec_pdo_info_vec_rx_;
		ec_sync_info_t ec_sync_info_[5];
		ec_slave_config_t* ec_slave_config_;
	};
	struct EcPdoGroupHandle
	{
		std::vector<ec_pdo_entry_info_t> ec_pdo_entry_info_vec_;
	};
	struct EcPdoHandle
	{
		std::uint32_t offset_;
	};
	auto aris_ecrt_master_init()->std::any
	{
		std::any master = EcMasterHandle();
		if (!(std::any_cast<EcMasterHandle&>(master).ec_master_ = ecrt_request_master(0))) { throw std::runtime_error("master request failed!"); }
		return master;
	}
	auto aris_ecrt_master_config(std::any& master)->void
	{
	}
	auto aris_ecrt_master_start(std::any& master)->void
	{
		// init begin //
		ecrt_master_activate(std::any_cast<EcMasterHandle>(master).ec_master_);
	}
	auto aris_ecrt_master_stop(std::any& master)->void
	{
		ecrt_master_deactivate(std::any_cast<EcMasterHandle>(master).ec_master_);
		ecrt_release_master(std::any_cast<EcMasterHandle>(master).ec_master_);
	}
	auto aris_ecrt_master_sync(std::any& master, std::uint64_t ns)->void
	{
		ecrt_master_application_time(std::any_cast<EcMasterHandle>(master).ec_master_, ns);
		ecrt_master_sync_reference_clock(std::any_cast<EcMasterHandle>(master).ec_master_);
		ecrt_master_sync_slave_clocks(std::any_cast<EcMasterHandle>(master).ec_master_);
	}
	auto aris_ecrt_master_receive(std::any& master)->void
	{
		ecrt_master_receive(std::any_cast<EcMasterHandle>(master).ec_master_);
	}
	auto aris_ecrt_master_send(std::any& master)->void
	{
		ecrt_master_send(std::any_cast<EcMasterHandle>(master).ec_master_);
	}
	auto aris_ecrt_slave_init()->std::any
	{
		std::any slave = EcSlaveHandle();
		return slave;
	}
	auto aris_ecrt_slave_config(std::any& master, std::any& slave, std::uint16_t alias, std::uint16_t position, std::uint32_t vendor_id, std::uint32_t product_code, std::uint32_t dc_assign_activate)->void
	{
		auto &ec_mst = std::any_cast<EcMasterHandle&>(master).ec_master_;
		auto &domain = std::any_cast<EcSlaveHandle&>(slave).domain_;
		auto &domain_pd = std::any_cast<EcSlaveHandle&>(slave).domain_pd_;

		auto &ec_pdo_entry_reg_vec = std::any_cast<EcSlaveHandle&>(slave).ec_pdo_entry_reg_vec_;
		auto &ec_pdo_info_vec_tx = std::any_cast<EcSlaveHandle&>(slave).ec_pdo_info_vec_tx_;
		auto &ec_pdo_info_vec_rx = std::any_cast<EcSlaveHandle&>(slave).ec_pdo_info_vec_rx_;
		auto &ec_sync_info = std::any_cast<EcSlaveHandle&>(slave).ec_sync_info_;
		auto &ec_slave_config = std::any_cast<EcSlaveHandle&>(slave).ec_slave_config_;

		// set pdo position
		for (auto &reg : ec_pdo_entry_reg_vec)
		{
			reg.alias = alias;
			reg.position = position;
			reg.product_code = product_code;
			reg.vendor_id = vendor_id;
		}

		ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{});

		ec_sync_info[0] = ec_sync_info_t{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE };
		ec_sync_info[1] = ec_sync_info_t{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE };
		ec_sync_info[2] = ec_sync_info_t{ 2, EC_DIR_OUTPUT, static_cast<unsigned int>(ec_pdo_info_vec_rx.size()), ec_pdo_info_vec_rx.data(), EC_WD_ENABLE };
		ec_sync_info[3] = ec_sync_info_t{ 3, EC_DIR_INPUT, static_cast<unsigned int>(ec_pdo_info_vec_tx.size()), ec_pdo_info_vec_tx.data(), EC_WD_ENABLE };
		ec_sync_info[4] = ec_sync_info_t{ 0xff };

		// Create domain
		if (!(domain = ecrt_master_create_domain(ec_mst)))throw std::runtime_error("failed to create domain");

		// Get the slave configuration 
		if (!(ec_slave_config = ecrt_master_slave_config(ec_mst, alias, position, vendor_id, product_code))) { throw std::runtime_error("failed to slave config"); }

		// Configure the slave's PDOs and sync masters
		if (ecrt_slave_config_pdos(ec_slave_config, 4, ec_sync_info))throw std::runtime_error("failed to slave config pdos");

		// Configure the slave's domain
		if (ecrt_domain_reg_pdo_entry_list(domain, ec_pdo_entry_reg_vec.data()))throw std::runtime_error("failed domain_reg_pdo_entry");

		// Configure the slave's discrete clock
		ecrt_slave_config_dc(ec_slave_config, dc_assign_activate, 1000000, 4400000, 0, 0);
	}
	auto aris_ecrt_slave_start(std::any& slave)->void
	{
		auto &domain_pd = std::any_cast<EcSlaveHandle&>(slave).domain_pd_;
		if (!(domain_pd = ecrt_domain_data(std::any_cast<EcSlaveHandle&>(slave).domain_)))throw std::runtime_error("failed ecrt_domain_data");
	}
	auto aris_ecrt_slave_send(std::any& slave)->void
	{
		ecrt_domain_queue(std::any_cast<EcSlaveHandle&>(slave).domain_);
	}
	auto aris_ecrt_slave_receive(std::any& slave)->void
	{
		ecrt_domain_process(std::any_cast<EcSlaveHandle&>(slave).domain_);
	}
	auto aris_ecrt_pdo_group_init()->std::any
	{
		std::any pdo_entry = EcPdoGroupHandle();
		return pdo_entry;
	}
	auto aris_ecrt_pdo_group_config(std::any& slave, std::any& pdo_group, std::uint16_t index, bool is_tx)->void
	{
		auto &ec_pdo_info_vec_tx = std::any_cast<EcSlaveHandle&>(slave).ec_pdo_info_vec_tx_;
		auto &ec_pdo_info_vec_rx = std::any_cast<EcSlaveHandle&>(slave).ec_pdo_info_vec_rx_;

		auto &ec_pdo_entry_info_vec = std::any_cast<EcPdoGroupHandle&>(pdo_group).ec_pdo_entry_info_vec_;

		if (is_tx)
		{
			ec_pdo_info_vec_tx.push_back(ec_pdo_info_t{ index, static_cast<std::uint8_t>(ec_pdo_entry_info_vec.size()), ec_pdo_entry_info_vec.data() });
		}
		else
		{
			ec_pdo_info_vec_rx.push_back(ec_pdo_info_t{ index, static_cast<std::uint8_t>(ec_pdo_entry_info_vec.size()), ec_pdo_entry_info_vec.data() });
		}
	}
	auto aris_ecrt_pdo_init()->std::any
	{
		std::any pdo = EcPdoHandle();
		return pdo;
	}
	auto aris_ecrt_pdo_config(std::any& slave, std::any& pdo_group, std::any& pdo, std::uint16_t index, std::uint8_t subindex, std::uint8_t bit_length)->void
	{
		auto &ec_pdo_entry_reg_vec = std::any_cast<EcSlaveHandle&>(slave).ec_pdo_entry_reg_vec_;
		auto &ec_pdo_entry_info_vec = std::any_cast<EcPdoGroupHandle&>(pdo_group).ec_pdo_entry_info_vec_;

		ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{ 0, 0, 0, 0, index, subindex, &(std::any_cast<EcPdoHandle&>(pdo).offset_) });
		ec_pdo_entry_info_vec.push_back(ec_pdo_entry_info_t{ index, subindex, bit_length });
	}
	auto aris_ecrt_pdo_read(std::any& slave, std::any& pdo, void *data, int byte_size)->void
	{
		std::copy_n(std::any_cast<EcSlaveHandle&>(slave).domain_pd_ + std::any_cast<EcPdoHandle&>(pdo).offset_, byte_size, static_cast<char *>(data));
	}
	auto aris_ecrt_pdo_write(std::any& slave, std::any& pdo, const void *data, int byte_size)->void
	{
		std::copy_n(static_cast<const char *>(data), byte_size, std::any_cast<EcSlaveHandle&>(slave).domain_pd_ + std::any_cast<EcPdoHandle&>(pdo).offset_);
	}
	auto aris_ecrt_sdo_read(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::size_t *result_size, std::uint32_t *abort_code)->int
	{
		return ecrt_master_sdo_upload(std::any_cast<EcMasterHandle>(master).ec_master_, slave_position, index, subindex, to_buffer, buffer_size, result_size, abort_code);
	}
	auto aris_ecrt_sdo_write(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t buffer_size, std::uint32_t *abort_code)->int
	{
		return ecrt_master_sdo_download(std::any_cast<EcMasterHandle>(master).ec_master_, slave_position, index, subindex, to_buffer, buffer_size, abort_code);
	}
	auto aris_ecrt_sdo_config(std::any& master, std::any& slave, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *buffer, std::size_t byte_size)->void
	{
		auto &ec_slave_config = std::any_cast<EcSlaveHandle&>(slave).ec_slave_config_;
		ecrt_slave_config_sdo(ec_slave_config, index, subindex, buffer, byte_size * 8);
	}
#endif
}
