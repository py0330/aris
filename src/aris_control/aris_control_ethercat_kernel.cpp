#ifdef UNIX
#include <ecrt.h>
#endif

#include <chrono>
#include <thread>
#include <memory>
#include <vector>
#include <algorithm>

#include "aris_control_ethercat_kernel.h"

namespace aris
{
	namespace control
	{
#ifdef WIN32
		auto aris_ecrt_master_init()->Handle* { return nullptr; }
		auto aris_ecrt_master_config(Handle* master_handle)->void {}
		auto aris_ecrt_master_start(Handle* master_handle)->void {}
		auto aris_ecrt_master_stop(Handle* master_handle)->void {}
		auto aris_ecrt_master_sync(Handle* master_handle, std::uint64_t ns)->void {}
		auto aris_ecrt_master_receive(Handle* master_handle)->void {}
		auto aris_ecrt_master_send(Handle* master_handle)->void {}
		auto aris_ecrt_slave_init()->Handle* { return nullptr; }
		auto aris_ecrt_slave_config(Handle* master_handle, Handle* slave_handle, std::uint16_t alias, std::uint16_t position, std::uint32_t vendor_id, std::uint32_t product_code, std::uint32_t distribute_clock)->void {}
		auto aris_ecrt_slave_start(Handle* slave_handle)->void {}
		auto aris_ecrt_slave_send(Handle* slave_handle)->void {}
		auto aris_ecrt_slave_receive(Handle* slave_handle)->void {}
		auto aris_ecrt_pdo_group_init()->Handle* { return nullptr; }
		auto aris_ecrt_pdo_group_config(Handle* slave_handle, Handle* pdo_group_handle, std::uint16_t index, bool is_tx)->void {}
		auto aris_ecrt_pdo_init()->Handle*{ return nullptr; }
		auto aris_ecrt_pdo_config(Handle* slave_handle, Handle* pdo_group_handle, Handle* pdo_handle, std::uint16_t index, std::uint8_t subindex, std::uint8_t bit_length)->void{}
		auto aris_ecrt_pdo_read(Handle* slave_handle, Handle* pdo_handle, void *data, int byte_size)->void{	}
		auto aris_ecrt_pdo_write(Handle* slave_handle, Handle* pdo_handle, const void *data, int byte_size)->void{ }
		auto aris_ecrt_sdo_read(Handle* master_handle, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
			std::uint8_t *to_buffer, std::size_t buffer_size, std::size_t *result_size, std::uint32_t *abort_code) ->int{return 0;}
		auto aris_ecrt_sdo_write(Handle* master_handle, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
			std::uint8_t *to_buffer, std::size_t buffer_size, std::uint32_t *abort_code) ->int{	return 0;}
		auto aris_ecrt_sdo_config(Handle* master_handle, Handle* slave_handle, std::uint16_t index, std::uint8_t subindex,
			std::uint8_t *buffer, std::size_t bit_size)->void {}
#endif

#ifdef UNIX
		struct EcMasterHandle :public Handle
		{
			ec_master_t* ec_master_;
		};
		struct EcSlaveHandle :public Handle
		{
			ec_domain_t* domain_;
			std::uint8_t* domain_pd_;

			std::vector<ec_pdo_entry_reg_t> ec_pdo_entry_reg_vec_;
			std::vector<ec_pdo_info_t> ec_pdo_info_vec_tx_, ec_pdo_info_vec_rx_;
			ec_sync_info_t ec_sync_info_[5];
			ec_slave_config_t* ec_slave_config_;
		};
		struct EcPdoGroupHandle :public Handle
		{
			std::vector<ec_pdo_entry_info_t> ec_pdo_entry_info_vec_;
		};
		struct EcPdoHandle :public Handle
		{
			std::uint32_t offset_;
		};
		auto aris_ecrt_master_init()->Handle *
		{
			std::unique_ptr<Handle> handle(new EcMasterHandle);
			if (!(static_cast<EcMasterHandle*>(handle.get())->ec_master_ = ecrt_request_master(0))) { throw std::runtime_error("master request failed!"); }
			return handle.release();
		}
		auto aris_ecrt_master_config(Handle* master_handle)->void
		{
		}
		auto aris_ecrt_master_start(Handle* master_handle)->void
		{
			// init begin //
			ecrt_master_activate(static_cast<EcMasterHandle*>(master_handle)->ec_master_);
		}
		auto aris_ecrt_master_stop(Handle* master_handle)->void
		{
			ecrt_master_deactivate(static_cast<EcMasterHandle*>(master_handle)->ec_master_);
			ecrt_release_master(static_cast<EcMasterHandle*>(master_handle)->ec_master_);
		}
		auto aris_ecrt_master_sync(Handle* master_handle, std::uint64_t ns)->void
		{
			ecrt_master_application_time(static_cast<EcMasterHandle*>(master_handle)->ec_master_, ns);
			ecrt_master_sync_reference_clock(static_cast<EcMasterHandle*>(master_handle)->ec_master_);
			ecrt_master_sync_slave_clocks(static_cast<EcMasterHandle*>(master_handle)->ec_master_);
		}
		auto aris_ecrt_master_receive(Handle* master_handle)->void
		{
			ecrt_master_receive(static_cast<EcMasterHandle*>(master_handle)->ec_master_);
		}
		auto aris_ecrt_master_send(Handle* master_handle)->void
		{
			ecrt_master_send(static_cast<EcMasterHandle*>(master_handle)->ec_master_);
		}
		auto aris_ecrt_slave_init()->Handle*
		{
			std::unique_ptr<Handle> handle(new EcSlaveHandle);
			return handle.release();
		}
		auto aris_ecrt_slave_config(Handle* master_handle, Handle* slave_handle, std::uint16_t alias, std::uint16_t position, std::uint32_t vendor_id, std::uint32_t product_code, std::uint32_t distribute_clock)->void
		{
			auto &ec_mst = static_cast<EcMasterHandle*>(master_handle)->ec_master_;
			auto &domain = static_cast<EcSlaveHandle*>(slave_handle)->domain_;
			auto &domain_pd = static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_;

			auto &ec_pdo_entry_reg_vec = static_cast<EcSlaveHandle*>(slave_handle)->ec_pdo_entry_reg_vec_;
			auto &ec_pdo_info_vec_tx = static_cast<EcSlaveHandle*>(slave_handle)->ec_pdo_info_vec_tx_;
			auto &ec_pdo_info_vec_rx = static_cast<EcSlaveHandle*>(slave_handle)->ec_pdo_info_vec_rx_;
			auto &ec_sync_info = static_cast<EcSlaveHandle*>(slave_handle)->ec_sync_info_;
			auto &ec_slave_config = static_cast<EcSlaveHandle*>(slave_handle)->ec_slave_config_;

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
			if (distribute_clock)ecrt_slave_config_dc(ec_slave_config, distribute_clock, 1000000, 4400000, 0, 0);
		}
		auto aris_ecrt_slave_start(Handle* slave_handle)->void
		{
			auto &domain_pd = static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_;
			if(!(domain_pd = ecrt_domain_data(static_cast<EcSlaveHandle*>(slave_handle)->domain_)))throw std::runtime_error("failed ecrt_domain_data");
		}
		auto aris_ecrt_slave_send(Handle* slave_handle)->void
		{
			ecrt_domain_queue(static_cast<EcSlaveHandle*>(slave_handle)->domain_);
		}
		auto aris_ecrt_slave_receive(Handle* slave_handle)->void
		{
			ecrt_domain_process(static_cast<EcSlaveHandle*>(slave_handle)->domain_);
		}
		auto aris_ecrt_pdo_group_init()->Handle*
		{
			std::unique_ptr<Handle> handle(new EcPdoGroupHandle);
			return handle.release();
		}
		auto aris_ecrt_pdo_group_config(Handle* slave_handle, Handle* pdo_group_handle, std::uint16_t index, bool is_tx)->void
		{
			auto &ec_pdo_info_vec_tx = static_cast<EcSlaveHandle*>(slave_handle)->ec_pdo_info_vec_tx_;
			auto &ec_pdo_info_vec_rx = static_cast<EcSlaveHandle*>(slave_handle)->ec_pdo_info_vec_rx_;

			auto &ec_pdo_entry_info_vec = static_cast<EcPdoGroupHandle*>(pdo_group_handle)->ec_pdo_entry_info_vec_;

			if (is_tx)
			{
				ec_pdo_info_vec_tx.push_back(ec_pdo_info_t{ index, static_cast<std::uint8_t>(ec_pdo_entry_info_vec.size()), ec_pdo_entry_info_vec.data() });
			}
			else
			{
				ec_pdo_info_vec_rx.push_back(ec_pdo_info_t{ index, static_cast<std::uint8_t>(ec_pdo_entry_info_vec.size()), ec_pdo_entry_info_vec.data() });
			}
		}
		auto aris_ecrt_pdo_init()->Handle*
		{
			std::unique_ptr<Handle> handle(new EcPdoHandle);
			return handle.release();
		}
		auto aris_ecrt_pdo_config(Handle* slave_handle, Handle* pdo_group_handle, Handle* pdo_handle, std::uint16_t index, std::uint8_t subindex, std::uint8_t bit_length)->void
		{
			auto &ec_pdo_entry_reg_vec = static_cast<EcSlaveHandle*>(slave_handle)->ec_pdo_entry_reg_vec_;
			auto &ec_pdo_entry_info_vec = static_cast<EcPdoGroupHandle*>(pdo_group_handle)->ec_pdo_entry_info_vec_;

			ec_pdo_entry_reg_vec.push_back(ec_pdo_entry_reg_t{ 0, 0, 0, 0, index, subindex, &(static_cast<EcPdoHandle*>(pdo_handle)->offset_) });
			ec_pdo_entry_info_vec.push_back(ec_pdo_entry_info_t{ index, subindex, bit_length });
		}
		auto aris_ecrt_pdo_read(Handle* slave_handle, Handle* pdo_handle, void *data, int byte_size)->void
		{
			std::copy_n(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_, byte_size, static_cast<char *>(data));
		}
		auto aris_ecrt_pdo_write(Handle* slave_handle, Handle* pdo_handle, const void *data, int byte_size)->void
		{
			std::copy_n(static_cast<const char *>(data), byte_size, static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_);
		}
		auto aris_ecrt_sdo_read(Handle* master_handle, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
			std::uint8_t *to_buffer, std::size_t buffer_size, std::size_t *result_size, std::uint32_t *abort_code)->int
		{
			return ecrt_master_sdo_upload(static_cast<EcMasterHandle*>(master_handle)->ec_master_, slave_position, index, subindex, to_buffer, buffer_size, result_size, abort_code);
		}
		auto aris_ecrt_sdo_write(Handle* master_handle, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
			std::uint8_t *to_buffer, std::size_t buffer_size, std::uint32_t *abort_code)->int
		{
			return ecrt_master_sdo_download(static_cast<EcMasterHandle*>(master_handle)->ec_master_, slave_position, index, subindex, to_buffer, buffer_size, abort_code);
		}
		auto aris_ecrt_sdo_config(Handle* master_handle, Handle* slave_handle, std::uint16_t index, std::uint8_t subindex,
			std::uint8_t *buffer, std::size_t byte_size)->void
		{
			auto &ec_slave_config = static_cast<EcSlaveHandle*>(slave_handle)->ec_slave_config_;
			ecrt_slave_config_sdo(ec_slave_config, index, subindex, buffer, byte_size * 8);
		}
#endif
	}
}
