#ifdef WIN32
#include <ecrt_windows_py.h>//just for IDE vs2015, it does not really work
#endif
#ifdef UNIX
#include <ecrt.h>
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>
#include <sys/mman.h>
// following for pipe
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <rtdm/rtdm.h>
#include <rtdm/rtipc.h>
#endif

#include <cstdarg>
#include <memory>
#include <vector>

#include "aris_control_kernel.h"

namespace aris
{
	namespace control
	{
#ifdef WIN32
        auto aris_rt_printf(const char * format, ...)->void
		{
			va_list args;
			va_start(args, format);
			vprintf(format, args);
			va_end(args);
		}

		auto aris_rt_set_periodic(int nanoseconds)->void
		{
		};
		auto aris_rt_wait_period()->void
		{
		};
		auto aris_rt_timer_read()->std::int64_t
		{
			return 0;
		}
		auto aris_rt_task_start(void(*task_func)(void*), void*param)->Handle*
		{
			std::unique_ptr<Handle> handle;
			return handle.release();
		}
		auto aris_rt_task_stop(Handle* handle)->void
		{

		}
#endif

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
		auto aris_ecrt_pdo_read_uint8(Handle* slave_handle, Handle* pdo_handle)->std::uint8_t { return 0; }
		auto aris_ecrt_pdo_read_uint16(Handle* slave_handle, Handle* pdo_handle)->std::uint16_t { return 0; }
		auto aris_ecrt_pdo_read_uint32(Handle* slave_handle, Handle* pdo_handle)->std::uint32_t { return 0; }
		auto aris_ecrt_pdo_read_int8(Handle* slave_handle, Handle* pdo_handle)->std::int8_t { return 0; }
		auto aris_ecrt_pdo_read_int16(Handle* slave_handle, Handle* pdo_handle)->std::int16_t { return 0; }
		auto aris_ecrt_pdo_read_int32(Handle* slave_handle, Handle* pdo_handle)->std::int32_t { return 0; }
		auto aris_ecrt_pdo_write_uint8(Handle* slave_handle, Handle* pdo_handle, std::uint8_t value)->void {}
		auto aris_ecrt_pdo_write_uint16(Handle* slave_handle, Handle* pdo_handle, std::uint16_t value)->void {}
		auto aris_ecrt_pdo_write_uint32(Handle* slave_handle, Handle* pdo_handle, std::uint32_t value)->void {}
		auto aris_ecrt_pdo_write_int8(Handle* slave_handle, Handle* pdo_handle, std::int8_t value)->void {}
		auto aris_ecrt_pdo_write_int16(Handle* slave_handle, Handle* pdo_handle, std::int16_t value)->void {}
		auto aris_ecrt_pdo_write_int32(Handle* slave_handle, Handle* pdo_handle, std::int32_t value)->void {}
		auto aris_ecrt_sdo_read(Handle* master_handle, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
			std::uint8_t *to_buffer, std::size_t buffer_size, std::size_t *result_size, std::uint32_t *abort_code) ->int{return 0;}
		auto aris_ecrt_sdo_write(Handle* master_handle, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
			std::uint8_t *to_buffer, std::size_t buffer_size, std::uint32_t *abort_code) ->int{	return 0;}
		auto aris_ecrt_sdo_config(Handle* master_handle, Handle* slave_handle, std::uint16_t index, std::uint8_t subindex,
			char *buffer, std::size_t bit_size)->void {}
#endif

#ifdef UNIX
		struct RtTaskHandle :public Handle { RT_TASK task; };

        auto aris_rt_printf(const char * format, ...)->void
		{
			va_list args;
			va_start(args, format);
			rt_vprintf(format, args);
			va_end(args);
		}
		auto aris_rt_set_periodic(int nanoseconds)->void
		{
			rt_task_set_periodic(NULL, TM_NOW, nanoseconds);
		};
		auto aris_rt_wait_period()->void
		{
			rt_task_wait_period(NULL);
		};
		auto aris_rt_timer_read()->std::int64_t
		{
			return rt_timer_read();
		}
		auto aris_rt_task_start(void(*task_func)(void*), void*param)->Handle*
		{
			std::unique_ptr<Handle> handle(new RtTaskHandle);
			rt_print_auto_init(1);
			rt_task_create(&static_cast<RtTaskHandle*>(handle.get())->task, "realtime core", 0, 99, T_FPU);
			rt_task_start(&static_cast<RtTaskHandle*>(handle.get())->task, task_func, param);
			return handle.release();
		}
		auto aris_rt_task_stop(Handle* handle)->void
		{
			rt_task_delete(&static_cast<RtTaskHandle*>(handle)->task);
		}
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
			if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) { throw std::runtime_error("lock failed"); }
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
			static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ = ecrt_domain_data(static_cast<EcSlaveHandle*>(slave_handle)->domain_);
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
		auto aris_ecrt_pdo_read_uint8(Handle* slave_handle, Handle* pdo_handle)->std::uint8_t
		{
			return *reinterpret_cast<const std::uint8_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_);
		}
		auto aris_ecrt_pdo_read_uint16(Handle* slave_handle, Handle* pdo_handle)->std::uint16_t
		{
			return *reinterpret_cast<const std::uint16_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_);
		}
		auto aris_ecrt_pdo_read_uint32(Handle* slave_handle, Handle* pdo_handle)->std::uint32_t
		{
			return *reinterpret_cast<const std::uint32_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_);
		}
		auto aris_ecrt_pdo_read_int8(Handle* slave_handle, Handle* pdo_handle)->std::int8_t
		{
			return *reinterpret_cast<const std::int8_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_);
		}
		auto aris_ecrt_pdo_read_int16(Handle* slave_handle, Handle* pdo_handle)->std::int16_t
		{
			return *reinterpret_cast<const std::int16_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_);
		}
		auto aris_ecrt_pdo_read_int32(Handle* slave_handle, Handle* pdo_handle)->std::int32_t
		{
			return *reinterpret_cast<const std::int32_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_);
		}
		auto aris_ecrt_pdo_write_uint8(Handle* slave_handle, Handle* pdo_handle, std::uint8_t value)->void
		{
			*reinterpret_cast<std::uint8_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_) = value;
		}
		auto aris_ecrt_pdo_write_uint16(Handle* slave_handle, Handle* pdo_handle, std::uint16_t value)->void
		{
			*reinterpret_cast<std::uint16_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_) = value;
		}
		auto aris_ecrt_pdo_write_uint32(Handle* slave_handle, Handle* pdo_handle, std::uint32_t value)->void
		{
			*reinterpret_cast<std::uint32_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_) = value;
		}
		auto aris_ecrt_pdo_write_int8(Handle* slave_handle, Handle* pdo_handle, std::int8_t value)->void
		{
			*reinterpret_cast<std::int8_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_) = value;
		};
		auto aris_ecrt_pdo_write_int16(Handle* slave_handle, Handle* pdo_handle, std::int16_t value)->void
		{
			*reinterpret_cast<std::int16_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_) = value;
		}
		auto aris_ecrt_pdo_write_int32(Handle* slave_handle, Handle* pdo_handle, std::int32_t value)->void
		{
			*reinterpret_cast<std::int32_t*>(static_cast<EcSlaveHandle*>(slave_handle)->domain_pd_ + static_cast<EcPdoHandle*>(pdo_handle)->offset_) = value;
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
			char *buffer, std::size_t bit_size)->void
		{
			auto &ec_slave_config = static_cast<EcSlaveHandle*>(slave_handle)->ec_slave_config_;
			
			switch (bit_size)
			{
			case 8:		ecrt_slave_config_sdo8(ec_slave_config, index, subindex, *reinterpret_cast<std::uint8_t*>(buffer)); break;
			case 16:	ecrt_slave_config_sdo16(ec_slave_config, index, subindex, *reinterpret_cast<std::uint16_t*>(buffer)); break;
			case 32:	ecrt_slave_config_sdo32(ec_slave_config, index, subindex, *reinterpret_cast<std::uint32_t*>(buffer)); break;
			}
		}
#endif
	}
}
