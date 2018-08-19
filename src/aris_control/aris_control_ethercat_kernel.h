#ifndef ARIS_CONTROL_ETHERCAT_KERNEL_H
#define ARIS_CONTROL_ETHERCAT_KERNEL_H


#include <cstddef>
#include <cstdint>

#include <aris_control_rt_timer.h>

namespace aris::control
{
	//------------------------ Ecrt 的配置流程 ------------------------//
	// 1. init master
	// 2. init slave
	// 3. init pdo_group
	// 4. init pdo
	// 5. config pdo
	// 6. config pdo_group
	// 7. config slave
	// 8. config master
	// 9. config sdo
	// 10.lock memory
	// 11.start master
	// 12.start slave

	//------------------------ Ecrt 的通讯流程 ------------------------//
	// 1. master receive
	// 2. slave receive
	// 3. pdo read update
	// 4. control strategy
	// 5. pdo write update
	// 6. master sync
	// 7. slave send
	// 8. master send

	auto aris_ecrt_master_init()->Handle*;
	auto aris_ecrt_master_config(Handle* master_handle)->void;
	auto aris_ecrt_master_start(Handle* master_handle)->void;
	auto aris_ecrt_master_stop(Handle* master_handle)->void;
	auto aris_ecrt_master_sync(Handle* master_handle, std::uint64_t ns)->void;
	auto aris_ecrt_master_receive(Handle* master_handle)->void;
	auto aris_ecrt_master_send(Handle* master_handle)->void;
	auto aris_ecrt_slave_init()->Handle*;
	auto aris_ecrt_slave_config(Handle* master_handle, Handle* slave_handle, std::uint16_t alias, std::uint16_t position, std::uint32_t vendor_id, std::uint32_t product_code, std::uint32_t distribute_clock)->void;
	auto aris_ecrt_slave_start(Handle* slave_handle)->void;
	auto aris_ecrt_slave_send(Handle* slave_handle)->void;
	auto aris_ecrt_slave_receive(Handle* slave_handle)->void;
	auto aris_ecrt_pdo_group_init()->Handle*;
	auto aris_ecrt_pdo_group_config(Handle* slave_handle, Handle* pdo_group_handle, std::uint16_t index, bool is_tx)->void;
	auto aris_ecrt_pdo_init()->Handle*;
	auto aris_ecrt_pdo_config(Handle* slave_handle, Handle* pdo_group_handle, Handle* pdo_handle, std::uint16_t index, std::uint8_t subindex, std::uint8_t bit_length)->void;
	auto aris_ecrt_pdo_read(Handle* slave_handle, Handle* pdo_handle, void *data, int byte_size)->void;
	auto aris_ecrt_pdo_write(Handle* slave_handle, Handle* pdo_handle, const void *data, int byte_size)->void;
	auto aris_ecrt_sdo_read(Handle* master_handle, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t bit_size, std::size_t *result_size, std::uint32_t *abort_code)->int;
	auto aris_ecrt_sdo_write(Handle* master_handle, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t bit_size, std::uint32_t *abort_code) ->int;
	auto aris_ecrt_sdo_config(Handle* master_handle, Handle* slave_handle, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *buffer, std::size_t bit_size)->void;
}

#endif
