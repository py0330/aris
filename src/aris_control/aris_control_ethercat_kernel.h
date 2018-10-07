#ifndef ARIS_CONTROL_ETHERCAT_KERNEL_H
#define ARIS_CONTROL_ETHERCAT_KERNEL_H


#include <cstddef>
#include <cstdint>

#include <aris_control_rt_timer.h>
#include <any>

namespace aris::control
{
	//------------------------ Ecrt 的配置流程 ------------------------//
	// 1. init master
	// 2. init slave
	// 3. init pdo
	// 4. init pdo_entry
	// 5. config pdo_entry
	// 6. config pdo
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



	// pdo可以分为4级 //
	// 1. Sm(sync manager), 每个sm可以同步多个pdo entry
	// 2. pdo entry, 每个pdo entry 包含多个pdo
	// 3. pdo index, 每个pdo index下包含多个subindex
	// 4. pdo subindex, 每个pdo index 和 subindex共同确定一个pdo

	//int ecrt_master(
	//	ec_master_t *master, /**< EtherCAT master */
	//	ec_master_info_t *master_info /**< Structure that will output the
	//								  information */
	//);
	//int ecrt_master_get_slave(
	//	ec_master_t *master, /**< EtherCAT master */
	//	uint16_t slave_position, /**< Slave position. */
	//	ec_slave_info_t *slave_info /**< Structure that will output the
	//								information */
	//);
	//int ecrt_master_get_sync_manager(
	//	ec_master_t *master, /**< EtherCAT master. */
	//	uint16_t slave_position, /**< Slave position. */
	//	uint8_t sync_index, /**< Sync manager index. Must be less
	//						than #EC_MAX_SYNC_MANAGERS. */
	//	ec_sync_info_t *sync /**< Pointer to output structure. */
	//);
	//int ecrt_master_get_pdo(
	//	ec_master_t *master, /**< EtherCAT master. */
	//	uint16_t slave_position, /**< Slave position. */
	//	uint8_t sync_index, /**< Sync manager index. Must be less
	//						than #EC_MAX_SYNC_MANAGERS. */
	//	uint16_t pos, /**< Zero-based PDO position. */
	//	ec_pdo_info_t *pdo /**< Pointer to output structure. */
	//);
	//int ecrt_master_get_pdo_entry(
	//	ec_master_t *master, /**< EtherCAT master. */
	//	uint16_t slave_position, /**< Slave position. */
	//	uint8_t sync_index, /**< Sync manager index. Must be less
	//						than #EC_MAX_SYNC_MANAGERS. */
	//	uint16_t pdo_pos, /**< Zero-based PDO position. */
	//	uint16_t entry_pos, /**< Zero-based PDO entry position. */
	//	ec_pdo_entry_info_t *entry /**< Pointer to output structure. */
	//);

	class EthercatMaster;

	auto aris_ecrt_scan(EthercatMaster *master)->int;



	auto aris_ecrt_master_init()->std::any;
	auto aris_ecrt_master_config(std::any& master)->void;
	auto aris_ecrt_master_start(std::any& master)->void;
	auto aris_ecrt_master_stop(std::any& master)->void;
	auto aris_ecrt_master_sync(std::any& master, std::uint64_t ns)->void;
	auto aris_ecrt_master_receive(std::any& master)->void;
	auto aris_ecrt_master_send(std::any& master)->void;
	auto aris_ecrt_slave_init()->std::any;
	auto aris_ecrt_slave_config(std::any& master, std::any& slave, std::uint16_t alias, std::uint16_t position, std::uint32_t vendor_id, std::uint32_t product_code, std::uint32_t distribute_clock)->void;
	auto aris_ecrt_slave_start(std::any& slave)->void;
	auto aris_ecrt_slave_send(std::any& slave)->void;
	auto aris_ecrt_slave_receive(std::any& slave)->void;
	auto aris_ecrt_pdo_group_init()->std::any;
	auto aris_ecrt_pdo_group_config(std::any& slave, std::any& pdo_group, std::uint16_t index, bool is_tx)->void;
	auto aris_ecrt_pdo_init()->std::any;
	auto aris_ecrt_pdo_config(std::any& slave, std::any& pdo_group, std::any& pdo, std::uint16_t index, std::uint8_t subindex, std::uint8_t bit_length)->void;
	auto aris_ecrt_pdo_read(std::any& slave, std::any& pdo, void *data, int byte_size)->void;
	auto aris_ecrt_pdo_write(std::any& slave, std::any& pdo, const void *data, int byte_size)->void;
	auto aris_ecrt_sdo_read(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t bit_size, std::size_t *result_size, std::uint32_t *abort_code)->int;
	auto aris_ecrt_sdo_write(std::any& master, std::uint16_t slave_position, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *to_buffer, std::size_t bit_size, std::uint32_t *abort_code) ->int;
	auto aris_ecrt_sdo_config(std::any& master, std::any& slave, std::uint16_t index, std::uint8_t subindex,
		std::uint8_t *buffer, std::size_t bit_size)->void;
}

#endif
