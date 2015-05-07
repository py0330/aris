/*
 * Aris_Sensor.cpp
 *
 *  Created on: Apr 22, 2015
 *      Author: hex
 */
#include "Aris_Sensor.h"

namespace ElmoMotor{

CAnalogInputsEL3104::CAnalogInputsEL3104()
{
}

CAnalogInputsEL3104::~CAnalogInputsEL3104()
{
}

int CAnalogInputsEL3104::SetEtherCATPosition(uint16_t position,uint32_t product_code,uint32_t vender_id, uint16_t alias)
{
    // Set the device's position in ethercat network
    m_position=position;
    m_product_code=product_code;
    m_vender_id=vender_id;
    m_alias=alias;

    // Register ethercat domain for this device
    m_domainAllRegs[0]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6000, 0x11, &m_offsetValueChannel1};
    m_domainAllRegs[1]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6010, 0x11, &m_offsetValueChannel2};
    m_domainAllRegs[2]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6020, 0x11, &m_offsetValueChannel3};
    m_domainAllRegs[3]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x6030, 0x11, &m_offsetValueChannel4};

    // Register the PDO entries for this device
    m_slavePdoEntries[ 0] = (ec_pdo_entry_info_t){0x6000, 0x01, 1}; /* Under range */
    m_slavePdoEntries[ 1] = (ec_pdo_entry_info_t){0x6000, 0x02, 1}; /* Over range */
    m_slavePdoEntries[ 2] = (ec_pdo_entry_info_t){0x6000, 0x03, 2}; /* Limit 1 */
    m_slavePdoEntries[ 3] = (ec_pdo_entry_info_t){0x6000, 0x05, 2}; /* Limit 2 */
    m_slavePdoEntries[ 4] = (ec_pdo_entry_info_t){0x6000, 0x07, 1}; /* Error */
    m_slavePdoEntries[ 5] = (ec_pdo_entry_info_t){0x0000, 0x00, 1}; /* Gap */
    m_slavePdoEntries[ 6] = (ec_pdo_entry_info_t){0x0000, 0x00, 5}; /* Gap */
    m_slavePdoEntries[ 7] = (ec_pdo_entry_info_t){0x1c32, 0x20, 1}; /* Sync error */
    m_slavePdoEntries[ 8] = (ec_pdo_entry_info_t){0x1800, 0x07, 1}; /* TxPDO State */
    m_slavePdoEntries[ 9] = (ec_pdo_entry_info_t){0x1800, 0x09, 1}; /* TxPDO Toggle */
    m_slavePdoEntries[10] = (ec_pdo_entry_info_t){0x6000, 0x11, 16}; /* Value */
    m_slavePdoEntries[11] = (ec_pdo_entry_info_t){0x6010, 0x01, 1}; /* Under range */
    m_slavePdoEntries[12] = (ec_pdo_entry_info_t){0x6010, 0x02, 1}; /* Over range */
    m_slavePdoEntries[13] = (ec_pdo_entry_info_t){0x6010, 0x03, 2}; /* Limit 1 */
    m_slavePdoEntries[14] = (ec_pdo_entry_info_t){0x6010, 0x05, 2}; /* Limit 2 */
    m_slavePdoEntries[15] = (ec_pdo_entry_info_t){0x6010, 0x07, 1}; /* Error */
    m_slavePdoEntries[16] = (ec_pdo_entry_info_t){0x0000, 0x00, 1}; /* Gap */
    m_slavePdoEntries[17] = (ec_pdo_entry_info_t){0x0000, 0x00, 5}; /* Gap */
    m_slavePdoEntries[18] = (ec_pdo_entry_info_t){0x1c32, 0x20, 1}; /* Sync error */
    m_slavePdoEntries[19] = (ec_pdo_entry_info_t){0x1802, 0x07, 1}; /* TxPDO State */
    m_slavePdoEntries[20] = (ec_pdo_entry_info_t){0x1802, 0x09, 1}; /* TxPDO Toggle */
    m_slavePdoEntries[21] = (ec_pdo_entry_info_t){0x6010, 0x11, 16}; /* Value */
    m_slavePdoEntries[22] = (ec_pdo_entry_info_t){0x6020, 0x01, 1}; /* Under range */
    m_slavePdoEntries[23] = (ec_pdo_entry_info_t){0x6020, 0x02, 1}; /* Over range */
    m_slavePdoEntries[24] = (ec_pdo_entry_info_t){0x6020, 0x03, 2}; /* Limit 1 */
    m_slavePdoEntries[25] = (ec_pdo_entry_info_t){0x6020, 0x05, 2}; /* Limit 2 */
    m_slavePdoEntries[26] = (ec_pdo_entry_info_t){0x6020, 0x07, 1}; /* Error */
    m_slavePdoEntries[27] = (ec_pdo_entry_info_t){0x0000, 0x00, 1}; /* Gap */
    m_slavePdoEntries[28] = (ec_pdo_entry_info_t){0x0000, 0x00, 5}; /* Gap */
    m_slavePdoEntries[29] = (ec_pdo_entry_info_t){0x1c32, 0x20, 1}; /* Sync error */
    m_slavePdoEntries[30] = (ec_pdo_entry_info_t){0x1804, 0x07, 1}; /* TxPDO State */
    m_slavePdoEntries[31] = (ec_pdo_entry_info_t){0x1804, 0x09, 1}; /* TxPDO Toggle */
    m_slavePdoEntries[32] = (ec_pdo_entry_info_t){0x6020, 0x11, 16}; /* Value */
    m_slavePdoEntries[33] = (ec_pdo_entry_info_t){0x6030, 0x01, 1}; /* Under range */
    m_slavePdoEntries[34] = (ec_pdo_entry_info_t){0x6030, 0x02, 1}; /* Over range */
    m_slavePdoEntries[35] = (ec_pdo_entry_info_t){0x6030, 0x03, 2}; /* Limit 1 */
    m_slavePdoEntries[36] = (ec_pdo_entry_info_t){0x6030, 0x05, 2}; /* Limit 2 */
    m_slavePdoEntries[37] = (ec_pdo_entry_info_t){0x6030, 0x07, 1}; /* Error */
    m_slavePdoEntries[38] = (ec_pdo_entry_info_t){0x0000, 0x00, 1}; /* Gap */
    m_slavePdoEntries[39] = (ec_pdo_entry_info_t){0x0000, 0x00, 5}; /* Gap */
    m_slavePdoEntries[40] = (ec_pdo_entry_info_t){0x1c32, 0x20, 1}; /* Sync error */
    m_slavePdoEntries[41] = (ec_pdo_entry_info_t){0x1806, 0x07, 1}; /* TxPDO State */
    m_slavePdoEntries[42] = (ec_pdo_entry_info_t){0x1806, 0x09, 1}; /* TxPDO Toggle */
    m_slavePdoEntries[43] = (ec_pdo_entry_info_t){0x6030, 0x11, 16}; /* Value */

    // Register PDOs for the PDO entries just registered
    m_slaveTxPdos[0] = (ec_pdo_info_t){0x1A00, 11, m_slavePdoEntries +  0};
    m_slaveTxPdos[1] = (ec_pdo_info_t){0x1A02, 11, m_slavePdoEntries + 11};
    m_slaveTxPdos[2] = (ec_pdo_info_t){0x1A04, 11, m_slavePdoEntries + 22};
    m_slaveTxPdos[3] = (ec_pdo_info_t){0x1A06, 11, m_slavePdoEntries + 33};

    // Assign sync masters for the PDOs just registered
    m_slaveSyncs[0] = (ec_sync_info_t){0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[1] = (ec_sync_info_t){1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[2] = (ec_sync_info_t){2, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[3] = (ec_sync_info_t){3, EC_DIR_INPUT,  4, m_slaveTxPdos + 0, EC_WD_DISABLE};
    m_slaveSyncs[4] = (ec_sync_info_t){0xff};

    return 0;
}

int CAnalogInputsEL3104::Initialize(ec_master_t **p_master)
{
    m_pMaster=*p_master;
   // printf("2\n");

    m_pDomainAll = ecrt_master_create_domain(m_pMaster);
    if (!m_pDomainAll)
    {
        printf("create domain failed!\n");
        return -3;
    }
   // printf("3\n");
   // printf("v:%x:%x\n",(int)m_vender_id,m_product_code);
   // printf("pos: %d: %d\n", m_alias, m_position);

    // Get the slave configuration
    if (!(m_slaveConfigEL3104 = ecrt_master_slave_config(
        m_pMaster, m_alias,m_position,m_vender_id,m_product_code)))
    {
        printf("Failed to get slave 0 configuration.\n");
        return -4;
    }
   // printf("4\n");

    // Configure the slave's PDOs and sync masters
    if (ecrt_slave_config_pdos(m_slaveConfigEL3104, 4, m_slaveSyncs))
    {
        // handle error
        printf("Error slave config PDOs!\n");
        return -5;
    }
   // printf("5\n");

    // Configure the slave's domain
    if (ecrt_domain_reg_pdo_entry_list(m_pDomainAll, m_domainAllRegs))
    {
        printf("PDO entry registration failed!\n");
        return -6;
    }
    //printf("6\n");
    //printf("OK!\n");

    // Set the initial base zero offset value
    for ( int i = 0; i < 4; i++)
    {
        m_baseValue[i] = 0;
        m_summedValue[i] = 0;
    }

    return 0;
}

int CAnalogInputsEL3104::Activate()
{
    m_pDomainAllPd = ecrt_domain_data(m_pDomainAll);
    if (!m_pDomainAllPd)
    {
        return -10;
    }
    return 0;
}

int CAnalogInputsEL3104::ReadData( CAnalogInputsData *p_data )
{
    p_data->values[0]      = EC_READ_S16(m_pDomainAllPd + m_offsetValueChannel1);
    p_data->values[1]      = EC_READ_S16(m_pDomainAllPd + m_offsetValueChannel2);
    p_data->values[2]      = EC_READ_S16(m_pDomainAllPd + m_offsetValueChannel3);
    p_data->values[3]      = EC_READ_S16(m_pDomainAllPd + m_offsetValueChannel4);

    // Zeroing procedure is ongoing
    if (m_zeroingCountLast > 0)
    {
        // sum up the readout data
        for (int i = 0; i < 4; i++)
        {
            m_summedValue[i] += p_data->values[i];
        }
        m_zeroingCountLast--;

        // it's time to end zeroing procedure, calculate the average and update baseValue
        if (m_zeroingCountLast <= 0)
        {
            for (int i = 0; i < 4; i++)
            {
                m_baseValue[i] = m_summedValue[i] / ZEROING_DATA_COUNT;
            }
        }
    }
    return 0;
}

void CAnalogInputsEL3104::Upload()
{
    ecrt_domain_process(m_pDomainAll);
    ReadData(&m_InputsData);
}

void CAnalogInputsEL3104::Download()
{
    ecrt_domain_queue(m_pDomainAll);
}

int CAnalogInputsEL3104::GetAnalogInputs( CAnalogInputsData* p_data ) const
{
    for (int i = 0; i < 4; i++)
    {
        p_data->values[i] = m_InputsData.values[i] - m_baseValue[i];
    }
    return 0;
}

void CAnalogInputsEL3104::RequestZeroing()
{
    m_zeroingCountLast = ZEROING_DATA_COUNT;
    for (int i = 0; i < 4; i++)
    {
        m_summedValue[i] = 0;
    }
}



///////////////////////////////////////////////////////////////////////////////////////////

CAnalogOutputsEL4002::CAnalogOutputsEL4002(void)
{
}


CAnalogOutputsEL4002::~CAnalogOutputsEL4002(void)
{
}

int ElmoMotor::CAnalogOutputsEL4002::SetEtherCATPosition( uint16_t position, uint32_t product_code, uint32_t vender_id, uint16_t alias)
{
    // Set the device's position in ethercat network
    m_position=position;
    m_product_code=product_code;
    m_vender_id=vender_id;
    m_alias=alias;

    // Register ethercat domain for this device
    m_domainAllRegs[0]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x7000, 0x01, &m_offsetValueChannel1};
    m_domainAllRegs[1]=(ec_pdo_entry_reg_t){m_alias, m_position, m_vender_id, m_product_code, 0x7010, 0x01, &m_offsetValueChannel2};

    // Register the PDO entries for this device
    m_slavePdoEntries[0] = (ec_pdo_entry_info_t){0x7000, 0x01, 16}; /* Analog output ch.1 */
    m_slavePdoEntries[1] = (ec_pdo_entry_info_t){0x7010, 0x01, 16}; /* Analog output ch.2 */

    // Register PDOs for the PDO entries just registered
    m_slaveRxPdos[0] = (ec_pdo_info_t){0x1600, 1, m_slavePdoEntries +  0};
    m_slaveRxPdos[1] = (ec_pdo_info_t){0x1601, 1, m_slavePdoEntries +  1};

    // Assign sync masters for the PDOs just registered
    m_slaveSyncs[0] = (ec_sync_info_t){0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[1] = (ec_sync_info_t){1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[2] = (ec_sync_info_t){2, EC_DIR_OUTPUT, 2, m_slaveRxPdos + 0, EC_WD_DISABLE};
    m_slaveSyncs[3] = (ec_sync_info_t){3, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE};
    m_slaveSyncs[4] = (ec_sync_info_t){0xff};

    return 0;
}

int ElmoMotor::CAnalogOutputsEL4002::Initialize( ec_master_t **p_master )
{
    m_pMaster=*p_master;
    //printf("2\n");

    m_pDomainAll = ecrt_master_create_domain(m_pMaster);
    if (!m_pDomainAll)
    {
        printf("create domain failed!\n");
        return -3;
    }
    //printf("3\n");
    //printf("v:%x:%x\n",(int)m_vender_id,m_product_code);

    // Get the slave configuration
    if (!(m_slaveConfigEL4002 = ecrt_master_slave_config(
        m_pMaster, m_alias,m_position,m_vender_id,m_product_code)))
    {
        printf("Failed to get slave 0 configuration.\n");
        return -4;
    }
   //printf("4\n");

    // Configure the slave's PDOs and sync masters
    if (ecrt_slave_config_pdos(m_slaveConfigEL4002, 4, m_slaveSyncs))
    {
        // handle error
        printf("Error slave config PDOs!\n");
        return -5;
    }
   // printf("5\n");

    // Configure the slave's domain
    if (ecrt_domain_reg_pdo_entry_list(m_pDomainAll, m_domainAllRegs))
    {
        printf("PDO entry registration failed!\n");
        return -6;
    }
  //  printf("6\n");
   // printf("OK!\n");

    return 0;
}

int CAnalogOutputsEL4002::Activate()
{
    m_pDomainAllPd = ecrt_domain_data(m_pDomainAll);
    if (!m_pDomainAllPd)
    {
        return -10;
    }
    return 0;
}

void CAnalogOutputsEL4002::Upload()
{
    ecrt_domain_process(m_pDomainAll);
}

void CAnalogOutputsEL4002::Download()
{
    WriteData(m_OutputsData);
    ecrt_domain_queue(m_pDomainAll);
}

int CAnalogOutputsEL4002::WriteData( const CAnalogOutputsData& data )
{
    EC_WRITE_S16(m_pDomainAllPd + m_offsetValueChannel1, data.values[0]);
    EC_WRITE_S16(m_pDomainAllPd + m_offsetValueChannel2, data.values[1]);
    return 0;
}

int CAnalogOutputsEL4002::SetAnalogOutputs(const CAnalogOutputsData& data)
{
    m_OutputsData.values[0] = data.values[0];
    m_OutputsData.values[1] = data.values[1];

    return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

CEthercatCouplerEK1100::CEthercatCouplerEK1100(void)
{
}


CEthercatCouplerEK1100::~CEthercatCouplerEK1100(void)
{
}

int ElmoMotor::CEthercatCouplerEK1100::SetEtherCATPosition( uint16_t position, uint32_t product_code, uint32_t vender_id, uint16_t alias)
{
    // Set the driver's position in ethercat network
    m_position=position;
    m_product_code=product_code;
    m_vender_id=vender_id;
    m_alias=alias;
    return 0;
}

int ElmoMotor::CEthercatCouplerEK1100::Initialize( ec_master_t **p_master )
{
    m_pMaster=*p_master;
    if (!(m_slaveConfigEK1100 = ecrt_master_slave_config(
        m_pMaster, m_alias,m_position,m_vender_id,m_product_code)))
    {
        printf("Failed to get slave 0 configuration.\n");
        return -4;
    }
    return 0;
}

int ElmoMotor::CEthercatCouplerEK1100::Activate()
{
    return 0;
}


}




