#include "AnalogOutputs.h"

namespace ElmoMotor
{

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
    printf("2\n");

    m_pDomainAll = ecrt_master_create_domain(m_pMaster);
    if (!m_pDomainAll)
    {
        printf("create domain failed!\n");
        return -3;
    }
    printf("3\n");
    printf("v:%x:%x\n",(int)m_vender_id,m_product_code);

    // Get the slave configuration 
    if (!(m_slaveConfigEL4002 = ecrt_master_slave_config(
        m_pMaster, m_alias,m_position,m_vender_id,m_product_code)))
    {
        printf("Failed to get slave 0 configuration.\n");
        return -4;
    }
    printf("4\n");

    // Configure the slave's PDOs and sync masters
    if (ecrt_slave_config_pdos(m_slaveConfigEL4002, 4, m_slaveSyncs))
    {
        // handle error
        printf("Error slave config PDOs!\n");
        return -5;
    }
    printf("5\n");

    // Configure the slave's domain
    if (ecrt_domain_reg_pdo_entry_list(m_pDomainAll, m_domainAllRegs))
    {
        printf("PDO entry registration failed!\n");
        return -6;
    }
    printf("6\n");
    printf("OK!\n");

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

}
