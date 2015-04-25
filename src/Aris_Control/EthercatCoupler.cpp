#include "EthercatCoupler.h"

namespace ElmoMotor{

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
