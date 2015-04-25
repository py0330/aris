#ifndef ETHERCATCOUPLER_H
#define ETHERCATCOUPLER_H

#include "ecrt.h"
#include <stdio.h>

namespace ElmoMotor{

// Backhoff EtherCat coupler EK1100
class CEthercatCouplerEK1100
{
public:
    CEthercatCouplerEK1100(void);
    ~CEthercatCouplerEK1100(void);

    // assign the device's position in a Ethercat net according to the network's topology, register domain, pdos and sync masters for this driver
    int SetEtherCATPosition(uint16_t position,
        uint32_t product_code =  0x14140626,
        uint32_t vender_id = 0x00000002,
        uint16_t alias = 0);

    // function to initialize the device, mush be called after ecrt_request_master(0) which is defined in the EthercatMaster object
    int Initialize(ec_master_t **p_master);

    // Activate the device
    int Activate();

private:
    ec_master_t* m_pMaster;
    
    //slave
    uint16_t m_alias; //= 0;
    uint16_t m_position;
    uint32_t m_vender_id; //= 0x00000002;
    uint32_t m_product_code; //= 0x14140626;

    ec_slave_config_t* m_slaveConfigEK1100;		
};

}

#endif // ETHERCATCOUPLER_H


