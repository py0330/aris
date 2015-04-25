#ifndef ANALOGOUTPUTS_H
#define ANALOGOUTPUTS_H

#include "ecrt.h"
#include <sys/mman.h>
#include <stdio.h>
#include <cstring>
#include "AnalogIOData.h"

namespace ElmoMotor{

// Two-channel analog outputs card Beckhoff EL4002 via EtherCat
class CAnalogOutputsEL4002
{
public:
    CAnalogOutputsEL4002(void);
    ~CAnalogOutputsEL4002(void);

    // assign the device's position in a Ethercat net according to the network's topology, register domain, pdos and sync masters for this driver
    int SetEtherCATPosition(uint16_t position,
        uint32_t product_code =  0x0fa23052,
        uint32_t vender_id = 0x00000002,
        uint16_t alias = 0);

    // function to initialize the device, mush be called after ecrt_request_master(0) which is defined in the EthercatMaster object
    int Initialize(ec_master_t **p_master);

    // Activate the slave's cyclic task, return value 0 means successfully activated, otherwise failed
    int Activate();

    // Set analog outputs to the device
    int SetAnalogOutputs(const CAnalogOutputsData& data);

    void Upload();//call at the beginning of a cycle, retrieve data from the card
    void Download(); // call at the end of a cycle, download data to the card

private:
    CAnalogOutputsData m_OutputsData;
    int WriteData(const CAnalogOutputsData& data);//cyclic tasks

    ec_master_t* m_pMaster;
    //configuration for multi-domain
    ec_domain_t* m_pDomainAll;// = NULL;
    uint8_t* m_pDomainAllPd;

    //slave
    uint16_t m_alias; //= 0;
    uint16_t m_position;
    uint32_t m_vender_id; //= 0x00000002;
    uint32_t m_product_code; //= 0x0fa23052;

    ec_slave_config_t* m_slaveConfigEL4002;
    ec_pdo_entry_reg_t m_domainAllRegs[3];
    ec_pdo_entry_info_t m_slavePdoEntries[2];
    ec_pdo_info_t m_slaveRxPdos[2];
    ec_sync_info_t m_slaveSyncs[5];

    unsigned int m_offsetValueChannel1;		
    unsigned int m_offsetValueChannel2;				
};

}

#endif // ANALOGOUTPUTS_H
