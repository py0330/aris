#include "newslave.h"
namespace robot
{
    class EsgImu::Imp
    {
    public:
        enum PDO_Entry_Index
        {
            EsgImuIndex=0x6011
        };
        enum PDO_Entry_SubIndex
        {
            WX=0x01,
            WY=0x02,
            WZ=0x03,
            APX=0x04,
            APY=0x05,
            APZ=0x06,
            NORTHX=0x07,
            NORTHY=0x08,
            NORTHZ=0x09,
            REX=0x0a,
            REY=0x0b,
            REZ=0x0c,
        };
        const double w_factor=0.01;
        const double ap_factor=0.00025;
        const double mag_factor=1.22;
        const double ang_factor=0.1;

        Imp(EsgImu *esgimu) :pFather(esgimu) {}
        ~Imp() = default;
        std::int16_t wx() { std::int16_t wx; pFather->readPdoIndex(EsgImuIndex, WX, wx); return wx; }
        std::int16_t wy() { std::int16_t wy; pFather->readPdoIndex(EsgImuIndex, WY, wy); return wy; }
        std::int16_t wz() { std::int16_t wz; pFather->readPdoIndex(EsgImuIndex, WZ, wz); return wz; }
        std::int16_t apx() { std::int16_t apx; pFather->readPdoIndex(EsgImuIndex, APX, apx); return apx; }
        std::int16_t apy() { std::int16_t apy; pFather->readPdoIndex(EsgImuIndex, APY, apy); return apy; }
        std::int16_t apz() { std::int16_t apz; pFather->readPdoIndex(EsgImuIndex, APZ, apz); return apz; }
        std::int16_t northx() { std::int16_t northx; pFather->readPdoIndex(EsgImuIndex, NORTHX, northx); return northx; }
        std::int16_t northy() { std::int16_t northy; pFather->readPdoIndex(EsgImuIndex, NORTHY, northy); return northy; }
        std::int16_t northz() { std::int16_t northz; pFather->readPdoIndex(EsgImuIndex, NORTHZ, northz); return northz; }
        std::int16_t rex() { std::int16_t rex; pFather->readPdoIndex(EsgImuIndex, REX, rex); return rex; }
        std::int16_t rey() { std::int16_t rey; pFather->readPdoIndex(EsgImuIndex, REY, rey); return rey; }
        std::int16_t rez() { std::int16_t rez; pFather->readPdoIndex(EsgImuIndex, REZ, rez); return rez; }

        EsgImu *pFather;
    };
    EsgImu::~EsgImu() {}
    EsgImu::EsgImu(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :SlaveTemplate(father, id, xml_ele), imp_(new EsgImu::Imp(this)){}
    auto EsgImu::readUpdate()->void
    {
        rxData().wx=static_cast<double>(imp_->wx())*imp_->w_factor;
        rxData().wy=static_cast<double>(imp_->wy())*imp_->w_factor;
        rxData().wz=static_cast<double>(imp_->wz())*imp_->w_factor;
        rxData().apx=static_cast<double>(imp_->apx())*imp_->ap_factor;
        rxData().apy=static_cast<double>(imp_->apy())*imp_->ap_factor;
        rxData().apz=static_cast<double>(imp_->apz())*imp_->ap_factor;
        rxData().northx=static_cast<double>(imp_->northx())*imp_->mag_factor;
        rxData().northy=static_cast<double>(imp_->northy())*imp_->mag_factor;
        rxData().northz=static_cast<double>(imp_->northz())*imp_->mag_factor;
        rxData().rex=static_cast<double>(imp_->rex())*imp_->ang_factor;
        rxData().rey=static_cast<double>(imp_->rey())*imp_->ang_factor;
        rxData().rez=static_cast<double>(imp_->rez())*imp_->ang_factor;
    }
    auto EsgImu::writeUpdate()->void
    {

    }
    auto EsgImu::logData(const Slave::TxType &tx_data, const Slave::RxType &rx_data, std::fstream &file)->void
    {
        auto &rx_esgimudata=static_cast<const RxType &>(rx_data);
        auto &tx_esgimudata=static_cast<const TxType &>(tx_data);
        file<<rx_esgimudata.wx<<" ";
        file<<rx_esgimudata.wy<<" ";
        file<<rx_esgimudata.wz<<" ";
        file<<rx_esgimudata.apx<<" ";
        file<<rx_esgimudata.apy<<" ";
        file<<rx_esgimudata.apz<<" ";
        file<<rx_esgimudata.northx<<" ";
        file<<rx_esgimudata.northy<<" ";
        file<<rx_esgimudata.northz<<" ";
        file<<rx_esgimudata.rex<<" ";
        file<<rx_esgimudata.rey<<" ";
        file<<rx_esgimudata.rez;
    }

}
