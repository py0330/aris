#ifndef NEWSLAVE_H
#define NEWSLAVE_H
#include <aris.h>

namespace robot
{
    struct TxEsgImuData :public aris::control::Slave::TxType
    {

    };
    struct RxEsgImuData :public aris::control::Slave::RxType
    {
        double wx,wy,wz,apx,apy,apz,northx,northy,northz,rex,rey,rez;
    };
    class EsgImu :public aris::control::SlaveTemplate<TxEsgImuData, RxEsgImuData>
    {
    public:
        static auto Type()->const std::string &{ static const std::string type("esgimu"); return std::ref(type); }
        virtual auto type() const->const std::string&{ return Type(); }
        EsgImu(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

        virtual ~EsgImu();
    protected:
        virtual auto readUpdate()->void override;
        virtual auto writeUpdate()->void override;
        virtual auto logData(const Slave::TxType &tx_data, const Slave::RxType &rx_data, std::fstream &file)->void override;
    private:
        class Imp;
        std::unique_ptr<Imp> imp_;
    };


    struct TxRecordData :public aris::control::Slave::TxType
    {

    };
    struct RxRecordData :public aris::control::Slave::RxType
    {
        double target_pos_{0};
    };
    class Record :public aris::control::SlaveTemplate<TxRecordData, RxRecordData>
    {
    public:
        static auto Type()->const std::string &{ static const std::string type("record"); return std::ref(type); }
        virtual auto type() const->const std::string&{ return Type(); }
        Record(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

        virtual ~Record();
    protected:
        virtual auto readUpdate()->void override{}
        virtual auto writeUpdate()->void override{}
        virtual auto logData(const Slave::TxType &tx_data, const Slave::RxType &rx_data, std::fstream &file)->void override;
    };

}






#endif
