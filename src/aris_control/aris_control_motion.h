#ifndef ARIS_CONTROL_MOTION_H
#define ARIS_CONTROL_MOTION_H

#include <functional>
#include <thread>
#include <atomic>

#include <aris_control_ethercat.h>


namespace aris
{
    namespace control
    {
        struct TxMotionData :public Slave::TxType
        {
            double target_pos{ 0 };
            double target_vel{ 0 };
            double target_tor{ 0 };
            double vel_offset{0};
            double tor_offset{0};

            std::uint8_t cmd{ 0 };
            std::uint8_t mode{ 8 };
            std::int8_t home_mode{35};
        };
        struct RxMotionData :public Slave::RxType
        {
            double feedback_pos{ 0 };
            double feedback_vel{ 0 };
            double feedback_tor{ 0 };
            std::uint8_t cmd{ 0 };
            std::uint8_t mode{ 8 };
            std::int8_t fault_warning{0};
        };
        class Motion :public SlaveTemplate<TxMotionData, RxMotionData>
        {
        public:
            enum Error
			{
                SUCCESS=0,
                EXECUTING=1,
                EXE_FAULT=2,//the fault accure during executing ,can reset
                MODE_CHANGE=3,//changing mode
                NOT_START=4,

                CMD_ERROR=-1,//all motor should disable when the error accure
                HOME_ERROR=-2,//all motor should halt when the error accure during executing
                ENABLE_ERROR=-3,//motor change to disable when run
                MODE_ERROR=-4,//motor change to wrong mode when run
            };
            enum Cmd
            {
                IDLE = 0,
                ENABLE,
                DISABLE,
                HOME,
                RUN
            };
            enum Mode
            {
                HOME_MODE = 0x0006,
                POSITION = 0x0008,
                VELOCITY = 0x0009,
                TORQUE = 0x0010,
            };

			static auto Type()->const std::string &{ static const std::string type("Motion"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
            auto maxPos()->double;
            auto minPos()->double;
            auto maxVel()->double;
            auto pos2countRatio()->std::int32_t;

			virtual ~Motion();
			Motion(Object &father, const aris::core::XmlElement &xml_ele);

        protected:
            auto virtual readUpdate()->void override;
            auto virtual writeUpdate()->void override;
            auto virtual logData(const Slave::TxType &tx_data, const Slave::RxType &rx_data, std::fstream &file)->void override;

        private:
            class Imp;
            std::unique_ptr<Imp> imp_;
        };
        class Controller:public Master
        {
        public:
            auto setControlStrategy(std::function<void()> strategy)->void { strategy_ = strategy; }
            Controller() { registerChildType<Motion>(); }

        protected:
            auto controlStrategy()->void override final { if (strategy_)strategy_(); }

        private:
            std::function<void()> strategy_{nullptr};
        };
    }
}

#endif
