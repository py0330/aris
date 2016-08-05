#ifdef UNIX
#include <ecrt.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <sys/mman.h>
#endif
#ifdef WIN32
#define rt_printf printf
#endif


#include <string>
#include <iostream>
#include <map>
#include <fstream>
#include <algorithm>

#include "aris_control_motion.h"


namespace aris
{
    namespace control
    {
        class Motion::Imp
        {
        public:
            enum PDO_Entry
            {
				CONTROLWORD = 0x6040,
				STATUSWORD = 0x6041,
				TARGETPOSITION = 0x607A,
				ACTUALPOSITION = 0x6064,
				TARGETVELOCITY = 0x60FF,
				ACTUALVELOCITY = 0x606C,
				TARGETTORQUE = 0x6071,
				ACTUALTORQUE = 0x6077,
				MODEOPERATION = 0x6060,
				MODEOPERATIONDIS = 0x6061,

				VELOCITYOFFSET = 0x60B1,
				TORQUEOFFSET = 0x60B2,
				MAXTORQUE = 0x6072,
            };

            enum SDO_Entry_Index
            {
                HOMEOFFSET=0x607C,
            };


            Imp(Motion *mot) :pFather(mot) {}
            ~Imp() = default;

            std::int16_t enable(const std::uint8_t mode)
            {
                is_fake = false;

                std::uint16_t statusWord;
                pFather->readPdoIndex(STATUSWORD, 0x00, statusWord);

                std::uint8_t modeRead;
                pFather->readPdoIndex(MODEOPERATIONDIS, 0x00,modeRead);

                int motorState = (statusWord & 0x006F);

                if(motorState !=0x0027){
                    switch (mode)
                    {
                        case Motion::POSITION:
                            //targetposition should be equal to actualposition
                            std::int32_t actualPosition;
                            pFather->readPdoIndex(ACTUALPOSITION, 0x00, actualPosition);
                            pFather->writePdoIndex(TARGETPOSITION, 0x00, actualPosition);
                            break;
                        case Motion::VELOCITY:
                            //velocity loop to set velocity of 0
                            pFather->writePdoIndex(TARGETVELOCITY, 0x00, static_cast<std::int32_t>(0));
                            break;
                        case Motion::TORQUE:
                            pFather->writePdoIndex(TARGETTORQUE, 0x00, static_cast<std::int16_t>(0));
                            pFather->writePdoIndex(MAXTORQUE, 0x00, static_cast<std::int16_t>(1500));
                            break;
                    }
                }

                if(modeRead != mode)
                {
                     /*state is RUNNING, now change it to desired mode*/
                     pFather->writePdoIndex(MODEOPERATION, 0x00, static_cast<std::uint8_t>(mode));
                     return Motion::MODE_CHANGE;
                }

                if (motorState == 0x0060 || motorState ==0x0040)
                {
                    /*switch on disable*/
                    pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x06));
                    return Motion::EXECUTING;
                }
                else if (motorState == 0x0021)
                {
                    /*ready to switch on*/
                    pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x07));
                    return Motion::EXECUTING;
                }
                else if (motorState == 0x0023)
                {
                    /*switch on*/
                    pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x0F));
                    return Motion::EXECUTING;
                }
                else if (motorState == 0x0027)
                {
                    /*successfull, but still need to wait for 10 more cycles to make it stable*/
                    if (++enable_period >= 10)
                    {
                        running_mode = mode;
                        enable_period = 0;
                        return Motion::SUCCESS;
                    }
                    else
                    {
                        return Motion::EXECUTING;
                    }
                }
                else
                {
                    /*the motor is in fault*/
                    pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x80));
                    return Motion::EXE_FAULT;
                }
            }
            std::int16_t disable()
            {
                is_fake = false;

                std::uint16_t statusWord;
                pFather->readPdoIndex(STATUSWORD, 0x00, statusWord);

                int motorState = (statusWord & 0x006F);
                if (motorState == 0x0021)
                {
                    /*alReady disabled*/
                    return Motion::SUCCESS;
                }
                else if (motorState == 0x0023 || motorState == 0x0027 || motorState == 0x0060 || motorState == 0x0040)
                {
                    /*try to disable*/
                    pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x06));
                    return Motion::EXECUTING;
                }
                else
                {
                    /*the motor is in fault*/
                    pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x80));
                    return Motion::EXE_FAULT;
                }

            }
            std::int16_t home()
            {
                if(is_waiting_mode)
                {
                    auto ret = this->enable(running_mode);
                    is_waiting_mode = (ret == 0 ? false : true);
                    return ret;
                }

                std::uint16_t statusWord;
                pFather->readPdoIndex(STATUSWORD, 0x00, statusWord);
                std::uint8_t modeRead;
                pFather->readPdoIndex(MODEOPERATIONDIS, 0x00,modeRead);
                int motorState = (statusWord & 0x3400);

                if(modeRead != Motion::HOME_MODE){
                    pFather->writePdoIndex(MODEOPERATION, 0x00, static_cast<std::uint8_t>(Motion::HOME_MODE));
                    return Motion::MODE_CHANGE;
                }
                else if(motorState == 0x0400){
                    //homing procedure is interrupted or not started
                    if(home_period<10){
                        //write 15 to controlword, make the bit4 equal to 0, 10 times
                        pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x1F));
                        home_period++;
                        return Motion::NOT_START;
                    }
                    else if( home_period<20)
                    {
                        pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x0F));
                        home_period++;
                        return Motion::NOT_START;
                    }
                    else
                    {
                        home_period=0;
                        return Motion::NOT_START;
                    }
                }
                else if(motorState == 0x0000){
                    //in progress
                    home_period=0;
                    pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x1F));
                    pFather->writePdoIndex(TARGETPOSITION, 0x00, home_count_);
                    return Motion::EXECUTING;
                }
                else if(motorState == 0x2000 || motorState == 0x2400)
                {
                    //homing error occurred, velocity is not 0 , or homing error occurred, velocity is 0, should halt
                    pFather->writePdoIndex(CONTROLWORD, 0x00, static_cast<std::uint16_t>(0x0100));
                    rt_printf("%s\n","homing error occurred, the motor is halting!");
                    home_period=0;
                    return Motion::HOME_ERROR;
                }
                else if(motorState == 0x1400)
                {
                    //homing procedure is completed successfully, home method 35<->0x1400,
                    pFather->writePdoIndex(TARGETPOSITION, 0x00, home_count_);
                    home_period=0;
                    is_waiting_mode = true;
                    return Motion::EXECUTING;
                }
                else{
                    //other statusworld
                    home_period=0;
                    return Motion::EXECUTING;
                }
            }
            std::int16_t runPos(const std::int32_t pos, const std::int32_t velocity_offset, const std::int16_t torque_offset)
            {
                if (is_fake)return 0;

                std::uint16_t statusWord;
                pFather->readPdoIndex(STATUSWORD, 0x00, statusWord);
                int motorState = (statusWord & 0x006F);

                std::uint8_t modeRead;
                pFather->readPdoIndex(MODEOPERATIONDIS, 0x00,modeRead);
                if (motorState != 0x0027 )
                {
                    return Motion::ENABLE_ERROR;
                }
                else if(modeRead != Motion::POSITION)
                {
                    return Motion::MODE_ERROR;
                }
                else
                {
                    pFather->writePdoIndex(TARGETPOSITION, 0x00, pos);
                    pFather->writePdoIndex(VELOCITYOFFSET, 0x00, velocity_offset);
                    pFather->writePdoIndex(TORQUEOFFSET, 0x00, torque_offset);

                    return Motion::SUCCESS;
                }
            }
            std::int16_t runVel(const std::int32_t vel, const std::int32_t velocity_offset, const std::int16_t torque_offset)
            {
                if (is_fake)return 0;

                std::uint16_t statusWord;
                pFather->readPdoIndex(STATUSWORD, 0x00, statusWord);
                int motorState = (statusWord & 0x006F);

                std::uint8_t modeRead;
                pFather->readPdoIndex(MODEOPERATIONDIS, 0x00,modeRead);
                if (motorState != 0x0027 )
                {
                    return Motion::ENABLE_ERROR;
                }
                else if(modeRead != Motion::VELOCITY)
                {
                    return Motion::MODE_ERROR;
                }
                else
                {
                    pFather->writePdoIndex(TARGETVELOCITY, 0x00, vel);
                    pFather->writePdoIndex(VELOCITYOFFSET, 0x00, velocity_offset);
                    pFather->writePdoIndex(TORQUEOFFSET, 0x00, torque_offset);
                    return Motion::SUCCESS;
                }
            }
            std::int16_t runTor(const std::int16_t tor, const std::int16_t torque_offset)
            {
                if (is_fake)return 0;

                std::uint16_t statusWord;
                pFather->readPdoIndex(STATUSWORD, 0x00, statusWord);
                int motorState = (statusWord & 0x006F);

                std::uint8_t modeRead;
                pFather->readPdoIndex(MODEOPERATIONDIS, 0x00,modeRead);
                if (motorState != 0x0027 )
                {
                    return Motion::ENABLE_ERROR;
                }
                else if(modeRead != Motion::TORQUE)
                {
                    return Motion::MODE_ERROR;
                }
                else
                {
                    pFather->writePdoIndex(TARGETTORQUE, 0x00, tor);
                    pFather->writePdoIndex(TORQUEOFFSET, 0x00, torque_offset);
                    return Motion::SUCCESS;
                }
            }
            std::int32_t pos() { std::int32_t pos; pFather->readPdoIndex(ACTUALPOSITION, 0x00, pos); return pos; }
            std::int32_t vel() { std::int32_t vel; pFather->readPdoIndex(ACTUALVELOCITY, 0x00, vel); return vel; }
            std::int32_t tor() { std::int16_t tor; pFather->readPdoIndex(ACTUALTORQUE, 0x00, tor); return tor; }
            std::int8_t hasFault()
            {
                std::uint16_t statusWord;
                pFather->readPdoIndex(STATUSWORD, 0x00, statusWord);
                int motorState = (statusWord & 0x0088);
                if(motorState == 0x0000)
                    //no fault and no warning
                    return 0;
                else if(motorState == 0x0008)
                    //fault
                    return -1;
                else if(motorState == 0x0080)
                    //warning
                    return -2;
                else
                    //fault and warning
                    return -3;
            }

            std::int32_t input2count_;
            std::int32_t home_count_;
            double max_pos;
            double min_pos;
            double max_vel;

            Motion *pFather;

            bool is_fake{ true };//for not all motor can be run
            bool is_waiting_mode{ false };

            int enable_period{ 0 };
            int home_period{0};
            std::uint8_t running_mode{ 9 };
        };

        Motion::~Motion() {}
        Motion::Motion(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :SlaveTemplate(father, id, xml_ele), imp_(new Motion::Imp(this))
        {
            imp_->input2count_ = attributeInt32(xml_ele, "input2count");
            imp_->max_pos = attributeDouble(xml_ele, "max_pos");
            imp_->min_pos = attributeDouble(xml_ele, "min_pos");
            imp_->max_vel = attributeDouble(xml_ele, "max_vel");
            imp_->home_count_ = static_cast<std::int32_t>(attributeDouble(xml_ele, "home_pos") * imp_->input2count_);
            configSdoIndex(Imp::HOMEOFFSET, 0x00, static_cast<std::int32_t>(-imp_->home_count_));
        }
        auto Motion::readUpdate()->void
        {
            rxData().feedback_tor = static_cast<double>(imp_->tor());
            rxData().feedback_pos = static_cast<double>(imp_->pos()) / imp_->input2count_;
            rxData().feedback_vel = static_cast<double>(imp_->vel()) / imp_->input2count_;
            rxData().fault_warning=imp_->hasFault();
        }
        auto Motion::writeUpdate()->void
        {
            switch (txData().cmd)
            {
            case IDLE:
                rxData().ret = 0;
                return;
            case ENABLE:
                rxData().ret = imp_->enable(txData().mode);
                return;
            case DISABLE:
                rxData().ret = imp_->disable();
                return;
            case HOME:
                rxData().ret = imp_->home();
                return;
            case RUN:
                switch (txData().mode)
                {
                case POSITION:
                    rxData().ret = imp_->runPos(static_cast<std::int32_t>(txData().target_pos * imp_->input2count_), static_cast<std::int32_t>(txData().vel_offset * imp_->input2count_), static_cast<std::int16_t>(txData().tor_offset));
                    return;
                case VELOCITY:
                    rxData().ret = imp_->runVel(static_cast<std::int32_t>(txData().target_vel * imp_->input2count_), static_cast<std::int32_t>(txData().vel_offset * imp_->input2count_), static_cast<std::int16_t>(txData().tor_offset));
                    return;
                case TORQUE:
                    rxData().ret = imp_->runTor(static_cast<std::int16_t>(txData().target_tor), static_cast<std::int16_t>(txData().tor_offset));
                    return;
                default:
                    rxData().ret = -1;
                    return;
                }
            default:
                rxData().ret = -1;
                return;
            }

        }
        auto Motion::logData(const Slave::TxType &tx_data, const Slave::RxType &rx_data, std::fstream &file)->void
        {
            auto &rx_motiondata=static_cast<const RxType &>(rx_data);
            auto &tx_motiondata=static_cast<const TxType &>(tx_data);
            file<<rx_motiondata.feedback_pos<<" "<< tx_motiondata.target_pos<<" "<<rx_motiondata.feedback_tor;
        }
        auto Motion::maxPos()->double { return imp_->max_pos; }
        auto Motion::minPos()->double { return imp_->min_pos; }
        auto Motion::maxVel()->double { return imp_->max_vel; }
        auto Motion::pos2countRatio()->std::int32_t { return imp_->input2count_; }
    }
}
