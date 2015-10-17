#include <Platform.h>
#ifdef PLATFORM_IS_LINUX
#include <ecrt.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <sys/mman.h>
#endif
#ifdef PLATFORM_IS_WINDOWS
#define rt_printf printf
#endif


#include <string>
#include <iostream>
#include <map>
#include <Aris_Motion.h>


namespace Aris
{
	namespace Control
	{
		MOTION::MOTION(Aris::Core::ELEMENT *ele)
			:ETHERCAT_SLAVE(ele)
		{
		};
		void MOTION::Initialize()
		{
            this->WriteSdo(6, 950);//torque limit
            this->WriteSdo(9, 0);//home offset
			this->ETHERCAT_SLAVE::Initialize();
		}
        int MOTION::Enable(MODE mode)
        {
            std::uint16_t statusWord;
            this->ReadPdo(1,3, statusWord);
            
			std::uint8_t modeRead;
            this->ReadPdo(4,0, modeRead);

			int motorState = (statusWord & 0x000F);

            if(motorState==0x0000)
            {
				/*state is POWERED_OFF, now set it to STOPPED*/
				this->WritePdo(0,4,static_cast<std::uint16_t>(0x06));
                return 1;
            }
            else if (motorState==0x0001)
            {
				/*state is STOPPED, now set it to ENABLED*/
                this->WritePdo(0,4,static_cast<std::uint16_t>(0x07));
                return 1;
            }
            else if(motorState==0x0003)
            {
				/*state is ENABLED, now set it to RUNNING*/
                this->WritePdo(0,4,static_cast<std::uint16_t>(0x0F));
                return 1;
            }
            else if(modeRead !=mode)
            {
				/*state is RUNNING, now change it to desired mode*/
                this->WritePdo(0,5,static_cast<std::uint8_t>(mode));
                return 1;
            }
            else if(motorState==0x0007)
            {
				/*successfull, but still need to wait for 10 more cycles to make it stable*/
                switch(mode)
                {
				case MOTION::VELOCITY:
					this->WritePdo(0, 1, static_cast<std::int32_t>(0));
                    break;
                case MOTION::CURRENT:
					this->WritePdo(0, 2, static_cast<std::int16_t>(0));
					this->WritePdo(0, 3, static_cast<std::int16_t>(1500));
					break;
                }
                if(++enableCount >=10)
                {
                    runningMode=mode;
                    enableCount=0;
                    return 0;
                }
				else
				{
					return 1;
				}
                    
            }
            else
            {
                /*the motor is in fault*/
				this->WritePdo(0,4,static_cast<std::uint16_t>(0x80));
                return 1;
            }
         }
        int MOTION::Disable()
        {
            std::uint16_t statusword;
            this->ReadPdo(1,3,statusword);

            int motorState=(statusword & 0x000F);
            if(motorState==0x0001)
            {
                /*already disabled*/
                return 0;
            }
            else if(motorState==0x0003||motorState==0x0007||motorState==0x0000)
            {
				/*try to disable*/
				this->WritePdo(0,4,static_cast<std::uint16_t>(0x06));
				return 1;
            }
            else
            {
				/*the motor is in fault*/
				this->WritePdo(0,4,static_cast<std::uint16_t>(0x80));
				return 1;
            }

        }
        int MOTION::Home()
        {
            std::uint16_t statusword;
			this->ReadPdo(1, 3, statusword);
            int motorState=(statusword & 0x000F);
            if(motorState==0x007) 
            {
				/*motor is in running state*/
                std::uint8_t mode_read;
				this->ReadPdo(4, 0, mode_read);
				if (mode_read != 0x006)
                {
                    /*set motor to mode 0x006, which is homing mode*/
					this->WritePdo(0,5,static_cast<std::uint8_t>(0x006));
                    return 1;
                }
                else
                {
                    if (statusword & 0x1000)
                    {
                        /*home finished, set mode to running mode, whose value is decided by enable function*/
                        this->WritePdo(0,5,static_cast<uint8_t>(runningMode));
                        isEverHomed=true;
                        return 0;
                    }
                    else
                    {
						/*still homing*/
                        this->WritePdo(0,4,static_cast<uint16_t>(0x1F));
                        return 1;
                    }
                }
            }
			else
			{
				return -1;
			}
        }
        int MOTION::RunPos(std::int32_t pos)
        {
            std::uint16_t statusword;
            this->ReadPdo(1,3,statusword);
            int motorState=(statusword & 0x000F);

            std::uint8_t mode_read;
            this->ReadPdo(4,0,mode_read);
			if (motorState != 0x0007 || mode_read != VELOCITY)
			{
				return -1;
			}
            else
            {
                std::int32_t current_pos=this->Pos();
                double Kp=150;
				std::int32_t desired_vel = static_cast<std::int32_t>(Kp*(pos - current_pos));
				this->WritePdo(0, 1, desired_vel);
                return 0;
            }
        }
        int MOTION::RunVel(std::int32_t vel)
        {
            std::uint16_t statusword;
            this->ReadPdo(1,3,statusword);
            int motorState=(statusword & 0x000F);

            std::uint8_t mode_read;
            this->ReadPdo(4,0,mode_read);
            if(motorState!=0x0007||mode_read!=VELOCITY)
            {
                return -1;
            }
            else
            {
                this->WritePdo(0,1,vel);
                return 0;
            }
        }
        int MOTION::RunCur(std::int16_t cur)
        {
            std::uint16_t statusword;
            this->ReadPdo(1,3,statusword);
            int motorState=(statusword & 0x000F);

            std::uint8_t mode_read;
            this->ReadPdo(4,0,mode_read);
            if(motorState!=0x0007||mode_read!=CURRENT) //need running and cur mode
            {
                return -1;
            }
            else
            {
                this->WritePdo(0,2,cur);
                return 0;
            }
        }
        bool MOTION::HasFault()
        {
            std::uint16_t statusword;
            this->ReadPdo(1,3,statusword);
            int motorState=(statusword & 0x000F);
			if (motorState != 0x0003 && motorState != 0x0007 && motorState != 0x0001 && motorState != 0x0000)
                return true;
            else
                return false;
        }
		
		void CONTROLLER::LoadXml(Aris::Core::ELEMENT *ele)
		{
			/*Load EtherCat slave types*/
			std::map<std::string, Aris::Core::ELEMENT *> slaveTypeMap;

			auto pSlaveTypes = ele->FirstChildElement("SlaveType");
			for (auto pType = pSlaveTypes->FirstChildElement(); pType != nullptr; pType = pType->NextSiblingElement())
			{
				slaveTypeMap.insert(std::make_pair(std::string(pType->Name()), pType));
			}

			/*Load all slaves*/
			auto pSlaves = ele->FirstChildElement("Slave");
			for (auto pSla = pSlaves->FirstChildElement(); pSla != nullptr; pSla = pSla->NextSiblingElement())
			{
				std::string type{ pSla->Attribute("type") };
				if (type == "ElmoSoloWhistle")
				{
					pMotions.push_back(AddSlave<MOTION>(slaveTypeMap.at(type)));
				}
				else if (type == "AtiForceSensor")
				{
				}
				else
				{
					throw std::runtime_error(std::string("unknown slave type of \"") + type + "\"");
				}
				
			}
			
			/*
			auto pSlaves = ele->FirstChildElement("Slave");

			for (auto pSla = pSlaves->FirstChildElement(); pSla != nullptr; pSla = pSla->NextSiblingElement())
			{			
				static int i = 0;
				pMotions.push_back(AddSlave<MOTION>(pSla));
			}*/
		}
		void CONTROLLER::ControlStrategy()
		{

            static int i = 0;
            i++;

            const int motorNum=3;

///////*************disable************//////////
          static int disabledNum=0;

          static bool isDisabled[motorNum];



          if(i>1000)
          {
              if(disabledNum<motorNum)
             {

                 for(int i=0;i<motorNum;i++)
                 {
                     if(isDisabled[i]==false)
                     {
                         if(Motion(i)->Disable()==0)
                         {
                             isDisabled[i]=true;
                             rt_printf("motor %d disabled\n",i);
                             disabledNum++;
                         }
                      }
                 }


             }
          }




///////*************enable************//////////
       /*   if(disabledNum==1)
          {
              Motion(0)->Enable(MOTION::VELOCITY);
          }*/

         static int enabledNum=0;

          static bool isEnabled[motorNum];


          if(i>5000)
          {

              if(disabledNum==motorNum)
              {

                  if(enabledNum<motorNum)
                  {
                       //rt_printf("enabling...\n");

                      for(int i=0;i<motorNum;i++)
                      {
                          if(isEnabled[i]==false)
                          {
                              if(Motion(i)->Enable(MOTION::VELOCITY)==0)
                              {
                                  isEnabled[i]=true;
                                  rt_printf("motor %d enabled\n",i);
                                  enabledNum++;
                                  rt_printf("enabled num %d\n",enabledNum);
                              }

                          }

                      }
                  }


              }

          }

          static bool ishomed[motorNum];
          static int homedNum;
          if(enabledNum==motorNum)
          {

              for(int i=0;i<motorNum;i++)
              {
                  if(ishomed[i]==false)
                  {
                      if(Motion(i)->Home()==0)
                      {
                          ishomed[i]=true;
                          homedNum++;
                      }
                   }
              }



          }









/////////////////*****go position**********///////////////

                if(homedNum==motorNum)
               {

                   static int k[motorNum];

                   for(int i=0;i<motorNum;i++)
                   {
                       std::int32_t pos=k[i]*200;
                       Motion(i)->RunPos(pos);
                       k[i]++;
                    }
               }


#ifdef PLATFORM_IS_LINUX

            if(i%1000==0)
            {
                for(int i=0;i<motorNum;i++)
                {
                    std::uint16_t state;
                    std::uint8_t mode;
                    std::int16_t current;
                    Motion(i)->ReadPdo(1,3,state);
                    Motion(i)->ReadPdo(4,0,mode);
                    Motion(i)->ReadPdo(2,0,current);
                    rt_printf("motor %d,state 0x%4x,mode %d pos %d, current %d\n",i,state,mode,Motion(i)->Pos(),current);
                    std::int32_t home_mode,home_offset,home_torque,home_speed,home_acc;
                    Motion(i)->ReadSdo(0,home_mode);
                    Motion(i)->ReadSdo(9,home_offset);
                    Motion(i)->ReadSdo(6,home_torque);
                    Motion(i)->ReadSdo(3,home_speed);
                    Motion(i)->ReadSdo(2,home_acc);
                    //rt_printf("mode home%d offset %d torquelimit%d, home_speed %d home-acc %d\n",home_mode,home_offset,home_torque,home_speed,home_acc);


                }


            }

            if ((i % 10000) == 0)
			{
              //rt_printf("pos is:%d\n", pos);
                Aris::Core::RT_MSG::instance[0].CopyStruct(Motion(0)->Pos());
				MsgPipe().SendToNRT(Aris::Core::RT_MSG::instance[0]);
			}
#endif
			int length;
			if((length=MsgPipe().RecvInRT(Aris::Core::RT_MSG::instance[0]))>0)
			{
				rt_printf(Aris::Core::RT_MSG::instance[0].GetDataAddress());
			};
			
		}
	}
}
