#include <Platform.h>

#include <string>

#include "Aris_Core.h"
#include "Aris_IMU.h"
#include "Aris_DynKer.h"
#include "Aris_ExpCal.h"

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>

#include "deviceclass.h"

namespace Aris
{
	namespace Sensor
	{
		void ImuData::ToPmBody2Ground(double *pm) const
		{
			ToPmBody2Ground(pm, this->yaw);
		}
		void ImuData::ToPmBody2Ground(double *pm, double yawValue) const
		{
			double tem_pm[16];
			double pe[6]{ 0,0,0,yawValue, pitch, roll };
			Aris::DynKer::s_pe2pm(pe, tem_pm, "321");
			Aris::DynKer::s_pm_dot_pm(pmLhs, tem_pm, pmRhs, pm);
		}
		void ImuData::ToEulBody2Ground(double *eul, const char *eulType) const
		{
			this->ToEulBody2Ground(eul, this->yaw, eulType);
		}
		void ImuData::ToEulBody2Ground(double *eul, double yawValue, const char *eulType) const
		{
			double pm[16], pe[6];
			this->ToPmBody2Ground(pm, yawValue);
			Aris::DynKer::s_pm2pe(pm, pe, eulType);
			std::copy_n(pe + 3, 3, eul);
		}
		void ImuData::ToOmegaBody2Ground(double *omega) const
		{
			ToOmegaBody2Ground(omega, this->yaw);
		}
		void ImuData::ToOmegaBody2Ground(double *omega, double yawValue) const
		{
			double pm[16];
			ToPmBody2Ground(pm, yawValue);
			Aris::DynKer::s_pm_dot_v3(pm, this->omega, omega);
		}
		void ImuData::ToPntAccBody2Ground(double *acc) const
		{
			ToPntAccBody2Ground(acc, this->yaw);
		}
		void ImuData::ToPntAccBody2Ground(double *acc, double yawValue) const
		{
			/*需要角加速度才能算出准确值*/
			
			//ToPntAccBody2Ground(acc, this->yaw);
		}
		
		
		
		class IMU::IMU_IMP :public DeviceClass
		{
		public:
			std::string port;
			int baudRate;
			int sampleRate;

			double pmImuGround2BodyGround[4][4];
			double pmBody2Imu[4][4];


		public:
			XsPortInfo mtPort;
		};
		
		IMU::IMU():pDevice(new IMU_IMP)
		{
			pDevice->port = "/dev/ttyUSB0";
			pDevice->baudRate = 921600;
			pDevice->sampleRate = 400;

			std::fill_n(&pDevice->pmImuGround2BodyGround[0][0], 16, 0);
			pDevice->pmImuGround2BodyGround[0][0] = -1;
			pDevice->pmImuGround2BodyGround[1][2] = 1;
			pDevice->pmImuGround2BodyGround[2][1] = 1;
			pDevice->pmImuGround2BodyGround[3][3] = 1;

			std::fill_n(&pDevice->pmBody2Imu[0][0], 16, 0);
			pDevice->pmBody2Imu[0][0] = 1;
			pDevice->pmBody2Imu[1][2] = 1;
			pDevice->pmBody2Imu[2][1] = -1;
			pDevice->pmBody2Imu[3][3] = 1;
		}
		IMU::IMU(const Aris::Core::XmlElement *xmlEle) :pDevice(new IMU_IMP)
		{
#ifdef PLATFORM_IS_LINUX
			pDevice->port = xmlEle->Attribute("portLinux");
#endif
			pDevice->baudRate = std::stoi(xmlEle->Attribute("baudRate"));
			pDevice->sampleRate = std::stoi(xmlEle->Attribute("sampleRate"));;

			Aris::DynKer::Calculator c;
			c.AddVariable("PI", PI);
			
			auto m = c.CalculateExpression(xmlEle->Attribute("PeImuGround2BodyGound"));
			Aris::DynKer::s_pe2pm(m.Data(), &pDevice->pmImuGround2BodyGround[0][0]);
			m = c.CalculateExpression(xmlEle->Attribute("PeImu2Body"));
			double pmImu2Body[16];
			Aris::DynKer::s_pe2pm(m.Data(), pmImu2Body);
			Aris::DynKer::s_inv_pm(pmImu2Body, &pDevice->pmBody2Imu[0][0]);

		}
		IMU::~IMU()
		{
		}

		void IMU::Init()
		{
			XsPortInfoArray portInfoArray;
			xsEnumerateUsbDevices(portInfoArray);

			if (!portInfoArray.size())
			{
#ifdef PLATFORM_IS_WINDOWS
				throw std::runtime_error("IMU: failed to find IMU sensor");
#endif
#ifdef PLATFORM_IS_LINUX
				XsPortInfo portInfo(pDevice->port, XsBaud::numericToRate(pDevice->baudRate));
				portInfoArray.push_back(portInfo);
#endif
			}
			
			pDevice->mtPort = portInfoArray.at(0);
			
			// Open the port with the detected device
			if (!pDevice->openPort(pDevice->mtPort))
				throw std::runtime_error("IMU: could not open port.");
			
			Aris::Core::Sleep(10);

			// Put the device in configuration mode
			if (!pDevice->gotoConfig()) // Put the device into configuration mode before configuring the device
			{
				throw std::runtime_error("IMU: could not put device into configuration mode");
			}
			
			// Request the device Id to check the device type
			pDevice->mtPort.setDeviceId(pDevice->getDeviceId());
			
			// Check if we have an MTi / MTx / MTmk4 device
			if (!pDevice->mtPort.deviceId().isMtMk4())
			{
				throw std::runtime_error("IMU: No MTi / MTx / MTmk4 device found.");
			}
			
			// Check device type
			if (pDevice->mtPort.deviceId().isMtMk4())
			{
				XsOutputConfiguration config0(XDI_Quaternion, pDevice->sampleRate);
				XsOutputConfiguration config1(XDI_DeltaQ, pDevice->sampleRate);
				XsOutputConfiguration config2(XDI_DeltaV, pDevice->sampleRate);
				XsOutputConfiguration config3(XDI_Acceleration, pDevice->sampleRate);

				XsOutputConfigurationArray configArray;
				configArray.push_back(config0);
				configArray.push_back(config1);
				configArray.push_back(config2);
				configArray.push_back(config3);
				if (!pDevice->setOutputConfiguration(configArray))
				{
					throw std::runtime_error("IMU: Could not configure MTmk4 pDevice-> Aborting.");
				}
			}
			else
			{
				throw std::runtime_error("IMU: Unknown device while configuring. Aborting.");
			}
			
			// Put the device in measurement mode
			if (!pDevice->gotoMeasurement())
			{
				throw std::runtime_error("IMU: Could not put device into measurement mode. Aborting.");
			}
		}
		void IMU::Release()
		{
			pDevice->close();
		}
		void IMU::UpdateData(ImuData &data)
		{
			XsByteArray imuData;
			XsMessageArray msgs;


			while (msgs.empty())
			{
				Aris::Core::Sleep(1);
				pDevice->readDataToBuffer(imuData);
				pDevice->processBufferedData(imuData, msgs);
			}

			//std::cout << "msg num:" << msgs.size()<<std::endl;

			for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
			{
				// Retrieve a packet
				XsDataPacket packet;
				if ((*it).getMessageId() == XMID_MtData2) 
				{
					packet.setMessage((*it));
					packet.setDeviceId(pDevice->mtPort.deviceId());
				}

				// Get the all data
				auto eul = packet.orientationEuler();
				auto sdi = packet.sdiData();
				auto acc = packet.calibratedAcceleration();

				data.yaw = eul.yaw()*PI / 180;
				data.pitch = eul.pitch()*PI / 180;
				data.roll = eul.roll()*PI / 180;

				data.va = sdi.orientationIncrement().x() * 2 * 100;
				data.vb = sdi.orientationIncrement().y() * 2 * 100;
				data.vc = sdi.orientationIncrement().z() * 2 * 100;
				
				std::copy_n(acc.data(), acc.size(), data.acc);

				data.time = packet.timeOfArrival().nowMs();
				data.pmLhs = *pDevice->pmImuGround2BodyGround;
				data.pmRhs = *pDevice->pmBody2Imu;
			}

			msgs.clear();
		}
	}
}
