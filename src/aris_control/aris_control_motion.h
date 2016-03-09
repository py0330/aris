#ifndef ARIS_CONTROL_MOTION_H
#define ARIS_CONTROL_MOTION_H

#include <functional>
#include <thread>
#include <atomic>

#include <aris_control_ethercat.h>


namespace Aris
{
	namespace Control
	{	
		class EthercatMotion :public EthercatSlave
		{
		public:
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
				POSITION = 0x0008,
				VELOCITY = 0x0009,
				CURRENT = 0x0010,
			};
			struct RawData
			{
				std::int32_t target_pos{ 0 }, feedback_pos{ 0 };
				std::int32_t target_vel{ 0 }, feedback_vel{ 0 };
				std::int16_t target_cur{ 0 }, feedback_cur{ 0 };
				std::uint8_t cmd{ IDLE };
				std::uint8_t mode{ POSITION };
				mutable std::int16_t ret{ 0 };
			};

			virtual ~EthercatMotion();
			EthercatMotion(const Aris::Core::XmlElement &xml_ele);
			auto hasFault()->bool;
			auto readFeedback(RawData &data)->void;
			auto writeCommand(const RawData &data)->void;
			auto absID()->std::int32_t;
			auto maxVelCount()->std::int32_t;
			auto pos2countRatio()->std::int32_t;

		private:
			class Imp;
			std::unique_ptr<Imp> imp;

			friend class EthercatController;
		};
		class EthercatForceSensor :public EthercatSlave
		{
		public:
			struct Data
			{
				union
				{
					struct { double Fx, Fy, Fz, Mx, My, Mz; };
					double fce[6];
				};
				
			};

			EthercatForceSensor(const Aris::Core::XmlElement &xml_ele): EthercatSlave(xml_ele){};
			auto readData(Data &data)->void;
		};

		class EthercatController :public EthercatMaster
		{
		public:
			struct Data
			{
				const std::vector<EthercatMotion::RawData> *last_motion_rawdata;
				std::vector<EthercatMotion::RawData> *motion_rawdata;
				std::vector<EthercatForceSensor::Data> *force_sensor_data;
				const Aris::Core::MsgRT *msg_recv;
				Aris::Core::MsgRT *pMsgSend;
			};

			virtual ~EthercatController(){};
			virtual void loadXml(const Aris::Core::XmlElement &xml_ele) override;
			virtual void start();
			virtual void stop();
			void setControlStrategy(std::function<int(Data&)>);
			
			std::size_t motionNum() { return motion_vec_.size(); };
			EthercatMotion &motionAtAbs(int i) { return *motion_vec_.at(a2p(i)); };
			EthercatMotion &motionAtPhy(int i) { return *motion_vec_.at(i); };
			std::size_t forceSensorNum() { return force_sensor_vec_.size(); };
			EthercatForceSensor &forceSensorAt(int i) { return *force_sensor_vec_.at(i); };
			Pipe<Aris::Core::Msg>& msgPipe(){return msg_pipe_;};
			
			inline int p2a(const int phy){	return map_phy2abs_[phy];}
			inline int a2p(const int abs){	return map_abs2phy_[abs];}
			inline void p2a(const int *phy, int *abs, int num) { for (int i = 0; i < num; ++i)abs[i] = p2a(phy[i]); }
			inline void a2p(const int *abs, int *phy, int num) { for (int i = 0; i < num; ++i)phy[i] = a2p(abs[i]); }

		protected:
			EthercatController() :EthercatMaster(), msg_pipe_(0, true) {};
			virtual void controlStrategy() override;

		private:
			std::vector<int> map_phy2abs_, map_abs2phy_;


			std::function<int(Data&)> strategy_;
			Pipe<Aris::Core::Msg> msg_pipe_;
			std::atomic_bool is_stopping_;

			std::vector<EthercatMotion *> motion_vec_;
			std::vector<EthercatMotion::RawData> motion_rawdata_, last_motion_rawdata_;

			std::vector<EthercatForceSensor *> force_sensor_vec_;
			std::vector<EthercatForceSensor::Data> force_sensor_data_;

			std::unique_ptr<Pipe<std::vector<EthercatMotion::RawData> > > record_pipe_;
			std::thread record_thread_;

			friend class EthercatMaster;
		};
	}
}

#endif
