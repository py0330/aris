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
		class Motion :public Slave
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Motion"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto maxPos()->double;
			auto minPos()->double;
			auto maxVel()->double;
			auto posOffset()->double;
			auto posFactor()->std::int32_t;

			auto modeOfOperation()const->std::uint8_t;
			auto targetPos()const->double;
			auto targetVel()const->double;
			auto targetCur()const->double;
			auto offsetVel()const->double;
			auto offsetCur()const->double;
			// require pdo 0x6061 //
			auto modeOfDisplay()->std::uint8_t;
			// require pdo 0x6064 //
			auto actualPos()->double;
			// require pdo 0x606C //
			auto actualVel()->double;
			// require pdo 0x6078 //
			auto actualCur()->double;
			// require pdo 0x6060 //
			auto setModeOfOperation(std::uint8_t mode)->void;
			// require pdo 0x607A //
			auto setTargetPos(double pos)->void;
			// require pdo 0x60FF //
			auto setTargetVel(double vel)->void;
			// require pdo 0x6071 //
			auto setTargetCur(double cur)->void;
			// require pdo 0x6071 //
			auto setOffsetVel(double vel)->void;
			// require pdo 0x6071 //
			auto setOffsetCur(double cur)->void;

			// require pdo 0x6040 0x6041 // 
			auto disable()->int;
			// require pdo 0x6040 0x6041 //
			auto enable()->int;
			// require pdo 0x6040 0x6041 0x6060 0x6061 //
			auto home()->int;
			// require pdo 0x6060 0x6061 //
			auto mode(std::uint8_t md)->int;


			virtual ~Motion();
			Motion(Object &father, const aris::core::XmlElement &xml_ele);
			Motion(const std::string &name, const SlaveType &slave_type, std::int32_t pos_factor, double max_pos, double min_pos, double max_vel, double home_pos = 0, double pos_offset = 0);

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;
		};

		class Controller
		{

		};
	}
}

#endif
