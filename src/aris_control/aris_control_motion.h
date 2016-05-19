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
			double target_pos;
		};
		struct RxMotionData :public Slave::RxType
		{
			double pos, vel, acc, cur;
		};
		class Motion :public SlaveTemplate<TxMotionData, RxMotionData>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("motion"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			Motion(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		protected:
			virtual auto readUpdate()->void override;
			virtual auto writeUpdate()->void override;

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;
		};

		class Controller:public Master
		{
		public:
			auto setControlStrategy(std::function<void(Controller*)> strategy) { strategy_ = strategy; };
			Controller() { registerChildType<Motion, false, false, false, false>(); }

		protected:
			auto controlStrategy()->void override final { if (strategy_)strategy_(this); };

		private:
			std::function<void(Controller*)> strategy_{nullptr};
		};
	}
}

#endif
