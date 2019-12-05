#ifndef ARIS_SERVER_CONTROL_SERVER_H_
#define ARIS_SERVER_CONTROL_SERVER_H_

#include <string>
#include <sstream>
#include <map>
#include <memory>
#include <future>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>
#include <aris/sensor/sensor.hpp>
#include <aris/dynamic/dynamic.hpp>
#include <aris/plan/plan.hpp>

#include "aris/server/interface.hpp"

namespace aris::server
{
	class ControlServer : public aris::core::Object
	{
	public:
		using PreCallback = std::add_pointer<void(ControlServer&)>::type;
		using PostCallback = std::add_pointer<void(ControlServer&)>::type;

		// members //
		static auto instance()->ControlServer &;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		template<typename T = aris::dynamic::Model, typename... Args>
		auto makeModel(Args&&... args)->void { this->resetModel(new T(std::forward<Args>(args)...)); }
		template<typename T = aris::control::Controller, typename... Args>
		auto makeController(Args&&... args)->void { this->resetController(new T(std::forward<Args>(args)...)); }
		template<typename T = aris::sensor::SensorRoot, typename... Args>
		auto makeSensorRoot(Args&&... args)->void { this->resetSensorRoot(new T(std::forward<Args>(args)...)); }
		template<typename T = aris::plan::PlanRoot, typename... Args>
		auto makePlanRoot(Args&&... args)->void { this->resetPlanRoot(new T(std::forward<Args>(args)...)); }
		auto resetModel(dynamic::Model *model)->void;
		auto resetController(control::Controller *controller)->void;
		auto resetSensorRoot(sensor::SensorRoot *sensor_root)->void;
		auto resetPlanRoot(plan::PlanRoot *sensor_root)->void;
		auto model()->dynamic::Model&;
		auto model()const->const dynamic::Model& { return const_cast<ControlServer *>(this)->model(); }
		auto controller()->control::Controller&;
		auto controller()const->const control::Controller& { return const_cast<ControlServer *>(this)->controller(); }
		auto sensorRoot()->sensor::SensorRoot&;
		auto sensorRoot()const->const sensor::SensorRoot& { return const_cast<ControlServer *>(this)->sensorRoot(); }
		auto planRoot()->plan::PlanRoot&;
		auto planRoot()const->const plan::PlanRoot& { return const_cast<ControlServer *>(this)->planRoot(); }
		auto interfacePool()->aris::core::ObjectPool<aris::server::Interface>&;
		auto interfacePool()const->const aris::core::ObjectPool<aris::server::Interface>& { return const_cast<ControlServer *>(this)->interfacePool(); }
		auto interfaceRoot()->InterfaceRoot&;
		auto interfaceRoot()const->const InterfaceRoot& { return const_cast<ControlServer *>(this)->interfaceRoot(); }

		// operation in RT & NRT context //
		auto setRtPlanPreCallback(PreCallback pre_callback)->void;
		auto setRtPlanPostCallback(PostCallback post_callback)->void;
		auto running()->bool;
		auto globalCount()->std::int64_t;
		auto currentExecutePlanRt()->aris::plan::Plan *;

		// operation in NRT context //
		auto open()->void;
		auto close()->void;
		auto runCmdLine()->void;
		auto executeCmd(const aris::core::Msg &cmd_string, std::function<void(aris::plan::Plan&)> post_callback = nullptr)->std::shared_ptr<aris::plan::Plan>;
		auto executeCmdInCmdLine(const aris::core::Msg &cmd_string, std::function<void(aris::plan::Plan&)> post_callback = nullptr)->std::shared_ptr<aris::plan::Plan>;
		auto start()->void;
		auto stop()->void;
		auto currentExecutePlan()->std::shared_ptr<aris::plan::Plan>;
		auto currentCollectPlan()->std::shared_ptr<aris::plan::Plan>;
		auto waitForAllExecution()->void;
		auto waitForAllCollection()->void;
		auto getRtData(const std::function<void(ControlServer&, const aris::plan::Plan *target, std::any&)>& get_func, std::any& data)->void;

		ARIS_REGISTER_TYPE(ControlServer);

	private:
		~ControlServer();
		ControlServer();
		ControlServer(const ControlServer &) = delete;
		ControlServer &operator=(const ControlServer &) = delete;

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
}

#endif

