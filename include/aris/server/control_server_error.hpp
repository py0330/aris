#ifndef ARIS_SERVER_CONTROL_SERVER_ERROR_H_
#define ARIS_SERVER_CONTROL_SERVER_ERROR_H_

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
#include "aris/server/middle_ware.hpp"

namespace aris::server{
	class ARIS_API ControlServerErrorChecker {
	public:
		virtual auto init(aris::server::ControlServer* cs)->void;
		virtual auto checkError(std::int64_t count, const std::uint64_t* mot_options, char* error_msg)->std::int32_t;
		virtual auto fixError()->std::int32_t;
		virtual auto storeServerData()->void;

		ControlServerErrorChecker();
		virtual ~ControlServerErrorChecker();
		ARIS_DECLARE_BIG_FOUR_NOEXCEPT(ControlServerErrorChecker);
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class ARIS_API ControlServerErrorCheckerByModel:public ControlServerErrorChecker {
	public:
		virtual auto init(aris::server::ControlServer* cs)->void override;
		virtual auto checkError(std::int64_t count, const std::uint64_t* mot_options, char* error_msg)->std::int32_t override;
		virtual auto fixError()->std::int32_t override;
		virtual auto storeServerData()->void override;

		ControlServerErrorCheckerByModel();
		virtual ~ControlServerErrorCheckerByModel();
		ARIS_DECLARE_BIG_FOUR_NOEXCEPT(ControlServerErrorCheckerByModel);
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif

