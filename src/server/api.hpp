#ifndef ARIS_SERVER_API_H_
#define ARIS_SERVER_API_H_

#include <filesystem>

namespace aris::server
{
	auto setRootPath(std::filesystem::path path)->void;

	auto fetchInterfaceConfig()->std::string;
	auto updateDashboard(std::string dash_id, std::string js_str)->std::string;
	auto createCell(std::string dash_id, std::string cell)->std::string;
	auto deleteCell(std::string dash_id, std::string cell_id)->std::string;

	auto fetchPrograms()->std::string;
	auto updateProgram(std::string pro_name, std::string data)->std::string;
}

#endif