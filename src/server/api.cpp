#include <filesystem>
#include <random>
#include <chrono>
#include <algorithm>

#include "aris.hpp"

#include "api.hpp"
#include "json.hpp"
#include "fifo_map.hpp"

namespace aris::server
{
	auto generate_random_id(int length)->std::string 
	{
		const char letters[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ:;.,?!-_+=-*";

		std::string ret;
		ret.reserve(length);
		for (int i = 0; i < length; ++i)ret.push_back(letters[std::rand() % 64]);
		return ret;
	};
	
	
	
	
	
	std::filesystem::path path_;
	auto rootPath()-> std::filesystem::path { return path_; };
	auto setRootPath(std::filesystem::path path)->void { path_ = path; }


	template<class K, class V, class dummy_compare, class A>
	using my_workaround_fifo_map = nlohmann::fifo_map<K, V, nlohmann::fifo_map_compare<K>, A>;
	using my_json = nlohmann::basic_json<my_workaround_fifo_map>;

	auto fetchInterfaceConfig()->std::string
	{
		aris::core::XmlDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		my_json js;
		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard"))
			{
				my_json js1;
				js1["name"] = std::string(ele->Attribute("name"));
				js1["editable"] = std::string(ele->Attribute("editable")) == "true";
				js1["i"] = std::string(ele->Attribute("id"));
				js1["cells"] = std::vector<std::string>();
				for (auto e1 = ele->FirstChildElement(); e1; e1 = e1->NextSiblingElement())
				{
					my_json j2;//{"name":"EthercatÅäÖÃ","type":"EthercatConfiguration","i":"EMlxGXxpwDGgz","w":48,"h":23,"x":0,"y":0,"options":"{}"}
					j2["name"] = e1->Attribute("name");
					j2["type"] = e1->Attribute("type");
					j2["i"] = e1->Attribute("id");
					j2["w"] = e1->IntAttribute("width");
					j2["h"] = e1->IntAttribute("height");
					j2["x"] = e1->IntAttribute("x");
					j2["y"] = e1->IntAttribute("y");
					j2["options"] = e1->Attribute("options");
					js1["cells"].push_back(j2);
				}
				js["dashboards"].push_back(js1);
			}
			else if (ele->Name() == std::string("WebSocket"))
			{
				js["ws"]["url"] = ele->Attribute("url");
				js["ws"]["commandSendInterval"] = ele->IntAttribute("commandSendInterval");
				js["ws"]["commandSendDelay"] = ele->IntAttribute("commandSendDelay");
				js["ws"]["getInterval"] = ele->IntAttribute("getInterval");
				js["ws"]["unityUpdateInterval"] = ele->IntAttribute("unityUpdateInterval");
			}
			else if (ele->Name() == std::string("LayoutConfig"))
			{
				js["layoutConfig"]["cols"] = ele->IntAttribute("cols");
				js["layoutConfig"]["rowHeight"] = ele->IntAttribute("rowHeight");
				js["layoutConfig"]["margin"] = ele->IntAttribute("margin");
				js["layoutConfig"]["containerPadding"] = ele->IntAttribute("containerPadding");
				js["layoutConfig"]["theme"] = ele->Attribute("theme");
			}
		}
		
		return js.dump(-1);
	}
	auto updateDashboard(std::string dash_id, std::string js_str)->std::string 
	{
		aris::core::XmlDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());
		
		aris::core::XmlElement *dash_ele = nullptr;

		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard") && ele->Attribute("id") == dash_id) 
			{
				dash_ele = ele;
				break;
			}
		}

		if (dash_ele == nullptr)return "";

		auto js = my_json::parse(js_str);


		dash_ele->SetAttribute("name", js["name"].get<std::string>().c_str());
		dash_ele->SetAttribute("editable", js["editable"].get<bool>());
		dash_ele->SetAttribute("id", js["i"].get<std::string>().c_str());

		dash_ele->DeleteChildren();
		for (auto js2 : js["cells"])
		{
			auto new_cell = doc.NewElement("Cell");

			new_cell->SetAttribute("name",    js2["name"].get<std::string>().c_str());
			new_cell->SetAttribute("type",    js2["type"].get<std::string>().c_str());
			new_cell->SetAttribute("id",      js2["i"].get<std::string>().c_str());
			new_cell->SetAttribute("width",   js2["w"].get<int>());
			new_cell->SetAttribute("height",  js2["h"].get<int>());
			new_cell->SetAttribute("x",       js2["x"].get<int>());
			new_cell->SetAttribute("y",       js2["y"].get<int>());
			new_cell->SetAttribute("options", js2["options"].get<std::string>().c_str());

			dash_ele->InsertEndChild(new_cell);
		}

		doc.SaveFile((rootPath() / "../robot/interface.xml").string().c_str());

		return js_str;
	}
	auto createCell(std::string dash_id, std::string cell)->std::string
	{
		aris::core::XmlDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		aris::core::XmlElement *dash_ele = nullptr;

		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard") && ele->Attribute("id") == dash_id)
			{
				dash_ele = ele;
				break;
			}
		}

		if (dash_ele == nullptr)return "";


		auto cell_js = my_json::parse(cell);
		auto new_cell = doc.NewElement("Cell");
		auto id = generate_random_id(10);

		new_cell->SetAttribute("name", cell_js["name"].get<std::string>().c_str());
		new_cell->SetAttribute("type", cell_js["type"].get<std::string>().c_str());
		new_cell->SetAttribute("width", cell_js["w"].get<int>());
		new_cell->SetAttribute("height", cell_js["h"].get<int>());
		new_cell->SetAttribute("id", id.c_str());
		new_cell->SetAttribute("x", 0);
		new_cell->SetAttribute("y", 0);
		new_cell->SetAttribute("options", "");

		cell_js["i"] = id;
		cell_js["x"] = 0;
		cell_js["y"] = 0;
		cell_js["options"] = "{}";

		doc.SaveFile((rootPath() / "../robot/interface.xml").string().c_str());

		return cell_js.dump(-1);
	}
	auto deleteCell(std::string dash_id, std::string cell_id)->std::string
	{
		aris::core::XmlDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		aris::core::XmlElement *dash_ele = nullptr;

		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard") && ele->Attribute("id") == dash_id)
			{
				dash_ele = ele;
				break;
			}
		}
		if (dash_ele == nullptr)return "";

		aris::core::XmlElement *cell_ele = nullptr;
		for (auto ele = dash_ele->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			std::cout << ele->Name() << " " << ele->Attribute("id") << "  " << cell_id << std::endl;
			
			if (ele->Name() == std::string("Cell") && ele->Attribute("id") == cell_id)
			{
				cell_ele = ele;
				break;
			}
		}
		if (cell_ele == nullptr)return "";
		

		my_json j2;
		j2["name"] = cell_ele->Attribute("name");
		j2["type"] = cell_ele->Attribute("type");
		j2["i"] = cell_ele->Attribute("id");
		j2["w"] = cell_ele->IntAttribute("width");
		j2["h"] = cell_ele->IntAttribute("height");
		j2["x"] = cell_ele->IntAttribute("x");
		j2["y"] = cell_ele->IntAttribute("y");
		j2["options"] = cell_ele->Attribute("options");
		
		if (cell_ele)dash_ele->DeleteChild(cell_ele);
		doc.SaveFile((rootPath() / "../robot/interface.xml").string().c_str());

		return j2.dump(-1);
	}

	template <typename TP>
	std::time_t to_time_t(TP tp)
	{
		using namespace std::chrono;
		auto sctp = time_point_cast<system_clock::duration>(tp - TP::clock::now()
			+ system_clock::now());
		return system_clock::to_time_t(sctp);
	}

	auto fetchPrograms()->std::string
	{
		my_json js;

		std::filesystem::path program_path(rootPath());
		program_path = program_path / "../robot/program";

		for (auto&dir : std::filesystem::directory_iterator(program_path))
		{
			if (std::filesystem::is_directory(dir))
			{
				my_json pro_dir_js;
				pro_dir_js["name"] = dir.path().filename().string();
				pro_dir_js["path"] = "/program/" + dir.path().filename().string();
				
				auto ftime = dir.last_write_time();
				std::time_t tt = to_time_t(ftime);
				std::tm *gmt = std::gmtime(&tt);
				std::stringstream buffer;
				buffer << std::put_time(gmt, "%A, %d %B %Y %H:%M");
				buffer.str();
				
				pro_dir_js["modTime"] = buffer.str();
				pro_dir_js["isDir"] = true;
				
				std::filesystem::path dat, pro;
				for (auto&file : std::filesystem::directory_iterator(dir))
				{
					if (file.is_regular_file())
					{
						if (file.path().extension() == ".dat")
						{
							dat = file.path();
						}
						else if (file.path().extension() == ".pro")
						{
							pro = file.path();
						}
					}
				}

				my_json pro_file_js;
				pro_file_js["name"] = pro.filename().string();
				pro_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + pro.filename().string();
				std::ifstream f(pro);
				std::string str((std::istreambuf_iterator<char>(f)),std::istreambuf_iterator<char>());
				
				std::cout << str << std::endl;
				std::cout << str.size() << std::endl;

				pro_file_js["content"] = str;
				pro_dir_js["files"].push_back(pro_file_js);

				
				my_json dat_file_js;
				dat_file_js["name"] = dat.filename().string();
				dat_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + dat.filename().string();
				f.close();
				f.open(dat);
				str = std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
				dat_file_js["content"] = str;
				pro_dir_js["files"].push_back(dat_file_js);
				
				js[dir.path().filename().string()] = pro_dir_js;
			}
		}


		auto ret = js.dump();
		int index = 0;
		while (true) {
			index = ret.find("<", index);
			if (index == std::string::npos) break;
			ret.replace(index, 1, "\\u003c");
			index += 5;
		}
		index = 0;
		while (true) {
			index = ret.find(">", index);
			if (index == std::string::npos) break;
			ret.replace(index, 1, "\\u003e");
			index += 5;
		}
		
		return ret;

	}
	
	auto createProgram()->std::string;
	auto updateProgram(std::string pro_name, std::string data)->std::string
	{
		std::filesystem::path program_path(rootPath());
		program_path = program_path / "../robot/program";

		std::fstream pro(program_path / (pro_name + ".pro"));

		auto js = my_json::parse(data);

		std::cout << js["files"][0]["content"].get<std::string>() << std::endl;

		pro << js["files"][0]["content"].get<std::string>();

		

		return data;
	}
	auto deleteProgram()->std::string;
	auto renameProgram()->std::string;

	auto fetchESIPath()->std::string;

	

	auto fetchConfigXml()->std::string;
	auto loadConfigModel()->std::string;

	auto fetchRobotModels()->std::string;
	auto deleteRobotModelPart()->std::string;
	auto deleteRobotModel()->std::string;
	auto createRobotModel()->std::string;

	auto fetchLogName()->std::string;
	auto fetchLogContent()->std::string;
}



