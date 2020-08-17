#include <filesystem>
#include <random>
#include <chrono>
#include <algorithm>
#include <codecvt>


#include "aris.hpp"

#include "aris/server/api.hpp"
#include "json.hpp"
#include "fifo_map.hpp"

namespace aris::server
{
	using Matrix = aris::core::Matrix;

	auto generate_random_id(int length)->std::string 
	{
		const char letters[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

		std::string ret;
		ret.reserve(length);
		for (int i = 0; i < length; ++i)ret.push_back(letters[std::rand() % 62]);
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
		tinyxml2::XMLDocument doc;
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
					my_json j2;//{"name":"Ethercat配置","type":"EthercatConfiguration","i":"EMlxGXxpwDGgz","w":48,"h":23,"x":0,"y":0,"options":"{}"}
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
		tinyxml2::XMLDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());
		
		tinyxml2::XMLElement *dash_ele = nullptr;

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
		tinyxml2::XMLDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		tinyxml2::XMLElement *dash_ele = nullptr;

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
		tinyxml2::XMLDocument doc;
		std::filesystem::path interface_path(rootPath());
		interface_path = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		tinyxml2::XMLElement *dash_ele = nullptr;

		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard") && ele->Attribute("id") == dash_id)
			{
				dash_ele = ele;
				break;
			}
		}
		if (dash_ele == nullptr)return "";

		tinyxml2::XMLElement *cell_ele = nullptr;
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


	std::wstring s2ws(const std::string& str)
	{
		using convert_typeX = std::codecvt_utf8<wchar_t>;
		std::wstring_convert<convert_typeX, wchar_t> converterX;

		return converterX.from_bytes(str);
	}

	std::string ws2s(const std::wstring& wstr)
	{
		using convert_typeX = std::codecvt_utf8<wchar_t>;
		std::wstring_convert<convert_typeX, wchar_t> converterX;

		return converterX.to_bytes(wstr);
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
				
				std::cout << pro_dir_js << std::endl;

				auto ftime = dir.last_write_time();
				std::time_t tt = to_time_t(ftime);
				std::tm *gmt = std::gmtime(&tt);
				std::stringstream buffer;
				buffer << std::put_time(gmt, "%A, %d %B %Y %H:%M");
				
				pro_dir_js["modTime"] = buffer.str();
				pro_dir_js["isDir"] = true;
				
				pro_dir_js["files"] = my_json::array();
				for (auto&file : std::filesystem::directory_iterator(dir))
				{
					if (file.is_regular_file())
					{
						if (file.path().extension() == ".dat")
						{
							// 校验是否为xml //
							tinyxml2::XMLDocument doc;
							if (doc.LoadFile(file.path().string().c_str()))continue;
							
							// push names... //
							my_json file_js;
							file_js["name"] = file.path().filename().string();
							file_js["path"] = "/program/" + dir.path().filename().string() + "/" + file.path().filename().string();
							std::ifstream f(file.path());
							std::string str((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
							file_js["content"] = str;

							// add variables //
							std::function<void(tinyxml2::XMLElement*)> addVariable;
							addVariable = [&](tinyxml2::XMLElement*blk) ->void
							{
								if (blk->Attribute("type", "jointtarget_variable"))
								{
									file_js["jointtargets"].push_back(blk->FirstChildElement("field")->GetText());
								}
								else if (blk->Attribute("type", "robtarget_variable"))
								{
									file_js["robtargets"].push_back(blk->FirstChildElement("field")->GetText());
								}
								else if (blk->Attribute("type", "speed_variable"))
								{
									file_js["speeds"].push_back(blk->FirstChildElement("field")->GetText());
								}
								else if (blk->Attribute("type", "zone_variable"))
								{
									file_js["zones"].push_back(blk->FirstChildElement("field")->GetText());
								}

								if (auto next = blk->FirstChildElement("next"))addVariable(next->FirstChildElement());
							};
							for (auto blk = doc.RootElement()->FirstChildElement(); blk; blk = blk->NextSiblingElement())
							{
								addVariable(blk);
							}

							pro_dir_js["files"].push_back(file_js);
						}
						else if (file.path().extension() == ".pro")
						{
							// 校验是否为xml //
							tinyxml2::XMLDocument doc;
							if (doc.LoadFile(file.path().string().c_str()))continue;
							
							my_json file_js;
							file_js["name"] = file.path().filename().string();
							file_js["path"] = "/program/" + dir.path().filename().string() + "/" + file.path().filename().string();
							std::ifstream f(file.path());
							std::string str((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
							file_js["content"] = str;

							// add functions //
							std::function<void(tinyxml2::XMLElement*)> addFunction;
							addFunction = [&](tinyxml2::XMLElement*blk) ->void
							{
								if (blk->Attribute("type", "function"))
								{
									file_js["functions"].push_back(blk->FirstChildElement("field")->GetText());
								}
								if (auto next = blk->FirstChildElement("next"))addFunction(next->FirstChildElement());
							};
							for (auto blk = doc.RootElement()->FirstChildElement(); blk; blk = blk->NextSiblingElement())
							{
								addFunction(blk);
							}

							pro_dir_js["files"].push_back(file_js);
						}
						else if (file.path().extension() == ".aris")
						{
							my_json file_js;
							file_js["name"] = file.path().filename().string();
							file_js["path"] = "/program/" + dir.path().filename().string() + "/" + file.path().filename().string();
							std::ifstream f(file.path());
							std::string str((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
							file_js["content"] = str;
							pro_dir_js["files"].push_back(file_js);
						}
					}
				}
				
				js[dir.path().filename().string()] = pro_dir_js;
			}
		}

		auto ret = js.dump(-1);
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
	auto createProgram(std::string pro_js_str)->std::string 
	{
		auto js = my_json::parse(pro_js_str);
		auto pro_name = js["name"].get<std::string>();
		auto program_path = rootPath() / "../robot/program" / pro_name;

		if (std::filesystem::exists(program_path))return "";

		std::filesystem::create_directories(program_path);
		std::filesystem::path dat = program_path / (pro_name + ".dat"), pro = program_path / (pro_name + ".pro");

		std::fstream f(pro, std::ios::out | std::ios::trunc);
		f << "<xml xmlns=\"https://developers.google.com/blockly/xml\"></xml>";
		f.close();

		f.open(dat, std::ios::out | std::ios::trunc);
		f << "<xml xmlns=\"https://developers.google.com/blockly/xml\"></xml>";
		f.close();

		f.open(program_path / (pro_name + ".aris"), std::ios::out | std::ios::trunc);
		f << "";
		f.close();

		auto dir = std::filesystem::directory_entry(program_path);

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

		my_json pro_file_js;
		pro_file_js["name"] = pro_name + ".pro";
		pro_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + pro_name + ".pro";
		pro_file_js["content"] = "<xml xmlns=\"https://developers.google.com/blockly/xml\"></xml>";
		pro_dir_js["files"].push_back(pro_file_js);

		my_json dat_file_js;
		dat_file_js["name"] = pro_name + ".dat";
		dat_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + pro_name + ".dat";
		dat_file_js["content"] = "<xml xmlns=\"https://developers.google.com/blockly/xml\"></xml>";
		pro_dir_js["files"].push_back(dat_file_js);

		my_json aris_file_js;
		aris_file_js["name"] = pro_name + ".aris";
		aris_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + pro_name + ".aris";
		aris_file_js["content"] = "";
		pro_dir_js["files"].push_back(aris_file_js);

		auto ret = pro_dir_js.dump(-1);
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
	auto updateProgram(std::string pro_name, std::string data)->std::string
	{
		auto program_path = rootPath() / "../robot/program";
		auto js = my_json::parse(data);
		
		// 先将所有文件存到 temp 路径下 //
		std::filesystem::remove_all(program_path / pro_name / "temp");
		std::filesystem::create_directories(program_path / pro_name / "temp");

		// 保存，并确认是否出错 //
		bool has_error{ false };
		for (auto &file : js["files"])
		{
			std::fstream f(program_path / pro_name / "temp" / file["name"].get<std::string>(), std::ios::out | std::ios::trunc);
			f << file["content"].get<std::string>();
			f.close();

			if (auto ext = std::filesystem::path(file["name"].get<std::string>()).extension(); ext != ".dat" && ext != ".pro") continue;

			tinyxml2::XMLDocument doc;
			if (doc.Parse(file["content"].get<std::string>().c_str())) { has_error = true; break; }

			if (file.contains("jointtargets"))file.erase("jointtargets");
			if (file.contains("robtargets"))file.erase("robtargets");
			if (file.contains("speeds"))file.erase("speeds");
			if (file.contains("zones"))file.erase("zones");
			if (file.contains("functions"))file.erase("functions");
			
			std::function<void(tinyxml2::XMLElement*)> addFunction;
			addFunction = [&](tinyxml2::XMLElement*blk) ->void
			{
				if (blk->Attribute("type", "jointtarget_variable"))
				{
					file["jointtargets"].push_back(blk->FirstChildElement("field")->GetText());
				}
				else if (blk->Attribute("type", "robtarget_variable"))
				{
					file["robtargets"].push_back(blk->FirstChildElement("field")->GetText());
				}
				else if (blk->Attribute("type", "speed_variable"))
				{
					file["speeds"].push_back(blk->FirstChildElement("field")->GetText());
				}
				else if (blk->Attribute("type", "zone_variable"))
				{
					file["zones"].push_back(blk->FirstChildElement("field")->GetText());
				}
				else if (blk->Attribute("type", "function"))
				{
					file["functions"].push_back(blk->FirstChildElement("field")->GetText());
				}
				if (auto next = blk->FirstChildElement("next"))addFunction(next->FirstChildElement());
			};
			for (auto blk = doc.RootElement()->FirstChildElement(); blk; blk = blk->NextSiblingElement())
			{
				addFunction(blk);
			}
		}

		// 如果无错，则覆盖当前文件 //
		if (has_error == false)
		{
			for (auto&file : std::filesystem::directory_iterator(program_path / pro_name))
			{
				if (std::filesystem::is_regular_file(file))
				{
					std::filesystem::remove(file.path());
				}
			}

			for (auto&file : std::filesystem::directory_iterator(program_path / pro_name / "temp"))
			{
				if (std::filesystem::is_regular_file(file))
				{
					std::filesystem::copy_file(file.path(), program_path / pro_name / file.path().filename());
				}
			}
		}

		return js.dump(-1);
	}
	auto deleteProgram(std::string pro_name)->std::string
	{
		auto program_path = rootPath() / "../robot/program" / pro_name;
		if (!std::filesystem::exists(program_path))return "";
		
		std::filesystem::remove_all(program_path);
		
		my_json js;

		js["name"] = pro_name;
		return js.dump(-1);
	}
	auto renameProgram(std::string old_name, std::string new_name_js)->std::string 
	{
		if (!std::filesystem::exists(rootPath() / "../robot/program" / old_name))return "";

		auto js = my_json::parse(new_name_js);

		auto new_name = js["name"].get<std::string>();

		std::filesystem::rename(rootPath() / "../robot/program" / old_name, rootPath() / "../robot/program" / new_name);
		std::filesystem::rename(rootPath() / "../robot/program" / new_name / (old_name + ".pro"), rootPath() / "../robot/program" / new_name / (new_name + ".pro"));
		std::filesystem::rename(rootPath() / "../robot/program" / new_name / (old_name + ".dat"), rootPath() / "../robot/program" / new_name / (new_name + ".dat"));

		///////////////////////以下返回///////////////////////////////////
		auto dir = std::filesystem::directory_entry(rootPath() / "../robot/program" / new_name);

		std::filesystem::path dat = rootPath() / "../robot/program" / new_name / (new_name + ".dat")
			, pro = rootPath() / "../robot/program" / new_name / (new_name + ".pro");

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

		my_json pro_file_js;
		pro_file_js["name"] = pro.filename().string();
		pro_file_js["path"] = "/program/" + dir.path().filename().string() + "/" + pro.filename().string();
		std::ifstream f(pro);
		std::string str((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
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

		return pro_dir_js.dump(-1);
	}

	auto fetchConfigXml()->std::string
	{
		return aris::core::toXmlString(aris::server::ControlServer::instance());
	}

	auto fetchESIPath()->std::string
	{
		auto p = std::filesystem::absolute(rootPath() / "../robot/esi");
		my_json js;
		js["path"] = p.string();
		return js.dump(-1);
	}
	auto loadConfigModel()->std::string;

	auto fetchRobotModels()->std::string;
	auto deleteRobotModelPart()->std::string;
	auto deleteRobotModel()->std::string;
	auto createRobotModel()->std::string;

	auto fetchLogName()->std::string;
	auto fetchLogContent()->std::string;

	auto fetchObjPictureList()->std::string
	{
		auto detech_path = std::filesystem::absolute(rootPath() / "detech/");

		my_json file_list;
		file_list["fileList"] = my_json::array();

		int uid = 0;
		for (auto&file : std::filesystem::directory_iterator(detech_path))
		{
			my_json file_js;
			if (file.is_regular_file())
			{
				uid--;
				file_js["uid"] = uid;
				file_js["name"] = ws2s(file.path().filename().wstring());
				file_js["status"] = "done";
				file_js["url"] = ws2s(("detech" / file.path().filename()).wstring());
			}

			file_list["fileList"].push_back(file_js);
		}

		return file_list.dump(-1);
	}
	auto postObjPicture(std::string str)->std::string
	{
		//std::cout << str << std::endl;
		std::stringstream ss(str);

		int head_size = 0;

		std::string line;
		std::getline(ss, line);
		head_size += line.size();
		std::getline(ss, line);
		head_size += line.size();
		auto filename_raw = line.substr(line.find("filename=\"") + 10);
		filename_raw = filename_raw.substr(0, filename_raw.size() - 2);

		std::wstring filename=s2ws(filename_raw.data());


		std::getline(ss, line);
		head_size += line.size();
		std::getline(ss, line);
		head_size += line.size();
		
		auto data = str.substr(head_size + 4, str.rfind("\r\n", str.size() - 5) - head_size - 4);

		std::fstream file;
		file.open(rootPath() / "detech" / (filename), std::ios::out | std::ios::trunc | std::ios::binary);
		file.write(data.data(), data.size());
		
		return "";
	}
}