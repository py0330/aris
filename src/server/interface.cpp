#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>

#include "aris/server/interface.hpp"
#include "aris/server/control_server.hpp"

#include "json.hpp"




namespace aris::server
{
	auto InterfaceRoot::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		auto ins = doc_.RootElement()->DeepClone(xml_ele.GetDocument());
		xml_ele.Parent()->InsertAfterChild(&xml_ele, ins);
		xml_ele.Parent()->DeleteChild(&xml_ele);
	}
	auto InterfaceRoot::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		doc_.Clear();
		auto root = xml_ele.DeepClone(&doc_);
		doc_.InsertEndChild(root);
	}

	Interface::Interface(const std::string &name) :Object(name) {}

	auto parse_ret_value(std::vector<std::pair<std::string, std::any>> &ret)->std::string
	{
		nlohmann::json js;
		
		for (auto &key_value : ret)
		{
#define ARIS_SET_TYPE(TYPE) if (auto value = std::any_cast<TYPE>(&key_value.second)) js[key_value.first] = *value; else

			ARIS_SET_TYPE(bool)
			ARIS_SET_TYPE(int)
			ARIS_SET_TYPE(double)
			ARIS_SET_TYPE(std::string)
			ARIS_SET_TYPE(std::vector<bool>)
			ARIS_SET_TYPE(std::vector<int>)
			ARIS_SET_TYPE(std::vector<double>)
			ARIS_SET_TYPE(std::vector<std::string>)
			{
				std::cout << "unrecognized return value" << std::endl;
			}

#undef ARIS_SET_TYPE
		}
		
		return  js.dump(2);
	}
	auto onReceivedMsg(aris::core::Socket *socket, aris::core::Msg &msg)->int
	{
		std::string msg_data = msg.toString();

		LOG_INFO << "receive cmd:"
			<< msg.header().msg_size_ << "&"
			<< msg.header().msg_id_ << "&"
			<< msg.header().msg_type_ << "&"
			<< msg.header().reserved1_ << "&"
			<< msg.header().reserved2_ << "&"
			<< msg.header().reserved3_ << ":"
			<< msg_data << std::endl;

		try
		{
			aris::server::ControlServer::instance().executeCmdInCmdLine(aris::core::Msg(msg), [socket, msg](aris::plan::Plan &plan)->void
			{
				// make return msg
				aris::core::Msg ret_msg(msg);

				// only copy if it is a str
				if (auto str = std::any_cast<std::string>(&plan.ret()))
				{
					ret_msg.copy(*str);
				}
				else if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
				{
					js->push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
					js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
					ret_msg.copy(parse_ret_value(*js));
				}

				// return back to source
				try
				{
					socket->sendMsg(ret_msg);
				}
				catch (std::exception &e)
				{
					std::cout << e.what() << std::endl;
					LOG_ERROR << e.what() << std::endl;
				}
			});
		}
		catch (std::exception &e)
		{
			std::vector<std::pair<std::string, std::any>> ret_pair;
			ret_pair.push_back(std::make_pair<std::string, std::any>("return_code", int(aris::plan::Plan::PARSE_EXCEPTION)));
			ret_pair.push_back(std::make_pair<std::string, std::any>("return_message", std::string(e.what())));
			std::string ret_str = parse_ret_value(ret_pair);

			std::cout << ret_str << std::endl;
			LOG_ERROR << ret_str << std::endl;

			try
			{
				aris::core::Msg m = msg;
				m.copy(ret_str);
				socket->sendMsg(m);
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		}

		return 0;
	}
	auto onReceivedConnection(aris::core::Socket *sock, const char *ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	}
	auto onLoseConnection(aris::core::Socket *socket)->int
	{
		std::cout << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer(socket->port());
				break;
			}
			catch (std::runtime_error &e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	}
	auto WebInterface::open()->void { sock_->startServer(); }
	auto WebInterface::close()->void { sock_->stop(); }
	auto WebInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Interface::loadXml(xml_ele);
		this->sock_ = findOrInsertType<aris::core::Socket>("socket", "", "5866", aris::core::Socket::WEB);

		sock_->setOnReceivedMsg(onReceivedMsg);
		sock_->setOnReceivedConnection(onReceivedConnection);
		sock_->setOnLoseConnection(onLoseConnection);
	}
	WebInterface::WebInterface(const std::string &name, const std::string &port, aris::core::Socket::TYPE type):Interface(name)
	{
		sock_ = &add<aris::core::Socket>("socket", "", port, type);
		
		sock_->setOnReceivedMsg(onReceivedMsg);
		sock_->setOnReceivedConnection(onReceivedConnection);
		sock_->setOnLoseConnection(onLoseConnection);
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
#include "mongoose.h"

#ifdef WIN32
#undef min
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////
namespace aris::server
{
	static struct mg_serve_http_opts s_http_server_opts;

	struct HttpInterface::Imp
	{
		std::thread http_thread_;
		std::string port_;
		std::string document_root_;
		
		struct mg_mgr mgr;
		struct mg_connection *nc;
		struct mg_bind_opts bind_opts;
		const char *err_str;

		std::mutex mu_running_;
		std::atomic_bool is_running_{ false };
	};
	auto HttpInterface::open()->void
	{
		std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
		
		if (imp_->is_running_.exchange(true) == false)
		{
			mg_mgr_init(&imp_->mgr, NULL);
			s_http_server_opts.document_root = imp_->document_root_.c_str();

			// Set HTTP server options //
			memset(&imp_->bind_opts, 0, sizeof(imp_->bind_opts));
			imp_->bind_opts.error_string = &imp_->err_str;

			imp_->nc = mg_bind_opt(&imp_->mgr, imp_->port_.c_str(), [](struct mg_connection *nc, int ev, void *ev_data)
			{
				struct http_message *hm = (struct http_message *) ev_data;

				switch (ev) {
				case MG_EV_HTTP_REQUEST:
				{
					std::cout << std::string(hm->uri.p, hm->uri.len) << std::endl;

					if (mg_vcmp(&hm->uri, "/api/config/interface") == 0)
					{
						mg_http_serve_file(nc, hm, "C:\\Users\\py033\\Desktop\\interface111.txt",
							mg_mk_str("text/plain; charset=utf-8"), mg_mk_str(""));
					}
					else 
					{
						mg_serve_http(nc, hm, s_http_server_opts);
					}
					break;
				}
				default:

					break;
				}
			}, imp_->bind_opts);
			if (imp_->nc == NULL) {
				//fprintf(stderr, "Error starting server on port %s: %s\n", s_http_port, *imp_->bind_opts.error_string);
				exit(1);
			}

			mg_set_protocol_http_websocket(imp_->nc);
			s_http_server_opts.enable_directory_listing = "yes";

			imp_->http_thread_ = std::thread([this]()
			{
				for (; imp_->is_running_; ) { mg_mgr_poll(&imp_->mgr, 1000); }
				mg_mgr_free(&imp_->mgr);
			});
		}
	}
	auto HttpInterface::close()->void 
	{
		std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
		
		if (imp_->is_running_.exchange(false) == true)
		{
			imp_->http_thread_.join();
		}
	}
	auto HttpInterface::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		Interface::saveXml(xml_ele);
		xml_ele.SetAttribute("document_root", imp_->document_root_.c_str());
		xml_ele.SetAttribute("port", imp_->port_.c_str());
	}
	auto HttpInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Interface::loadXml(xml_ele);
		imp_->document_root_ = Object::attributeString(xml_ele, "document_root", "./");
		imp_->port_ = Object::attributeString(xml_ele, "port", "8000");
	}
	HttpInterface::~HttpInterface() = default;
	HttpInterface::HttpInterface(HttpInterface && other) = default;
	HttpInterface& HttpInterface::operator=(HttpInterface&& other) = default;
	HttpInterface::HttpInterface(const std::string &name, const std::string &port, const std::string &document_root) :Interface(name), imp_(new Imp)
	{
		imp_->document_root_ = document_root;
		imp_->port_ = port;
	}
}