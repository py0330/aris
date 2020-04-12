#include "mongoose.h"
#include <string>
#include <cstring>

int main(int argc, char *argv[])
{
#if(WIN32)
	auto default_address = "C:/aris/aris-1.5.0/doc/html";
#endif
#if(UNIX)
	auto default_address = "/usr/aris/aris-1.5.0/doc/html";
#endif

	std::string document_root = argc < 2 ? default_address : argv[1];
	std::string port          = argc < 3 ? "5870" : argv[2];

	struct mg_mgr mgr;
	struct mg_connection *nc;
	struct mg_bind_opts bind_opts;
	struct mg_serve_http_opts s_http_server_opts;
	const char *err_str;

	mg_mgr_init(&mgr, NULL);

	// Set HTTP server options //
	memset(&bind_opts, 0, sizeof(bind_opts));
	bind_opts.error_string = &err_str;

	nc = mg_bind_opt(&mgr, port.c_str(), [](struct mg_connection *nc, int ev, void *ev_data)
	{
		struct http_message *hm = (struct http_message *) ev_data;

		switch (ev) {
		case MG_EV_HTTP_REQUEST:
		{
			mg_serve_http(nc, hm, *reinterpret_cast<mg_serve_http_opts*>(nc->user_data));
		}
		default:

			break;
		}
	}, bind_opts);
	if (nc == NULL) {
		fprintf(stderr, "Error starting server on http\n");
		exit(1);
	}

	mg_set_protocol_http_websocket(nc);
	std::memset(&s_http_server_opts, 0, sizeof(mg_serve_http_opts));
	s_http_server_opts.document_root = document_root.c_str();
	s_http_server_opts.enable_directory_listing = "yes";
	nc->user_data = &s_http_server_opts;

	for (;;) { mg_mgr_poll(&mgr, 1000); }
	mg_mgr_free(&mgr);


	return 0;
}