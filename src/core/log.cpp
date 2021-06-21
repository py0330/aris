#include <cstring>
#include <fstream>
#include <ctime>
#include <mutex>
#include <algorithm>
#include <iostream>
#include <atomic>
#include <map>
#include <sstream>


#ifdef UNIX
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>//for mkdir()
#include <dirent.h>
#include <sys/types.h>
#endif

#ifdef WIN32
#include <windows.h>
#undef min
#undef max
#endif

#include "aris/core/log.hpp"
#include "aris/core/basic_type.hpp"
#include "aris/core/msg.hpp"


namespace aris::core
{
	class ThreadSafeStreamBuf :public std::streambuf
	{
	public:
		virtual auto overflow(int_type c)->int_type override
		{
			msg_.resize(msg_.capacity());// 保证在下次resize的时候，所有数据都会被copy，这是因为在resize重新分配内存时，不是按照capacity来copy
			msg_.resize(msg_.capacity() + 1);
			setp(msg_.data() + msg_.size(), msg_.data() + msg_.capacity());
			*(pptr() - 1) = c;
			return c;
		}
		virtual auto sync()->int override
		{
			std::unique_lock<std::recursive_mutex> lck(*real_mutex_);

			msg_.resize(msg_.capacity() - static_cast<MsgSize>(epptr() - pptr()));
			**real_stream_ << msg_.toString() << std::flush;

			msg_.resize(0);
			setp(msg_.data(), msg_.data() + msg_.capacity());

			return 0;
		}
		explicit ThreadSafeStreamBuf(std::ostream** real_stream, std::recursive_mutex *real_mutex): real_stream_(real_stream), real_mutex_(real_mutex) { }

	private:
		aris::core::Msg msg_;
		std::ostream** real_stream_;
		std::recursive_mutex* real_mutex_;
	};
	template<int>
	class ThreadSafeStream :public std::ostream 
	{ 
	public: 
		static auto setStream(std::ostream& stream) ->void
		{ 
			std::unique_lock<std::recursive_mutex> lck(real_mutex_);
			real_stream_ = &stream; 
		}
		// 这个构造函数只有在 static 成员没有时，才会设置默认的ostream
		ThreadSafeStream(std::ostream &default_stream) :std::ostream(&buf_) 
		{
			std::unique_lock<std::recursive_mutex> lck(real_mutex_);
			real_stream_ = real_stream_ ? real_stream_ : &default_stream;
		}; 

		static std::recursive_mutex real_mutex_;
	private:
		ThreadSafeStreamBuf buf_{ &real_stream_, &real_mutex_ };
		static std::ostream* real_stream_;
	};
	template <int a>
	std::recursive_mutex ThreadSafeStream<a>::real_mutex_;
	template <int a>
	std::ostream* ThreadSafeStream<a>::real_stream_;

	auto cout()->std::ostream&
	{
		// 这个构造函数只有在 static 成员没有时，才会设置默认的ostream
		static thread_local ThreadSafeStream<0> local_stream_(std::cout);
		return local_stream_;
	}
	auto setCoutStream(std::ostream& cout_stream) { ThreadSafeStream<0>::setStream(cout_stream); }

	static std::filesystem::path log_dir_path_;
	static std::filesystem::path log_file_path_;
	static std::ofstream log_fstream_;
	static std::recursive_mutex log_file_mutex;
	static std::atomic_int max_info_num = 100000, current_info_num = 0;
	static int log_file_num = 0;

	auto log()->std::ostream&
	{
		// 这个参数仅仅用于设置默认值 //
		static thread_local ThreadSafeStream<1> local_stream_(log_fstream_);
		std::unique_lock<std::recursive_mutex> lck(ThreadSafeStream<1>::real_mutex_);

		// 如果记录满了，关闭文件，因为上面有锁，所以不可能出现内存stream在sync，而这里在关闭的情况 //
		if (++current_info_num > max_info_num) 
		{
			log_fstream_.close();
			current_info_num = 0;
			log_file_num++;
		}
		
		// 如果第一次或之前满了，打开文件 //
		if (!log_fstream_.is_open())
		{
			logFile();
			ThreadSafeStream<1>::setStream(log_fstream_);
		}

		// 返回memory stream //
		return local_stream_;
	}
	auto logDirectory(const std::filesystem::path &log_dir)->void
	{
		std::unique_lock<std::recursive_mutex> lck(ThreadSafeStream<1>::real_mutex_);

		log_dir_path_ = log_dir.empty() ? std::filesystem::absolute("log") : std::filesystem::absolute(log_dir);
		std::filesystem::create_directories(log_dir_path_);
	}
	auto logFile(const std::filesystem::path &log_file_path)->void
	{
		std::unique_lock<std::recursive_mutex> lck(ThreadSafeStream<1>::real_mutex_);

		log_file_path_ = log_file_path.empty() ? std::filesystem::path(logExeName() + "--" + logFileTimeFormat(std::chrono::system_clock::now()) + "--log.txt") : log_file_path;
		log_file_path_ = log_file_path_.has_root_path() ? log_file_path_ : logDirPath() / log_file_path_;
		log_file_path_.replace_filename(log_file_path_.filename().replace_extension().string() + std::string("_") + std::to_string(log_file_num) + log_file_path_.extension().string());
		std::filesystem::create_directories(log_file_path_.parent_path());

		log_fstream_.close();
		log_fstream_.open(log_file_path_, std::ios::out | std::ios::trunc);
	}
	auto logMaxInfoNum(int max_info_num_) { max_info_num.store(max_info_num_); }

	auto logDirPath()->std::filesystem::path 
	{ 
		if (log_dir_path_.empty())logDirectory();
		return log_dir_path_; 
	}
	auto logExeName()->std::string
	{
		const int TASK_NAME_LEN = 1024;

#ifdef WIN32
		char path[TASK_NAME_LEN] = { 0 };
		GetModuleFileName(NULL, path, TASK_NAME_LEN);

		char *p = strrchr(path, '\\');
		if (p == nullptr)
			THROW_FILE_LINE("windows can't identify the program name");

		char proName[TASK_NAME_LEN]{ 0 };

		char *dot = strrchr(path, '.');
		if ((dot != nullptr) && (dot - p > 0))
		{
			auto n = dot - p - 1;
			strncpy(proName, p + 1, n);
			proName[n] = 0;
		}
#endif

#ifdef UNIX
		int count = 0;
		int nIndex = 0;
		char path[TASK_NAME_LEN] = { 0 };
		char cParam[100] = { 0 };
		char *proName = path;

		pid_t pId = getpid();
		sprintf(cParam, "/proc/%d/exe", pId);
		count = readlink(cParam, path, TASK_NAME_LEN);

		if (count < 0 || count >= TASK_NAME_LEN)
		{
			THROW_FILE_LINE("Current System Not Surport Proc.\n");
		}
		else
		{
			nIndex = count - 1;

			for (; nIndex >= 0; nIndex--)
			{
				if (path[nIndex] == '/')
				{
					nIndex++;
					proName += nIndex;
					break;
				}
			}
		}
#endif
		return std::string(proName);

	}
	auto logFileTimeFormat(const std::chrono::system_clock::time_point &time)->std::string
	{
		auto time_t_var = std::chrono::system_clock::to_time_t(time);
		auto timeinfo = localtime(&time_t_var);
		char time_format[1024];
		strftime(time_format, 1024, "%Y-%m-%d--%H-%M-%S", timeinfo);
		return std::string(time_format);
	}

	auto ARIS_API dateFormat(const std::chrono::system_clock::time_point &time)->std::string {
		const auto &str = datetimeFormat(time);
		return str.substr(0, str.find_first_of(' '));
	}

	auto ARIS_API timeFormat(const std::chrono::system_clock::time_point &time)->std::string {
		const auto &str = datetimeFormat(time);
		return str.substr(str.find_first_of(' ')+1);
	}

	auto ARIS_API datetimeFormat(const std::chrono::system_clock::time_point &time)->std::string {
		auto tt = std::chrono::system_clock::to_time_t(time);

#ifdef UNIX
		std::tm tm;
		localtime_r(&tt, &tm);
#endif

#ifdef WIN32
		std::tm tm;
		localtime_s(&tm, &tt);
#endif

		std::ostringstream os;
		os << std::put_time(&tm, "%F %T");
		return os.str();
	}

	std::ostream& operator<<(std::ostream &s, const DbLogCell &cell) {
		s << std::flush;
		Sqlite3Log::instance().toDb(cell);
		return s;
	}

	Sqlite3Log::~Sqlite3Log() {
		to_work_ = false;
		if (worker_.joinable()) worker_.join();
		sqlite3_close(db_);
		db_ = nullptr;
	}

	auto Sqlite3Log::instance()->Sqlite3Log& {
		static Sqlite3Log obj;
		return obj;
	}

	auto Sqlite3Log::open(const std::filesystem::path &db_path)->void {
		if (isOpen()) throw std::runtime_error("Sqlite3 database has been already opened.");
	
		int ret = -1;
		char *msg = nullptr;

#define CHECK_OP_ERR(rc) if(rc) throw std::runtime_error(msg);

		try {
			// open database
			if (sqlite3_open(db_path.filename().string().c_str(), &db_) != SQLITE_OK) {
				throw std::runtime_error("Unable to open database file '"+db_path.filename().string()+"'.");
			}

			// check table
			bool has_tbl = false;
			auto tbl_check = [](void *context, int, char **, char **)->int {
				*(bool*)context = true;
				return 0;
			};
			CHECK_OP_ERR(sqlite3_exec(db_, "select * from sqlite_master where type='table' and name ='SYSLOG';", tbl_check, &has_tbl, &msg))

			if (!has_tbl) {
				const char *sql = "CREATE TABLE SYSLOG("
										"ID INTEGER PRIMARY KEY AUTOINCREMENT,"
										"TIME TIMESTAMP DEFAULT (datetime('now', 'localtime')),"
										"LEVEL INTEGER NOT NULL,"
										"TYPE INTEGER NOT NULL,"
										"CODE INTEGER DEFAULT 0,"
										"TEXT TEXT DEFAULT ''"
								   ");";
				CHECK_OP_ERR(sqlite3_exec(db_, sql, nullptr, nullptr, &msg))
			}

			// check column
			struct ColCheck {
				bool ok{true};
				std::map<std::string, std::pair<std::string, bool>> cols{
					{"ID", {"INTEGER", false}},
					{"TIME", {"TIMESTAMP", false}},
					{"LEVEL", {"INTEGER", false}},
					{"TYPE", {"INTEGER", false}},
					{"CODE", {"INTEGER", false}},
					{"TEXT", {"TEXT", false}},
				};

				auto isOk() const->bool {
					if (!ok) return ok;
					for (const auto &[name, tpair] : cols) {
						if (tpair.second == false) return false;
					}
					return true;
				}
			};
			auto col_check = [](void *context, int count, char **value, char **name)->int {
				std::string cname;
				std::string ctype;
				for (int i=0;i<count;++i) {
					if (!strcmp(name[i], "name")) cname = value[i];
            		if (!strcmp(name[i], "type")) ctype = value[i];
				}

				auto cell = static_cast<ColCheck*>(context);
				auto iter = cell->cols.find(cname);
				if (iter == cell->cols.end() || iter->second.first != ctype) {
					cell->ok = false;
				} else {
					iter->second.second = true;
				}

				return 0;
			};
			ColCheck col_check_cell;
			CHECK_OP_ERR(sqlite3_exec(db_, "PRAGMA table_info(SYSLOG)", col_check, &col_check_cell, &msg))
			if (!col_check_cell.isOk()) throw std::runtime_error("Incompitible columns.");

			to_work_ = true;
			worker_ = std::thread([this]() {
				while(this->to_work_) {
					if (this->queue_->isEmpty()) {
						std::this_thread::sleep_for(std::chrono::milliseconds(200));
						continue;
					}

					DbLogCell cell;
					this->queue_->pop(cell, AccessStrategy::kYield);

					char *msg{nullptr};
					std::string sql = "INSERT INTO SYSLOG (TIME, LEVEL, TYPE, CODE, TEXT) VALUES(";
					sql += "'"+datetimeFormat(cell.timeStamp())+"',";
					sql += std::to_string((int)cell.logLvl())+",";
					sql += std::to_string((int)cell.logType())+",";
					sql += std::to_string(cell.code())+",";
					sql += "'"+cell.text() + "');";
					this->mu_.lock();
					if(sqlite3_exec(this->db_, sql.c_str(), nullptr, nullptr, &msg)) {
						std::cout<<msg<<std::endl;
					}
					this->mu_.unlock();
					sqlite3_free(msg);
				}
			});
		} catch (const std::exception &e) {
			sqlite3_free(msg);
			sqlite3_close(db_);
			db_ = nullptr;
			throw std::runtime_error(e.what());
		}		
#undef CHECK_OP_ERR
	}

	auto Sqlite3Log::toDb(const DbLogCell &cell)->void {
		if (!isOpen()) throw std::runtime_error("Database has not been opened.");
		queue_->push(cell, cell.strategy());
	}

	auto Sqlite3Log::close()->void {
		if (!isOpen()) return;
		to_work_ = false;
		worker_.join();
		sqlite3_close(db_);
		db_ = nullptr;
		queue_->clear();
	}

	auto Sqlite3Log::isOpen() const->bool { return db_ != nullptr; }

	auto Sqlite3Log::select(const LogFilter &filter) const->std::vector<DbLogCell> {
		if (!isOpen()) throw std::runtime_error("Database has not been opened.");

		std::string sql = "SELECT * FROM SYSLOG";

		std::vector<std::string> rules;

		// time rule
		if (filter.timeRule().enable) {
			const auto &rule = filter.timeRule();
			if (rule.begin > rule.end) return {};
			rules.push_back("TIME BETWEEN '"+datetimeFormat(rule.begin)+"' AND '"+datetimeFormat(rule.end)+"'");
		}

		// lvl rule
		if (filter.lvlRule().enable) {
			const auto &rule = filter.lvlRule();
			if (rule.lvls.empty()) return {};

			std::string rule_str;
			for (const auto &lvl : rule.lvls) {
				rule_str += "LEVEL = "+std::to_string((int)(lvl))+" OR ";
			}
			rule_str.erase(rule_str.size()-4);
			rules.push_back(rule_str);
		}

		// type rule
		if (filter.typeRule().enable) {
			const auto &rule = filter.typeRule();
			if (rule.types.empty()) return {};

			std::string rule_str;
			for (const auto &type : rule.types) {
				rule_str += "TYPE = "+std::to_string((int)(type))+" OR ";
			}
			rule_str.erase(rule_str.size()-4);
			rules.push_back(rule_str);
		}

		// code rule
		if (filter.codeRule().enable) {
			const auto &rule = filter.codeRule();
			if (rule.min_code > rule.max_code) return {};
			rules.push_back("CODE BETWEEN "+std::to_string(rule.min_code)+" AND "+std::to_string(rule.max_code));
		}

		if (!rules.empty()) {
			sql += " WHERE ";
			for (const auto &rule : rules) sql += "(" + rule + ") AND ";
			sql.erase(sql.size()-5);
		}

		// order
		if (filter.order()) {
			sql += " ORDER BY ID ASC";
		} else {
			sql += " ORDER BY ID DESC";
		}

		// count rule
		if (filter.countRule().enable) {
			const auto &rule = filter.countRule();
			if (rule.offset == 0) return {};

			sql += " LIMIT "+std::to_string(rule.start)+","+std::to_string(rule.offset);
 		}

		sql += ";";

		std::vector<DbLogCell> ret;
		auto ret_construct = [](void *context, int count, char **value, char **name)->int {
			auto &ret = *static_cast<std::vector<DbLogCell>*>(context);

			DbLogCell cell;
			for (int i=0; i<count; ++i) {
				if (!strcmp(name[i], "TIME")) {
					std::tm t={};
					std::istringstream is(value[i]);
					is.imbue(std::locale("de_DE.utf-8"));
					is >> std::get_time(&t, "%Y-%m-%d %H:%M:%S");
					cell.setTimeStamp(std::chrono::system_clock::from_time_t(std::mktime(&t)));
				} else if (!strcmp(name[i], "LEVEL")) {
					cell.setLogLvl((LogLvl)(atoi(value[i])));
				} else if (!strcmp(name[i], "TYPE")) {
					cell.setLogType((LogType)(atoi(value[i])));
				} else if (!strcmp(name[i], "CODE")) {
					cell.setCode(atoi(value[i]));
				} else if (!strcmp(name[i], "TEXT")) {
					cell.setText(std::string(value[i]));
				}
			}

			ret.push_back(cell);

			return 0;
		};

		char *msg = nullptr;
		mu_.lock();
		if (sqlite3_exec(db_, sql.c_str(), ret_construct, &ret, &msg)) {
			std::string str_msg(msg);
			sqlite3_free(msg);
			mu_.unlock();
			throw std::runtime_error(str_msg);
		}
		mu_.unlock();

		return ret;

	}

	// auto CodeTextTable::getAndFormat(int code, int language, ...) const->std::string {
	// 	va_list va;
	// 	va_start(va, language);
	// 	char buf[1024];
	// 	vsnprintf(buf, 1023, get(code, language).c_str(), va);
	// 	va_end(va);

	// 	return std::string(buf);
	// }

	unsigned LocaleString::locale_{CHN};
}