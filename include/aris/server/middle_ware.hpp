#ifndef ARIS_SERVER_MIDDLE_WARE_HPP_
#define ARIS_SERVER_MIDDLE_WARE_HPP_

#include <string_view>
#include <functional>

#include <aris_lib_export.h>

namespace aris::server {

class ARIS_API MiddleWare {
public:
    MiddleWare() {}
    virtual ~MiddleWare() {}

    auto virtual executeCmd(std::string_view str, std::function<void(std::string)> send_ret)->int;     // 默认实现仅转发指令到控制器
};


}   // namespace aris::server

#endif  // ARIS_SERVER_MIDDLE_WARE_HPP_