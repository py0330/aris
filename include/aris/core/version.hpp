#ifndef ARIS_CORE_VERSION_H_
#define ARIS_CORE_VERSION_H_

#include <map>

namespace aris::core {

    // 返回值 <版本号, 版本描述>
    auto version()->std::pair<std::string, std::string>;

}   // namespace aris::core

#endif  // ARIS_CORE_VERSION_H_
