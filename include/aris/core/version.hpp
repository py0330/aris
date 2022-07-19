#ifndef ARIS_CORE_VERSION_H_
#define ARIS_CORE_VERSION_H_

#include <string>
#include <utility>

#include "aris_lib_export.h"

namespace aris::core {
    auto ARIS_API version()->std::pair<std::string, std::string>;
    
}   // namespace aris::core


#endif  // ARIS_CORE_VERSION_H_