#include "aris/core/version.hpp"

#include "aris_version.h"

#define __S(x) #x
#define _S(x) __S(x)

namespace aris::core {

auto version()->std::pair<std::string, std::string> {
    auto ver = std::string("v")+_S(ARIS_VERSION_MAJOR)+"."+_S(ARIS_VERSION_MINOR)+"."+_S(ARIS_VERSION_PATCH)+"."+_S(ARIS_VERSION_TIME);
    auto des = std::string(_S(ARIS_DESCRIPTION));
    return {ver, des};
}

}