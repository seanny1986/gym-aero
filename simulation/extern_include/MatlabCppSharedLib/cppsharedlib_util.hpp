/* Copyright 2017 The MathWorks, Inc. */

#ifndef CPPSHAREDLIB_UTIL_HPP
#define CPPSHAREDLIB_UTIL_HPP

#include "cppsharedlib_api.hpp"

#include <mutex>
#include <algorithm>
#include <ratio>
#include <cstring>

namespace matlab {
    namespace cpplib {

        class MATLABLibrary;

        enum class MATLABApplicationMode {
            OUT_OF_PROCESS = 0,
            IN_PROCESS = 1
        };
    }
}

#endif //CPPSHAREDLIB_UTIL_HPP