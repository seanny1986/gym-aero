/* Copyright 2016-2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_STRING_HPP
#define MATLAB_DATA_STRING_HPP

#include <string>
#include "Optional.hpp"

namespace matlab {
    namespace data {
        using String = std::basic_string<char16_t>;

        using MATLABString = optional<String>;
   }
}


#endif
