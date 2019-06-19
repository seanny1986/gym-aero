/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_GET_RETURN_TYPE_HPP_
#define MATLAB_DATA_GET_RETURN_TYPE_HPP_

#include "String.hpp"
#include "Optional.hpp"

#include <string>

namespace matlab {
    namespace data {

        template<typename T> struct GetReturnType {
            typedef T type;
        };

        template<> struct GetReturnType<String> {
            typedef MATLABString type;
        };

        template<> struct GetReturnType<std::string> {
            typedef MATLABString type;
        };

        template<> struct GetReturnType<const char*> {
            typedef MATLABString type;
        };

        template<> struct GetReturnType<const char16_t*> {
            typedef MATLABString type;
        };

        template<> struct GetReturnType<char> {
            typedef CHAR16_T type;
        };

    }
}

#endif
