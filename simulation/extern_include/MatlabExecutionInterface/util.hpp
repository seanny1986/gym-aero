/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLABEXECUTIONINTERFACE_UTIL_HPP
#define MATLABEXECUTIONINTERFACE_UTIL_HPP

#include <vector>
#include <streambuf>
#include <memory>

namespace detail {
    enum class ExceptionType {
        CANCELLED = 0,
        INTERRUPTED = 1,
        EXECUTION = 2,
        SYNTAX = 3,
        OTHER = 4,
        STOPPED = 5
    };

    enum class ConvertUTFStringExceptionType {
        CONVERT_ERROR = 1,
        OUT_OF_MEMORY = 2
    };
}

namespace matlab {
    namespace execution {

        typedef std::basic_streambuf<char16_t> StreamBuffer;

        std::u16string convertUTF8StringToUTF16String(const std::string &utf8string);
        std::string convertUTF16StringToUTF8String(const std::u16string &utf16string);
    }
}


#endif //MATLABEXECUTIONINTERFACE_UTIL_HPP