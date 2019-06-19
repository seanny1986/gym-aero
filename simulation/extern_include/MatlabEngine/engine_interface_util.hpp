/* Copyright 2017 The MathWorks, Inc. */

#ifndef INTERFACE_UTIL_HPP
#define INTERFACE_UTIL_HPP
#include "cpp_engine_api_util.hpp"
#include <MatlabExecutionInterface/util.hpp>
#include <MatlabExecutionInterface/exception.hpp>
#include <locale>
#include <string>
#ifdef _MSC_VER
#include <codecvt>
#endif

CPP_ENGINE_C_API void util_utf8_to_utf16(const char* utf8, char16_t** utf16, size_t* errType);

CPP_ENGINE_C_API void util_utf16_to_utf8(const char16_t* utf16, char** utf8, size_t* errType);

CPP_ENGINE_C_API void util_destroy_utf8(char* utf8);

CPP_ENGINE_C_API void util_destroy_utf16(char16_t* utf16);

namespace matlab {

    namespace execution {

        inline std::basic_string<char16_t> convertUTF8StringToUTF16String(const std::string &utf8string) {
            char16_t* name = nullptr;
            size_t errType = 0;
            util_utf8_to_utf16(utf8string.c_str(), &name, &errType);
            std::unique_ptr<char16_t, void(*)(char16_t*)> guard(name, util_destroy_utf16);
            if (errType > 0) {
                ::detail::ConvertUTFStringExceptionType excType = static_cast<::detail::ConvertUTFStringExceptionType>(errType);
                //it could be either out of memory
                if (excType == ::detail::ConvertUTFStringExceptionType::OUT_OF_MEMORY) throw(matlab::OutOfMemoryException("Not enough memory available to support the request."));
                // or std::range_error during the conversion
                if (excType ==  ::detail::ConvertUTFStringExceptionType::CONVERT_ERROR) throw(TypeConversionException("The input cannot be converted to the requested type."));
            }
            std::u16string uName(name);
            return uName;
        }

        inline std::string convertUTF16StringToUTF8String(const std::basic_string<char16_t> &utf16string) {
            char* name = nullptr;
            size_t errType = 0;
            util_utf16_to_utf8(utf16string.c_str(), &name, &errType);
            std::unique_ptr<char, void(*)(char*)> guard(name, util_destroy_utf8);
            if (errType > 0) {
                ::detail::ConvertUTFStringExceptionType excType = static_cast<::detail::ConvertUTFStringExceptionType>(errType);
                //it could be either out of memory
                if (excType == ::detail::ConvertUTFStringExceptionType::OUT_OF_MEMORY) throw(matlab::OutOfMemoryException("Not enough memory available to support the request."));
                // or std::range_error during the conversion
                if (excType == ::detail::ConvertUTFStringExceptionType::CONVERT_ERROR) throw(TypeConversionException("The input cannot be converted to the requested type."));
            }
            std::string sName(name);
            return sName;
        }
    }
}

#endif // INTERFACE_UTIL_HPP