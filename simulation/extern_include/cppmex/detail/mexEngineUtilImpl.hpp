/* Copyright 2017 The MathWorks, Inc. */

#ifndef __MEX_ENGINE_UTIL_IMPL_HPP__
#define __MEX_ENGINE_UTIL_IMPL_HPP__

#include <string>
#include <iostream>
#include <memory>
#include "MatlabDataArray.hpp"

EXTERN_C void mex_engine_utf8_to_utf16(const char* utf8, char16_t** utf16, size_t* errType);
EXTERN_C void mex_engine_utf16_to_utf8(const char16_t* utf16, char** utf8, size_t* errType);
EXTERN_C void mex_engine_destroy_utf8(char* utf8);
EXTERN_C void mex_engine_destroy_utf16(char16_t* utf16);

namespace matlab {
    namespace engine {
        namespace detail {
            enum class ConvertUTFStringExceptionType {
                CONVERT_ERROR = 1,
                OUT_OF_MEMORY = 2
            };
        }

        /*** matlab::engine definitions ***/
        inline std::basic_string<char16_t> convertUTF8StringToUTF16String(const std::string &utf8string) {
            char16_t* name = nullptr;
            size_t errType = 0;
            mex_engine_utf8_to_utf16(utf8string.c_str(), &name, &errType);
            if (errType > 0) {
                matlab::engine::detail::ConvertUTFStringExceptionType excType = static_cast<matlab::engine::detail::ConvertUTFStringExceptionType>(errType);
                //it could be either out of memory
                if (excType == matlab::engine::detail::ConvertUTFStringExceptionType::OUT_OF_MEMORY) throw(matlab::OutOfMemoryException("Not enough memory available to support the request."));
                // or std::range_error during the conversion
                if (excType == matlab::engine::detail::ConvertUTFStringExceptionType::CONVERT_ERROR) throw(TypeConversionException("The input cannot be converted to the requested type."));
            }
            std::unique_ptr<char16_t, void(*)(char16_t*)> guard(name, &mex_engine_destroy_utf16);
            std::u16string uName(name);
            return uName;
        }

        inline std::string convertUTF16StringToUTF8String(const std::basic_string<char16_t> &utf16string) {
            char* name = nullptr;
            size_t errType = 0;
            mex_engine_utf16_to_utf8(utf16string.c_str(), &name, &errType);
            if (errType > 0) {
                matlab::engine::detail::ConvertUTFStringExceptionType excType = static_cast<matlab::engine::detail::ConvertUTFStringExceptionType>(errType);
                //it could be either out of memory
                if (excType == matlab::engine::detail::ConvertUTFStringExceptionType::OUT_OF_MEMORY) throw(matlab::OutOfMemoryException("Not enough memory available to support the request."));
                // or std::range_error during the conversion
                if (excType == matlab::engine::detail::ConvertUTFStringExceptionType::CONVERT_ERROR) throw(TypeConversionException("The input cannot be converted to the requested type."));
            }
            std::unique_ptr<char, void(*)(char*)> guard(name, &mex_engine_destroy_utf8);
            std::string sName(name);
            return sName;
        }
    }
}

#endif
