// Copyright 2007-2012 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_API_UTIL_HPP
#define SFUN_API_UTIL_HPP

#include "../libmwcgir_construct.hpp"
#include <stdexcept>
#include <string>
#include "rtw_api_lint_begin.hpp"

namespace RTW
{
    extern "C"
    {
        CGIR_CONSTRUCT_API const char* SFun_getLastError();
        CGIR_CONSTRUCT_API void SFun_setLastError_c(const char* message);
    }

    inline void setErrorInfoFromActiveException()
    {
        try {
            throw;
        } catch (std::exception& e) {
            SFun_setLastError_c(e.what());
        } catch (...) {
            SFun_setLastError_c("Unspecified error");
        }
    }

    inline void raiseLastError()
    {
        std::string msg = std::string(SFun_getLastError());
        SFun_setLastError_c(NULL);
        throw std::runtime_error(msg);
    }

    inline void verify(int result)
    {
        if (!result) {
            raiseLastError();
        }
    }

    inline void verify(void* result)
    {
        verify(result != 0);
    }
}

#include "rtw_api_lint_end.hpp"

#endif
