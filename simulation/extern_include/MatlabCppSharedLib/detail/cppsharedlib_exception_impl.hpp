/* Copyright 2017 The MathWorks, Inc. */

#ifndef CPPSHAREDLIB_EXCEPTION_IMPL_HPP
#define CPPSHAREDLIB_EXCEPTION_IMPL_HPP

#include <vector>
#include <streambuf>
#include <memory>
#include <future>
#include "../cppsharedlib_exception.hpp"
#include <string>

#if defined(_WIN32 ) 
#define NOEXCEPT throw() 
#else
#define NOEXCEPT noexcept
#endif


namespace matlab {
    namespace cpplib { 


        inline CppSharedLibException::CppSharedLibException() {}

        inline CppSharedLibException::CppSharedLibException(const std::string& msg) : matlab::execution::Exception(msg) {}

    }
}


#endif /* CPPSHAREDLIB_EXCEPTION_IMPL_HPP */