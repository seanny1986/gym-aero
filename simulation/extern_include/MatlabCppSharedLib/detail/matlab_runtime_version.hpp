/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_RUNTIME_VERSION_HPP
#define MATLAB_RUNTIME_VERSION_HPP

#ifdef _WIN32

const wchar_t * const MATLAB_CPP_SHARED_LIB_BASE_NAME = L"libMatlabCppSharedLib";
const wchar_t * const MATLAB_CPP_SHARED_LIB_SUFFIX = L"9_6.dll";

#endif // _WIN32

#endif // MATLAB_RUNTIME_VERSION_HPP
