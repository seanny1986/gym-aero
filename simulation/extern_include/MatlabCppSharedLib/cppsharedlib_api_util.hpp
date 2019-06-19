/* Copyright 2017 The MathWorks, Inc. */

#ifndef CPPSHAREDLIB_API_UTIL_HPP
#define CPPSHAREDLIB_API_UTIL_HPP

#ifdef _MSC_VER
#define RUNTIME_DLL_EXPORT_SYM __declspec(dllexport)
#define RUNTIME_DLL_IMPORT_SYM __declspec(dllimport)
#elif __GNUC__ >= 4
#define RUNTIME_DLL_EXPORT_SYM __attribute__((visibility("default")))
#define RUNTIME_DLL_IMPORT_SYM __attribute__((visibility("default")))
#else
#define RUNTIME_DLL_EXPORT_SYM
#define RUNTIME_DLL_IMPORT_SYM
#endif

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C extern
#endif

#if defined(BUILDING_RUNTIME_PROXY)
#define CPP_RUNTIME_API RUNTIME_DLL_EXPORT_SYM
#define CPP_RUNTIME_C_API EXTERN_C RUNTIME_DLL_EXPORT_SYM
#else
#define CPP_RUNTIME_API RUNTIME_DLL_IMPORT_SYM
#define CPP_RUNTIME_C_API EXTERN_C RUNTIME_DLL_IMPORT_SYM

#endif

#endif // CPPSHAREDLIB_API_UTIL_HPP 