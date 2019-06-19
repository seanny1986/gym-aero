/* Copyright 2017 The MathWorks, Inc. */

#ifndef CPP_ENGINE_API_UTIL_HPP
#define CPP_ENGINE_API_UTIL_HPP

#ifdef _MSC_VER
#define ENGINE_DLL_EXPORT_SYM __declspec(dllexport)
#define ENGINE_DLL_IMPORT_SYM __declspec(dllimport)
#elif __GNUC__ >= 4
#define ENGINE_DLL_EXPORT_SYM __attribute__((visibility("default")))
#define ENGINE_DLL_IMPORT_SYM __attribute__((visibility("default")))
#else
#define ENGINE_DLL_EXPORT_SYM
#define ENGINE_DLL_IMPORT_SYM
#endif

#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C extern
#endif

#if defined(BUILDING_CPP_ENGINE)
#define CPP_ENGINE_API ENGINE_DLL_EXPORT_SYM
#define CPP_ENGINE_C_API EXTERN_C ENGINE_DLL_EXPORT_SYM
#else
#define CPP_ENGINE_API ENGINE_DLL_IMPORT_SYM
#define CPP_ENGINE_C_API EXTERN_C ENGINE_DLL_IMPORT_SYM

#endif

#endif /* CPP_ENGINE_API_UTIL_HPP */