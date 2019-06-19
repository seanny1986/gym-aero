#ifdef SUPPORTS_PRAGMA_ONCE
  #pragma once
#endif

/* Copyright 2015-2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_PUBLISH_UTIL_HPP
#define MATLAB_EXTDATA_PUBLISH_UTIL_HPP

#ifdef _MSC_VER
  #define MATLAB_EXTDATA_DLL_EXPORT_SYM __declspec(dllexport)
  #define MATLAB_EXTDATA_DLL_IMPORT_SYM __declspec(dllimport)
#elif __GNUC__ >= 4
  #define MATLAB_EXTDATA_DLL_EXPORT_SYM __attribute__ ((visibility("default")))
  #define MATLAB_EXTDATA_DLL_IMPORT_SYM __attribute__ ((visibility("default")))
#else
  #define MATLAB_EXTDATA_DLL_EXPORT_SYM
  #define MATLAB_EXTDATA_DLL_IMPORT_SYM
#endif

#if defined(BUILDING_MDA_PUBLISH_API)
  #define MATLAB_EXTDATA_PUBLISHED_API MATLAB_EXTDATA_DLL_EXPORT_SYM
#else
  #define MATLAB_EXTDATA_PUBLISHED_API MATLAB_EXTDATA_DLL_IMPORT_SYM
#endif

#define MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C extern "C" MATLAB_EXTDATA_PUBLISHED_API

#if defined(_MSC_FULL_VER) && _MSC_FULL_VER < 190023026
  #define MW_NOEXCEPT
#else
  #define MW_NOEXCEPT noexcept
#endif

#endif
