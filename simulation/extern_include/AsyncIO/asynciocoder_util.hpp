/**
 * @file    asynciocoder_util.hpp
 * @details export/import switch
 * @Copyright 2018 The MathWorks, Inc.
 */
#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef _ASYNCIOCODER_UTIL_HPP_
#define _ASYNCIOCODER_UTIL_HPP_

#ifdef BUILDING_ASYNCIOCODER
/* For DLL_EXPORT_SYM and EXTERN_C */
#include "package.h"
/* For size_t, boolean_T, etc */
#include "tmwtypes.h"

#define ASYNCIOCODER_API DLL_EXPORT_SYM
#else
/*
DLL_IMPORT_SYM is not needed because it's a C API that only uses EXTERN_C
#define ASYNCIOCODER_API  DLL_IMPORT_SYM
*/

/* For size_t, boolean_T, etc */
#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifndef EXTERN_C
#ifdef __cplusplus
#define EXTERN_C extern "C"
#else
#define EXTERN_C extern
#endif
#endif

#ifndef ASYNCIOCODER_API
#define ASYNCIOCODER_API
#endif

#endif

#endif
