/* Copyright 2013 The MathWorks, Inc. */

#ifndef _sf_runtime_utils_h_
#define _sf_runtime_utils_h_

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#include "mex.h"
#include "sf_runtime_spec.h"
#include "sf_runtime/sf_runtime_types.h"

#if !defined(S_FUNCTION_NAME)
#include "simstruct/simstruc.h"
#else
#include "simstruc.h"
#endif

#define MATLAB_CLASSES(X)                       \
    X(LOGICAL,boolean_T,int)                    \
    X(CHAR,char_T,int)                          \
    X(DOUBLE,real_T,double)                     \
    X(SINGLE,real32_T,double)                   \
    X(INT8,int8_T,int)                          \
    X(UINT8,uint8_T,int)                        \
    X(INT16,int16_T,int)                        \
    X(UINT16,uint16_T,int)                      \
    X(INT32,int32_T,int)                        \
    X(UINT32,uint32_T,unsigned int)

typedef enum
{
#define DEFINE_ENUM(NAME,CTYPE,CTYPE2) ML_##NAME = mx##NAME##_CLASS,
    MATLAB_CLASSES(DEFINE_ENUM)
#undef DEFINE_ENUM
    ML_MX_ARRAY,
    ML_STRING,
    ML_VOID,
} MatlabDataType;

#ifdef __cplusplus
#define MatlabWorkspaceType int
#define BASE_WORKSPACE   0
#define CALLER_WORKSPACE 1
#define GLOBAL_WORKSPACE 2
#define ALL_WORKSPACES   3
#else
typedef enum
{
    BASE_WORKSPACE,
    CALLER_WORKSPACE,
    GLOBAL_WORKSPACE,
    ALL_WORKSPACES
} MatlabWorkspaceType;
#endif

LIBMWSF_RUNTIME_API void sf_mex_set_error_status(SimStruct* S, const char* fmt, ...);
LIBMWSF_RUNTIME_API  bool mx_array_has_fi_type(const mxArray* mxArrayPtr);

LIBMWSF_RUNTIME_API const mxArray* sf_mex_get_ml_var_no_error(const char* variableName, MatlabWorkspaceType workspaceType);
LIBMWSF_RUNTIME_API const mxArray* sf_mex_get_ml_var(const char* variableName, MatlabWorkspaceType workspaceType);

LIBMWSF_RUNTIME_API void sf_mex_printf(const char* msg, ...);
LIBMWSF_RUNTIME_API bool sf_mex_mx_is_numeric(const mxArray* mxArrayPtr);

LIBMWSF_RUNTIME_API void initDynamicStringInSF(void* outputs);
LIBMWSF_RUNTIME_API void checkStringBufferOverflowDiagnostic(int diagnosticValue, SimStruct* S,
                                                             unsigned int dataNumber, unsigned int ssid,
                                                             int offsetSrcLoc, int lengthSrcLoc);
LIBMWSF_RUNTIME_API void stringFcnNumericArgumentDiagnostic(SimStruct* S, int limit, int argIndex, unsigned int ssid,
                                        int offsetSrcLoc, int lengthSrcLoc);

LIBMWSF_RUNTIME_API int sf_mex_call_matlab(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* fcnName);

LIBMWSF_RUNTIME_API mxArray* sf_load_rtw_optimization_info(const char* machineName,
                                                           const char* mainMachineName);

#endif
