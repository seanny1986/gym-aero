#ifndef SLEXEC_VM_RT_FUNCTIONS_H
#define SLEXEC_VM_RT_FUNCTIONS_H
/**
 * @file slexec_vm_rt_functions.h
 *
 * This header is the exported C interface of the Simulink RT functions
 * for use in VM applications.
 * 
 */

/* Copyright 2015 The MathWorks, Inc. */

#include "tmwtypes.h"

#ifdef BUILDING_SLEXEC_SIMBRIDGE
 /* being included from inside slexec_simbridge module */
  #include "package.h"
  #define SLEXEC_SIMBRIDGE_PUBLISHED_C extern "C" DLL_EXPORT_SYM
#else
 /* being included from outside, such as raccel code */
  #if defined(__cplusplus)
 /* Needed for non-inlined C++ S-functions */
    #define SLEXEC_SIMBRIDGE_PUBLISHED_C extern "C"
  #else 
    #define SLEXEC_SIMBRIDGE_PUBLISHED_C extern
  #endif
#endif

SLEXEC_SIMBRIDGE_PUBLISHED_C void rtErrorMsgID(
    void*  S, const char* msgID);

SLEXEC_SIMBRIDGE_PUBLISHED_C void rtSizeEqCheck1D(
    void*  S, int  dims1, int dims2);

SLEXEC_SIMBRIDGE_PUBLISHED_C void rtSizeEqCheckND(
    void*  S, int* dims1, int* dims2, int  nDims);

#endif
