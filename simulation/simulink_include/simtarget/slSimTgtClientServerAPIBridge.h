/* Copyright 2015-2017 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTCLIENTSERVERAPIBRIDGE_H
#define SLSIMTGTCLIENTSERVERAPIBRIDGE_H

#ifdef __cplusplus
#include <string>
#include <vector>
#endif

#include "sl_simtarget_core_spec.h"
#include "slClientServerAPIBridge_types.h"

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slcsInvokeSimulinkFunction(
    SimStruct* S, const char * fcnName, _ssFcnCallExecArgInfo *args);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slcsSetSimulinkFunctionPtr(
    SimStruct* S, const char * fcnName, SimulinkFunctionPtr fPtr);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slcsInitFcnCallInfo(_ssFcnCallInfo   *info,
                                                  int_T             numInArgs,
                                                  int_T             numOutArgs,
                                                  _ssFcnCallArgInfo *inArgs,
                                                  _ssFcnCallArgInfo *outArgs);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slcsRegisterSimulinkFunction(
    SimStruct* S, const char * fcnName, SimulinkFunctionPtr fcnPtr,
    _ssFcnCallInfo *callInfo, const char * fullPathToFunction, bool isGlobal);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slcsInvokeSimulinkFunctionVoidArgs(
    SimStruct* S, const char* fcnName, int_T numArgs, void* args[]);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C  void slcsRegisterCallerBlock(
    SimStruct* S, const char * fcnName, _ssFcnCallInfo *callInfo, const char * addlFullPath);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slcsUpdateServerSFcnCatalog(
    SimStruct* S, const char * fcnName, void * fPtr);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C bool slcsIsFunctionRegistered(
    SimStruct* S, const char * fcnName);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void* slcsGetInputArgFromExecInfo(_ssFcnCallExecArgs* args, int i);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void* slcsGetOutputArgFromExecInfo(_ssFcnCallExecArgs* args, int i);

#endif
