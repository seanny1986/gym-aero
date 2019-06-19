/* Copyright 2013-2017 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLCLIENTSERVERAPIBRIDGE_H
#define SLCLIENTSERVERAPIBRIDGE_H

#ifdef __cplusplus
#include <string>
#include <vector>
#endif

#include "simulink_spec.h"
#include "slClientServerAPIBridge_types.h"

namespace mdlref_simtarget {

SIMULINK_EXPORT_FCN void slcsInvokeSimulinkFunction(
    SimStruct* S, const char * fcnName, _ssFcnCallExecArgInfo *args);

SIMULINK_EXPORT_FCN void slcsSetSimulinkFunctionPtr(
    SimStruct* S, const char * fcnName, SimulinkFunctionPtr fPtr);

SIMULINK_EXPORT_FCN void slcsInitFcnCallInfo(_ssFcnCallInfo   *info,
                                             int_T             numInArgs,
                                             int_T             numOutArgs,
                                             _ssFcnCallArgInfo *inArgs,
                                             _ssFcnCallArgInfo *outArgs);

SIMULINK_EXPORT_FCN _ssFcnCallExecArgs *slcsCreateFcnCallExecArgs(int_T numInArgs,
                                                                       int_T numOutArgs);

SIMULINK_EXPORT_FCN void slcsDestroyFcnCallExecArgs(_ssFcnCallExecArgs *execArgs);

SIMULINK_EXPORT_FCN void slcsSetCanBeInvokedConcurrently(_ssFcnCallInfo *info, boolean_T val);
SIMULINK_EXPORT_FCN void slcsSetCallerBlockPaths(_ssFcnCallInfo *info,
                                                 int_T           nCallers,
                                                 const char    **callerBlockPaths);
SIMULINK_EXPORT_FCN void slcsSetReusedInOutArgs(_ssFcnCallInfo *info,
                                                int_T  nInOutArgs,
                                                int_T *inOutArgs);

SIMULINK_EXPORT_FCN void slcsSetCallerTIDs(_ssFcnCallInfo *info,
                                           int_T nCallerTIDs,
                                           int_T *callerTIDs);

SIMULINK_EXPORT_FCN void slcsSetSymbolicDims(_ssFcnCallInfo *info,
                                             int_T *arginSymbDims,
                                             int_T *argoutSymbDims);

SIMULINK_EXPORT_FCN void slcsSetArgumentIndices(_ssFcnCallInfo *info,
                                                const char **argNames,
                                                const char **argIndices);

SIMULINK_EXPORT_FCN void slcsSetReturnArgIndex(_ssFcnCallInfo *info,
                                               int_T returnArgIndex);

SIMULINK_EXPORT_FCN void slcsSetReturnArgName(_ssFcnCallInfo *info,
                                              const char * returnArgName);

SIMULINK_EXPORT_FCN bool slcsHasSimulinkFunctionsDefined(SimStruct* S);

#ifdef __cplusplus
SIMULINK_EXPORT_FCN void slcsGetDefinedSimulinkFunctions(SimStruct* S, std::vector<std::string> * slFcnList);
SIMULINK_EXPORT_FCN void slcsGetDefinedSimulinkFunctionsFromBlock(double blockH, std::vector<std::string> * slFcnList);
#endif

SIMULINK_EXPORT_FCN void slcsRegisterSimulinkFunction(
    SimStruct* S, const char * fcnName, SimulinkFunctionPtr fcnPtr,
    _ssFcnCallInfo *callInfo, const char * relativePathToFunction, bool isGlobal);

SIMULINK_EXPORT_FCN void slcsRequestService(
    SimStruct* S, const char * fcnName, _ssFcnCallExecArgs args);

SIMULINK_EXPORT_FCN void slcsInvokeSimulinkFunctionVoidArgs(
    SimStruct* S, const char* fcnName, int_T numArgs, void* args[]);

SIMULINK_EXPORT_FCN  void slcsRegisterCallerBlock(
    SimStruct* S, const char * fcnName, _ssFcnCallInfo *callInfo,
    const char * relativePathToCaller);

SIMULINK_EXPORT_FCN void slcsRegisterCallgraph(SimStruct *S,
                                               const char *callerFcnName,
                                               const char *relativePathToFunction,
                                               const char *calledFcnName,
                                               const char *calledFcnPath,
                                               const char *callerInfo);

SIMULINK_EXPORT_EXTERN_C const void *slcsGetSimulinkFunctionPathChecksum(SimStruct  *S,
                                                                         const char *fcnName);

SIMULINK_EXPORT_FCN _ssFcnCallInfo slcsGetFcnCallInfo(
    SimStruct* S, const char * fcnName);

SIMULINK_EXPORT_FCN void slcsFreeFcnCallInfo(
    _ssFcnCallInfo callInfo);

SIMULINK_EXPORT_FCN void slcsUpdateServerSFcnCatalog(
    SimStruct* S, const char * fcnName, void * fPtr);

SIMULINK_EXPORT_FCN bool slcsIsFunctionLocalScoped(
   SimStruct* S, const char * scopedFcnName);

SIMULINK_EXPORT_FCN bool slcsIsFunctionRegistered(
    SimStruct* S, const char * fcnName);

SIMULINK_EXPORT_FCN bool slcsIsFunctionRegisteredWithModel(
    double modelHandle, double blockHandle, const char * fcnName);
   
SIMULINK_EXPORT_EXTERN_C bool slcsIsFunctionRegisteredForBlock(
    double block, const char * fcnName);   

SIMULINK_EXPORT_FCN const char * slcsGetCodeGenScopedFcnName(
    SimStruct* S, const char * fcnName);

SIMULINK_EXPORT_FCN void slcsSetCodeGenScopedFcnName(
    SimStruct* S, const char * fcnName, const char * codeGenName);

SIMULINK_EXPORT_FCN
void slcgxeBlkRequestService(SimStruct* S, const char* fcnName,
                             const char* fullPath,
                             int blkId, int numInputs,
                             void* inArgs, void* inSizes,
                             int numOutputs, void* outArgs,
                             int* outSizes);
}
#endif
