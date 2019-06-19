/* Copyright 2015 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTINSTRUMENTATIONSLMESSAGEBRIDGE_H
#define SLSIMTGTINSTRUMENTATIONSLMESSAGEBRIDGE_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void * slmsg_ssGetMsgMemMgr(SimStruct *S);
SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C int slmsg_ssGetMsgQueueId(SimStruct *S, int qId);
SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C int slmsg_ssGetExternalQueueForOutput(SimStruct *S, int rootOutportIdx);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void ssGetInputServiceHostPtr(SimStruct* S, int portIdx, const char* svcName, void** hostPtr);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void ssGetInputServiceFcnPtr(SimStruct* S, int portIdx, const char* svcName, void** fcnPtr);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void ssGetOutputServiceHostPtr(SimStruct* S, int portIdx, const char* svcName, void** hostPtr);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void ssGetOutputServiceFcnPtr(SimStruct* S, int portIdx, const char* svcName, void** fcnPtr);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void ssSetInputServiceHost(SimStruct* S, int portIdx, const char* svcName, void* host);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void ssSetInputServiceFcn(SimStruct* S, int portIdx, const char* svcName, void* fcn);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void ssSetOutputServiceHost(SimStruct* S, int portIdx, const char* svcName, void* host);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void ssSetOutputServiceFcn(SimStruct* S, int portIdx, const char* svcName, void* fcn);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void slmsg_invokeSendData(SimStruct* S, void* hostPtr, void* fcnPtr, const void* data_to_send);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void slmsg_invokeRecvData(SimStruct* S, void* hostPtr, void* fcnPtr, void* received_data);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void ssUsingGRTCodeForSimulation(SimStruct* S, void* host, void* fcn, int* usingGRT);

#endif
