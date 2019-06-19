/* Copyright 2015 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTLOGLOADBLOCKSSFCNBRIDGE_H
#define SLSIMTGTLOGLOADBLOCKSSFCNBRIDGE_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void slSigLogAddElementDescriptionForBus(SimStruct* sfcnS,
                                         void **ppDatasetDesc,
                                         int_T hierInfoIdx,
                                         const char_T *busName,
                                         const char_T *signalName,
                                         const char_T *propName,
                                         const char_T *blockPath,
                                         int_T portIdx,            
                                         int_T maxPoints,          
                                         int_T decimation,         
                                         int_T numDims,            
                                         const int_T *dims,         
                                         SSLoggerInterpMethod interp);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void slSigLogAddElementDescriptionForStateflowBus(SimStruct* sfcnS,
                                         void **ppDatasetDesc,
                                         int_T hierInfoIdx,
                                         const char_T *busName,
                                         const char_T *signalName,
                                         const char_T *blockPath,
                                         int_T portIdx,            
                                         int_T maxPoints,          
                                         int_T decimation,         
                                         int_T numDims,            
                                         const int_T *dims,         
                                         SSLoggerInterpMethod interp,
                                         const char_T *className,
                                         const char_T *chartPath);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void slSigLogUpdateDatasetElementForBus(SimStruct* sfcnS,
                                        void *pwork,
                                        real_T inputTime,
                                        const char *inputData);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void slReportToWksNoRTWSupportForDataset(SimStruct* sfcnS,
                                         const char_T *logBlockPath,
                                         const char_T *saveFormat);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void slSigLogAddTimeseriesDescription(SimStruct* sfcnS,
                                      void         **ppElementDesc,
                                      void         **ppDatasetDesc,
                                      const char_T *name,
                                      int_T        nDims,
                                      const int_T  *dims,
                                      DTypeId      dataType,
                                      boolean_T    complexity,
                                      SSLoggerInterpMethod interp,
                                      const char_T *units);
SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void rtwGetLoggingIntervalFromSFBlock(void *pOpaqueSFBlock,
                                      void **outLoggingInterval);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
int_T slSigLogGetForEachDimsForRefModel(SimStruct* sfcnS, int_T *);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void slmrRunPluginEvent(SimStruct *sfcnS,
                        const char* modelName,
                        const char* event);
#endif
