/* Copyright 2016 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTSIGSTREAMRTW_H
#define SLSIMTGTSIGSTREAMRTW_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
void rtwAobHierarchyCreateRootNode(void **outAobHierarchy);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwAobHierarchyCreateNode(const char *name, void **outAobHierarchy);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwAobHierarchySetDim(
    void *pOpaqueAobHierarchy,
    const unsigned int nDims,
    const unsigned int *pDim
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwAobHierarchyAddChild(
    void *pOpaqueAobParent,
    void *pOpaqueAobChild
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwAobHierarchyVerifyNNodes(
    void *pOpaqueAobHierarchy,
    const unsigned int nNodes
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwSignalProbeCollectionCreateInstance(
    const char * datasetMetadataKey,
    const char *datasetName,
    const char *elementName,
    const char *propName,
    const char *allButLocalBlockPath,
    const char *localBlockPath,
    const unsigned int portIdx,
    void *pOpaqueOSigstreamManager,
    void **outSignalProbeCollection
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwSignalProbeCollectionDestroyInstance(
    void *pOpaqueSignalProbeCollection   
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwSignalProbeCollectionAddElement(
    void * const pOpaqueSignalProbeCollection,
    const char *signalName,
    const char *interpMethod,
    const unsigned int decimation,
    const unsigned int maxDataPoints,
    const unsigned int nDims,
    const unsigned int *pDim,
    const unsigned int complexity,
    const char *units,
    const char *typeName,
    const char *resolvedTypeName
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwSignalProbeCollectionAddElementFxp(
    void * const pOpaqueSignalProbeCollection,
    const char *signalName,
    const char *interpMethod,
    const unsigned int decimation,
    const unsigned int maxDataPoints,
    const unsigned int nDims,
    const unsigned int *pDim,
    const unsigned int complexity,
    const char *units,
    const char *typeName,
    const unsigned int isSigned,
    const int wordLength,
    const double slopeAdjustmentFactor,
    const int fixedExponent,
    const double bias
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwSignalProbeCollectionVerifyNLeaves(
    const void * const pOpaqueSignalProbeCollection,
    const unsigned int nLeaves
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwSignalProbeCollectionSetAobHierarchy(
    void * const pOpaqueSignalProbeCollection,
    void * const pOpaqueAobHierarchy
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwSignalProbeCollectionFinalizeMetaData(
    void * const pOpaqueSignalProbeCollection
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwSignalProbeCollectionFinalizeMetaDataForeach(
    void * const pOpaqueSignalProbeCollection,
    const unsigned int numForeachLevels,
    const unsigned int *foreachDims
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwSignalProbeCollectionAppendSample(
    void * const pOpaqueSignalProbeCollection,
    const unsigned int elementIdx,
    const double t,
    const void * const pData
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 void rtwOSigstreamManagerSaveDatasetsToMatFile(
    void *pOpaqueOSigstreamManager,
    const char *fileName
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
 int rtwSignalProbeCollectionGetLoggingIsOn(
    const void * const pOpaqueSignalProbeCollection
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
bool rtwTimeInLoggingInterval(void *pOpaqueLoggingInterval,
                                     const time_T time);

#endif

