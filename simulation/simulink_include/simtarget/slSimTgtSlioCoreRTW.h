/* Copyright 2016-2017 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTSLIOCORERTW_H
#define SLSIMTGTSLIOCORERTW_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void * rtwGetNewSlioCatalogue(const char * settingsFileName);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwTerminateSlioCatalogue(void ** catalogue);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwSlioUpdateAccessor(
    void * accessor,
    const unsigned int portIdx,
    const double time,
    const void * dataPointer);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwSaveDatasetsToMatFile(
    void * catalogue,
    const char * fileName);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C const void * rtwGetSignalDescriptor(
    void * treeVector,
    const unsigned int numberOfInputPorts,
    const unsigned int portType,
    const unsigned int maxDataPoints,
    const unsigned int decimation,
    const char *loggedName,
    const char *propagatedName,
    const char *allButLocalBlockPath,
    const char *localBlockPath,   
    const unsigned int portIdx,
    const size_t inportOrder,
    void * catalogue,
    const unsigned int* foreachDimsTop,
    const unsigned int foreachDimsTopSize,
    const unsigned int* foreachDimsSub,
    const unsigned int foreachDimsSubSize);
   
SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void * rtwGetAccessor(
    const void * signalDescriptor,
    const void * loggingInterval);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C unsigned int rtwLoggingOverride(
    const void * signalDescriptor,
    void * catalogue);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwAddTopNVBusNode(
    const unsigned int datatypeId,
    const char* signalName,   
    const unsigned int numberOfChildren,   
    const unsigned int* dimensions,
    const unsigned int numberOfDimensions,
    const unsigned int busSize,
    const unsigned int* offsets,
    const char * sampleTime,
    const double discreteInc,
    const double stopTime,
    void * treeVector);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwAddNVBusNode(
    const unsigned int datatypeId,
    const char * signalName,   
    const unsigned int numberOfChildren,   
    const unsigned int * dimensions,
    const unsigned int numberOfDimensions,
    const unsigned int busSize,
    const unsigned int* offsets,
    void * treeVector);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwAddTopBusNode(
    const char * signalName,
    const unsigned int numberOfChildren,
    void * treeVector);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwAddBusNode(
    const char * signalName,
    const unsigned int numberOfChildren,
    void * treeVector);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void * rtwAddLeafNode(
    const unsigned int datatypeId,
    const char * signalName,
    const char * interpolationMethod,
    const unsigned int isComplexData,
    const unsigned int * dimensions,
    const unsigned int numberOfDimensions,   
    const char * datatypeClassName,
    const char * units,
    const char * sampleTime,
    const double discreteInc,
    const double stopTime,
    void * treeVector);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void * rtwAddFixedPointLeafNode(
    const unsigned int datatypeId,
    const char * signalName,
    const char * interpolationMethod,
    const unsigned int isComplexData,
    const unsigned int * dimensions,
    const unsigned int numberOfDimensions,   
    const char * datatypeClassName,
    const char * units,
    const bool isSigned,
    const int32_T wordLength,
    const double slopeAdj,
    const int32_T exp,
    const double bias,
    const char * sampleTime,
    const double discreteInc,
    const double stopTime,
    void * treeVector);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void * rtwAddEnumLeafNode(
    const unsigned int datatypeId,
    const char * signalName,
    const char * interpolationMethod,
    const unsigned int * dimensions,
    const unsigned int numberOfDimensions,   
    const char * datatypeClassName,
    const char * units,
    const char * resolvedDatatypeClassName,
    const char * sampleTime,
    const double discreteInc,
    const double stopTime,
    const unsigned int * enumValues,
    const char ** enumLabels,
    const unsigned int numberOfEnumValues,
    void * treeVector);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwPopNVBusNode(
    void * treeVector);   

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void * rtwGetTreeVector();

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void * rtwGetPointerFromUniquePtr(void * uniquePtr);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwDestructAccessorPointer(void * uniquePtr);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C unsigned int rtwDisableStreamingToRepository(const void* catalogue);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C unsigned int rtwLogRootOutport(const void* catalogue);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void* rt_SlioAccessorUpdate(int version, int nargs, ...);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void* rt_SlioAccessorRelease(int version, int nargs, ...);

#endif
