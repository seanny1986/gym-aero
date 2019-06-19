/* Copyright 2016 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTSLFILEIORTW_H
#define SLSIMTGTSLFILEIORTW_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwMatFileLoaderCollectionCreateInstance(
    int    errorXMLMode,
    void **outMatFileLoaderCollection
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwMatFileLoaderCollectionAddElement(
     int                   errorXMLMode,
    void                 *matFileLoaderCollection,
    const char           *fileName,
    const char           *varPath,
    const int             extrapolationBeforeFirstDataPointInt,
    const int             interpolationWithinTimeRangeInt,
    const int             extrapolationAfterLastDataPointInt,
    const int             iZeroCrossingSwitch,
    const unsigned char  *ground,
    const char           *className,
    const int             storageDTypeId,
    const int             nDims,
    const int            *dims,
    const int             complexity,
    const int             isFixedPoint,
    const int             dataTypeMode,
    const int             isSigned,
    const int             wordLength,
    const double          slopeAdjustmentFactor,
    const int             fixedExponent,
    const double          bias,
    const int             enumNStrings,
    const char          **enumStrings,
    const int            *enumValues,
    int                   fxpDiagnosticOverflow,
    int                   fxpDiagnosticSaturation,
    const char           *fxpBlockPath
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwMatFileLoaderCollectionGetOutput(
    int            errorXMLMode,
    void          *matFileLoaderCollection,
    const size_t   elementId,
    const double   t,
    const int      iMajorTimeStep,
    void         **outOutputValue
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwMatFileLoaderCollectionDestroyInstance(
    int   errorXMLMode,
    void *matFileLoaderCollection
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwH5LoggingCollectionCreateInstance(
    int    errorXMLMode,
    void **outH5LoggingCollection,
    void * intervalLogging
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwH5LoggingCollectionDestroyInstance(
    int   errorXMLMode,
    void *h5LoggingCollection);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwH5LoggingCollectionAddElement(
    int           errorXMLMode,
    void         *h5LoggingCollection,
    const char   *fileName,
    const char   *mtxName,
    const int     leafIdx,
    const char   *signalName,
    const char   *interpolationName,
    const char   *className,
    const int     nDims,
    const int    *dims,
    const int     isComplex,
    const int     decimation,
    const int     formatInt,
    const int     isFixedPoint,
    const int     dataTypeMode,
    const int     isSigned,
    const int     wordLength,
    const double  slopeAdjustmentFactor,
    const int     fixedExponent,
    const double  bias
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwMATFileLoaderCollectionGetZeroCrossingSignal(
    void         *pMatFileLoaderCollection,
    const size_t  elementId,
    const double  t,
    const int     iMajorTimeStep,
    void         *outZeroCrossingSignal
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwH5LoggingCollectionWrite(
    int           errorXMLMode,
    void         *h5LoggingCollection,
    const size_t  elementId,
    const double  t,
    const void   *u
    );

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C
const char *rtwSetMcosObjName(
    const char *fileName,
    int        errorXMLMode,
    const char *objName
    );

#endif

