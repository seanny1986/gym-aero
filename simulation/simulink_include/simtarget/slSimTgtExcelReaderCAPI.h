/* Copyright 2015-2017 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTEXCELREADERCAPI_H
#define SLSIMTGTEXCELREADERCAPI_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C CHAR16_T * rtwExcelLoaderGetUnicodeStringFromChars(const char * str);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwExcelLoaderFreeLabel(CHAR16_T * str);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C const char *rtwExcelLoaderCreateInstanceWithSequenceSupport(const CHAR16_T * fileName,
    const CHAR16_T * sheetName,
    const CHAR16_T * range,
    const int    treatFirstColumnAs,
    const int    outputAfterLastPoint,
    const int    extrapolationBeforeFirstDataPointInt,
    const int    interpolationWithinTimeRangeInt,
    const int    extrapolationAfterLastDataPointInt,
    const unsigned char *ground,
    const int    iZeroCrossingSwitch,
    const int    signalNumber,
    const char * spreadsheetIOImpl,
    void        **outExcelLoader);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C const char *rtwExcelLoaderTerminate(void * readerObj);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C const char *rtwExcelLoaderGetZeroCrossingSignal(
    void         *pExcelFileLoader,
    const double  t,
    const int     iMajorTimeStep,
    void         *outZeroCrossingSignal);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C const char *rtwExcelLoaderGetOutput(void ** outOutputValue,
    void * pExcelLoader,
    const double t,
    const int isMajorTimeStep);

#endif
