/* Copyright 2017 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTSLIOSDIRTW_H
#define SLSIMTGTSLIOSDIRTW_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void rtwAddSdiClient(
    void * accessor,
    const void * signalDescriptor,
    void * catalogue,
    char * datasetName);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void* rt_SlioAccessorAddClientAssessmentSdi(
    int version, int nargs, ...);
#endif
