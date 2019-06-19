/* Copyright 2018 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SL_SIMTARGET_INSTRUMENTATION_SLSIMTGTPARTITIONINGBRIDGE_H
#define SL_SIMTARGET_INSTRUMENTATION_SLSIMTGTPARTITIONINGBRIDGE_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void simTgtScheduleTaskUsingRateIndex(SimStruct* S,
                                                                                   int_T sti);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void simTgtDisableTaskUsingRateIndex(SimStruct* S,
                                                                                  int_T sti);

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void simTgtEnableTaskUsingRateIndex(SimStruct* S,
                                                                                 int_T sti);

#endif
