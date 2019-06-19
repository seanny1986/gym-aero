/* Copyright 2018 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTINTERLEAVEDCOMPLEX_H
#define SLSIMTGTINTERLEAVEDCOMPLEX_H

#include "sl_simtarget_instrumentation_spec.h"

SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void simTarget_sfcnSeperateComplexCaller(SimStruct* simStruct, void(*fcn)(SimStruct*));
SL_SIMTARGET_INSTRUMENTATION_EXPORT_EXTERN_C void simTarget_sfcnSeperateComplexCaller_withTID(SimStruct* simStruct, void(*fcn)(SimStruct*, int_T), int_T tid);

#endif
