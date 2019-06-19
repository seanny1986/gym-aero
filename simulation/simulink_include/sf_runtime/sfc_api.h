/* Copyright 1995-2013 The MathWorks, Inc. */

#ifndef _SFC_API_H_
#define _SFC_API_H_

#include "sf_runtime_spec.h"
#include "mex.h"
struct SfDebugInstanceStruct;

LIBMWSF_RUNTIME_API unsigned int sf_debug_api(SfDebugInstanceStruct* debugInstance, int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]);

#endif
