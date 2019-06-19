/* Copyright 2013-2014 The MathWorks, Inc. */

#ifndef _sf_test_language_h_
#define _sf_test_language_h_

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#include "sf_runtime/sf_runtime_spec.h"
#include "slTestResult.h"
#include "slTestTypes.h"

LIBMWSF_RUNTIME_API void slTestInitialize(struct slTestBlkInfo* blkInfo, slTestResult* current, slTestResult* final, double* finalResultTimes, int count);
LIBMWSF_RUNTIME_API void slTestTerminate(struct slTestBlkInfo* blkInfo);

LIBMWSF_RUNTIME_API void slTestLogAssessments(struct slTestBlkInfo* blkInfo, double currentTime);
LIBMWSF_RUNTIME_API void slTestEvalOnFail(struct slTestBlkInfo* blkInfo, int idx, double currentTime, mxClassID fmtStrArgTypes[], void* fmtData[]);

LIBMWSF_RUNTIME_API void slTestRegAssessment(
    struct slTestBlkInfo* blkInfo,
    int idx,               
    char* sfPath,          
    int ssidNumber,        
    int labelStartPosition,
    int labelEndPosition,  
    char* msgId,           
    char* fmtStr,          
    mxClassID* fmtStrTypes 
    );

#endif

