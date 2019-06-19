/* Copyright 1990-2017 The MathWorks, Inc. */

#ifndef __SLCLIENTSERVERAPIBRIDGE_TYPES_H__
#define __SLCLIENTSERVERAPIBRIDGE_TYPES_H__

#ifndef _ssFCNCALLINFO_DEFINED
typedef struct _ssFcnCallArgInfo_tag {
    DimsInfo_T *dimsInfo;
    int_T      dataType;
    int_T      argumentType;
    int_T      complexSignal;
} _ssFcnCallArgInfo;

typedef struct _ssFcnCallStatusArgInfo_tag _ssFcnCallStatusArgInfo;

typedef struct _ssFcnCallInfo_tag {
    int_T                    numInArgs;
    int_T                    numOutArgs;
    int_T                    numInOutArgs;
    int_T                    numCallers;
    int_T                    numCallerTIDs;
    _ssFcnCallArgInfo        *inArgs;
    _ssFcnCallArgInfo        *outArgs;
    int_T                    *inOutArgs;
    int_T                    *inArgsSymbDims;
    int_T                    *outArgsSymbDims;
    int_T                    *callerTIDs;
    int_T                    returnArgIndex;
    const char               *returnArgName;
    const char               **callerBlockPaths;
    const char               **argNames;
    const char               **argIndices;
    struct {
        unsigned int canBeInvokedConcurrently : 1;
    } flags;
} _ssFcnCallInfo;
#define _ssFCNCALLINFO_DEFINED
#endif

typedef void (*SimulinkFunctionPtr)(SimStruct *S, int tid, _ssFcnCallExecArgs *args);

#endif
