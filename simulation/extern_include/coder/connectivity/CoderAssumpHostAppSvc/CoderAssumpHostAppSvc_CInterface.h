/* Copyright 2015-2017 The MathWorks, Inc. */

#ifndef CODERASSUMPHOSTAPPSVC_CINTERFACE_H
#define CODERASSUMPHOSTAPPSVC_CINTERFACE_H

#include "coder/connectivity/CoderAssumpHostAppSvc/CoderAssumpHostAppSvc.h"
#include "coder/connectivity/CoderAssumpHostAppSvc/CoderAssumpHostAppSvc_spec.h"
#include "mex.h"

CODERASSUMPHOSTAPPSVC_API_EXTERN_C boolean_T coderAssumpHostAppSvcCreate(
        void** const ppCoderAssumpHostApp,
        void* const pCSVoid,
        void* const pXILUtilsVoid,
        const boolean_T isVerbose,
        const boolean_T prodEqTarget,
        mxArray * codeGenComponent,
        mxArray * silpilInterface,
        mxArray * inTheLoopType,
        mxArray * blockPath,
        const boolean_T isSimulinkConfig,
        const boolean_T isUsingCS,
        const boolean_T isSIL);

CODERASSUMPHOSTAPPSVC_API_EXTERN_C void coderAssumpHostAppSvcDestroy(
        void* const pCoderAssumpHostApp);

#endif
