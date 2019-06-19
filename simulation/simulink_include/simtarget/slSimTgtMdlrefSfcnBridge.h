/* Copyright 2015-2018 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTMDLREFSFCNBRIDGE_H
#define SLSIMTGTMDLREFSFCNBRIDGE_H

#include "sl_simtarget_core_spec.h"

SL_SIMTARGET_CORE_EXPORT_EXTERN_C int_T slmrGetTopTidFromMdlRefChildTid(SimStruct* S,
                                                                        int_T childTid,
                                                                        bool childTidIsExplicitTasking);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrModelRefAddStateOwnerAndAccessor(
    SimStruct* S, const char *ownerBlkPath, const char *accessorBlkPath);

typedef void (*mdlSystemInitializeFcn)(SimStruct *S);
typedef void (*mdlSystemResetFcn)(SimStruct *S);
typedef void (*mdlPeriodicOutputUpdateFcn)(SimStruct *S, int_T tid);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrRegisterSystemInitializeMethod(
    SimStruct* S, mdlSystemInitializeFcn fcn);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrRegisterSystemResetMethod(
    SimStruct* S, mdlSystemResetFcn fcn);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrRegisterPeriodicOutputUpdateMethod(
    SimStruct* S, mdlPeriodicOutputUpdateFcn fcn);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrAccelRunBlockSystemInitialize(
    SimStruct *S, int sysidx, int blkidx);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrAccelRunBlockSystemReset(
    SimStruct *S, int sysidx, int blkidx);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrModelRefRegisterSimStateChecksum(
    SimStruct* S, const char* mdlname, const uint32_T* chksum);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrInitializeIOPortDataVectors(
    SimStruct *S, int nIPorts, int nOPorts);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrSetForeachDimensions(
    void *pDatasetDesc, uint_T numForEachLevels, const uint_T *forEachDims);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrLogStatesAfterStateWrite(
    SimStruct *s,  const char* writerSID);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrCreateDiagnostic(
    const char* id, int nargs, va_list args, void** diag);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmrCreateFullPathToTop(
    const char* toppath, const char* subpath, char** fullpath);
#endif
