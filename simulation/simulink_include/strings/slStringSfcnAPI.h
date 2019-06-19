/* API for s-functions and Stateflow to support Simulink String */

/* Copyright 2015-2016 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef _sl_string_sfcn_api_h_
#define _sl_string_sfcn_api_h_

#include "simulink_spec.h"

/* =================== String data type APIs ==================== */
/* Register string data type. String is dynamic (unbounded) if maxStringLength = 0. */
SIMULINK_EXPORT_EXTERN_C DTypeId ssRegisterStringDataType(SimStruct *S, size_t maxStringLength);

/* Check if a data type id is string. */
SIMULINK_EXPORT_EXTERN_C bool ssIsStringDataType(SimStruct *S, int dataType);

/* Return maximum length (number of characters) of a string data type. Return 0 if string is unbounded. */
SIMULINK_EXPORT_EXTERN_C size_t ssGetStringDataTypeMaxLength(SimStruct *S, int dataType);

/* Return default buffer size (Bytes) for dynamic (unbounded) string data type. */
SIMULINK_EXPORT_EXTERN_C size_t ssGetDynamicStringDefaultBufferSize(SimStruct *S);

/* Return buffer size (Bytes) for a string data type - N+1 for 'stringtype(N)' or model config setting for 'string'. */
SIMULINK_EXPORT_EXTERN_C size_t ssGetStringDataTypeBufferSize(SimStruct *S, int dataType);

/* Return diagnostic setting in model config set for string truncation - Error/Warning/None. */
SIMULINK_EXPORT_EXTERN_C BDErrorValue ssGetStringTruncationDiagnosticSetting(SimStruct *S);
SIMULINK_EXPORT_EXTERN_C int ssGetTargetLangStandard(SimStruct *S);

/* =================== String data type support opt-in ====================== */
/* The opt-in APIs needd to be called after EvalDlgPrms() stage, otherwise they won't have effect. */
SIMULINK_EXPORT_EXTERN_C void ssSetInputSupportStringDataType(SimStruct *S, int portIdx, bool support);
SIMULINK_EXPORT_EXTERN_C void ssSetOutputSupportStringDataType(SimStruct *S, int portIdx, bool support);
/* =============================================================== */

#endif /* _sl_string_sfcn_api_h_ */
