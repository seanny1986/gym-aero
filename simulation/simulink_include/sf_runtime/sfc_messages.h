/* Copyright 2013-2014 The MathWorks, Inc. */

#ifndef _sf_runtime_sf_runtime_h_
#define _sf_runtime_sf_runtime_h_

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#include "simstruc.h"

#include "sf_runtime_spec.h"

LIBMWSF_RUNTIME_API void sf_msg_calendar_push(SimStruct* S, int outportNum, void* payload, boolean_T isLocalMsg);
LIBMWSF_RUNTIME_API int sf_msg_calendar_pop(SimStruct* S, int inportNum, void* payload, boolean_T* isPopped, boolean_T isForwarded, void** msgHandle, boolean_T isLocalMsg, int sizeOfPayload);
LIBMWSF_RUNTIME_API void sf_msg_calendar_consume(SimStruct* S, int inportNum, boolean_T* isPopped, boolean_T* isForwarded, void* msgHandle);
LIBMWSF_RUNTIME_API void sf_msg_calendar_forward(SimStruct* S,  boolean_T isPopped, boolean_T* isForwarded, void* srcMsgHandle, int msgDestIndexE, boolean_T isLocalMsg);
LIBMWSF_RUNTIME_API void sf_msg_calendar_update_payload(SimStruct* S,  void* msgHandle, const void* updatedPayload, boolean_T isPopped, boolean_T isForwarded);

#endif
