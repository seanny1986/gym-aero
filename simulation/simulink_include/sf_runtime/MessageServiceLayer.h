/* Copyright 2013-2017 The MathWorks, Inc. */

#ifndef _message_service_layer_h_
#define _message_service_layer_h_

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#if defined(S_FUNCTION_NAME)
#include "mwmathutil.h"
#endif

#include "sf_runtime/sf_runtime_spec.h"
#ifndef _SIMSTRUCT
#define _SIMSTRUCT
typedef struct SimStruct_tag SimStruct;
#endif

typedef struct MsgBlockInfo {
    const char* blockSID;
    void* targetSpecificData;
} MsgBlockInfo_t;

LIBMWSF_RUNTIME_API void* slMessageRtRegisterLocalMessageQueueUsingMsgBlock(MsgBlockInfo_t* blkInfo, int queueId, unsigned int ssid);
LIBMWSF_RUNTIME_API void* slMessageRtRegisterInputMessageQueueUsingMsgBlock(MsgBlockInfo_t* blkInfo, int portNum, unsigned int ssid);
LIBMWSF_RUNTIME_API void* slMessageRtRegisterOutputMessageQueueUsingMsgBlock(MsgBlockInfo_t* blkInfo, int portNum, unsigned int ssid);

LIBMWSF_RUNTIME_API void* slMessageRtRegisterLocalMessageQueue(SimStruct* s, int queueId, unsigned int ssid);
LIBMWSF_RUNTIME_API void* slMessageRtRegisterInputMessageQueue(SimStruct* s, int portNum, unsigned int ssid);
LIBMWSF_RUNTIME_API void* slMessageRtRegisterOutputMessageQueue(SimStruct* s, int portNum, unsigned int ssid);

LIBMWSF_RUNTIME_API void slMessageRtDiagnosticCheck(void* msgInterface, void* destMsgInterface, const char* diagnosticDesc, unsigned int ssid, int offsetSrcLoc, int lengthSrcLoc);
LIBMWSF_RUNTIME_API void slMessageRtSetQueueAttribute(void* msgInterface, const char* attribute, int value, const char* attribute2, int value2);

LIBMWSF_RUNTIME_API uint_T slMessageRtHasOverflowOccurred(int status);
LIBMWSF_RUNTIME_API int slMessageRtMessageWasPopped(int status);
LIBMWSF_RUNTIME_API void slMessageRtDestroyQueue(void* msgInterface);

LIBMWSF_RUNTIME_API int slMessageRtSend(void* msgInterface, void* payload);
LIBMWSF_RUNTIME_API int slMessageRtReceive(void* msgInterface, void** payload);
LIBMWSF_RUNTIME_API int slMessageRtDesReceive(void* msgInterface, void** payload, bool isMsgTrigger);
LIBMWSF_RUNTIME_API int slMessageRtForward(void* msgInterfaceSrc, void* msgInterfaceDest);
LIBMWSF_RUNTIME_API void slMessageRtDestroyActiveMessageIfPresent(void* msgInterface);
LIBMWSF_RUNTIME_API void slMessageRtResetMsgHasBeenSet(void* msgInterface);
LIBMWSF_RUNTIME_API int slMessageRtIsMessageValid(void* msgInterface);

LIBMWSF_RUNTIME_API int slMessageRtHasMessageBeenForwarded(void* msgInterface);

LIBMWSF_RUNTIME_API int_T slMessageRtLength(void* msgInterface);
LIBMWSF_RUNTIME_API void* slMessageRtPeek(void* msgInterface);
LIBMWSF_RUNTIME_API void* slMessageRtPeekAtIndex(void* msgInterface, int indexInQueue);

#endif

