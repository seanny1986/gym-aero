/* Copyright 2018 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SLSIMTGTCORESLMESSAGEBRIDGE_H
#define SLSIMTGTCORESLMESSAGEBRIDGE_H

#include "sl_simtarget_core_spec.h"

#ifndef _ss_Message_Common_Enum_Type_
#define _ss_Message_Common_Enum_Type_

typedef enum {
    SS_MESSAGE_QUEUE_FIFO,
    SS_MESSAGE_QUEUE_LIFO,
    SS_MESSAGE_QUEUE_PRIORITY
} ssMessageQueueType;

typedef enum {
    SS_INPUT_PORT_MESSAGE_QUEUE,
    SS_LOCAL_MESSAGE_QUEUE
} ssMessageQueueLocationType;

typedef enum {
    SS_MESSAGE_QUEUE_ASCENDING,
    SS_MESSAGE_QUEUE_DESCENDING
} ssMessagePriorityOrder;

#endif

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmsg_ssSetOutputPortIsMessage(SimStruct *S, int pIdx, int val);
SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmsg_ssSetInputPortIsMessage(SimStruct *S, int pIdx, int val);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmsg_ssSetMessageQueueCapacity(
    SimStruct* S, ssMessageQueueLocationType queueType, int queueID, int qLen);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmsg_ssSetMessageQueueType(
    SimStruct *S, ssMessageQueueLocationType queueLocType, int queueID, ssMessageQueueType qType);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmsg_ssMessageQueueSetDataProperties(
    SimStruct *S, ssMessageQueueLocationType queueType, int queueID, int numDims, int_T* dims, DTypeId typeId, int complexity);   

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmsg_ssSetNumLocalMessageQueues(SimStruct *S, int num);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmsg_ssSetMessageQueuePriorityOrder(
    SimStruct *S, ssMessageQueueLocationType queueLocType, int queueID, ssMessagePriorityOrder pOrder);

SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmsg_ssSetInputPortMessageCompiledID(SimStruct *S, int portIdx, int qId);
SL_SIMTARGET_CORE_EXPORT_EXTERN_C void slmsg_ssSetMsgQueueSpecExternalInportIdx(SimStruct *S, int localIdx, int exInportIdx);

#endif
