/* Copyright 2013-2017 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef _sl_messages_sfcn_bridge_h_
#define _sl_messages_sfcn_bridge_h_

#include "simulink_spec.h"

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

typedef enum {
    SS_MESSAGE_QUEUE_OVERFLOW_ERROR,
    SS_MESSAGE_QUEUE_OVERFLOW_WARNING,
    SS_MESSAGE_QUEUE_OVERFLOW_IGNORE
} ssMessageQueueOverflowDiagnostic;

typedef void* ssMessage;

SIMULINK_EXPORT_EXTERN_C
void slmsg_ssSetMsgQueueDWorkId(SimStruct *S, int dIndex, int dId);

SIMULINK_EXPORT_EXTERN_C void slmsg_ssSetOutputPortIsMessage(SimStruct *S, int pIdx, int val);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssSetInputPortIsMessage(SimStruct *S, int pIdx, int val);

SIMULINK_EXPORT_EXTERN_C void slmsg_ssSetNumLocalMessageQueues(SimStruct *S, int num);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssSetMessageQueueCapacity(SimStruct *S, ssMessageQueueLocationType queueLocType, int queueID, int queueLen);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssMessageQueueSetDataProperties(SimStruct *S, ssMessageQueueLocationType queueType, int queueID, int numDims, int_T* dims, DTypeId typeId, int complexity);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssSetMessageQueueType(SimStruct *S, ssMessageQueueLocationType queueLocType, int queueID, ssMessageQueueType queueType);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssSetMessageQueuePriorityOrder(SimStruct *S, ssMessageQueueLocationType queueLocType, int queueID, ssMessagePriorityOrder pOrder);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssSetMessageQueueOverflowDiagnostic(SimStruct *S, ssMessageQueueLocationType queueLocType, int queueID, ssMessageQueueOverflowDiagnostic qOverflow);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssSetInputPortMightQueryNumMsg(SimStruct *S, size_t portIdx);

SIMULINK_EXPORT_EXTERN_C uint_T slmsg_ssMessageSend(SimStruct *S, int outputPortIdx, const void *payload, unsigned int priority);
SIMULINK_EXPORT_EXTERN_C uint_T slmsg_ssMessageSendToLocalQueue(SimStruct *S, int pIdx, const void *payload, unsigned int priority);
SIMULINK_EXPORT_EXTERN_C int_T  slmsg_ssGetNumMessagesInOutputQueue(SimStruct *S, int pIdx);

SIMULINK_EXPORT_EXTERN_C uint_T slmsg_ssMessageRelay(SimStruct *S, int outputPortIdx, ssMessage msgObj);

SIMULINK_EXPORT_EXTERN_C int_T slmsg_ssGetNumMessagesInQueue(SimStruct *S, ssMessageQueueLocationType qt, int queueID);

SIMULINK_EXPORT_EXTERN_C int_T slmsg_ssGetMessagePayloadSizeInLocalQueue(SimStruct *S, int queueID);

SIMULINK_EXPORT_EXTERN_C uint_T slmsg_ssMessageQueuePush(SimStruct *S, ssMessageQueueLocationType qt, int queueID, ssMessage msg);

SIMULINK_EXPORT_EXTERN_C ssMessage slmsg_ssMessageQueuePop(SimStruct *S, ssMessageQueueLocationType qt, int queueID, uint_T* isOverflowed);

SIMULINK_EXPORT_EXTERN_C ssMessage slmsg_ssMessageQueuePopForDesCharts(SimStruct *S, ssMessageQueueLocationType qt, int queueID, bool isMsgTrigger, uint_T* isOverflowed);

SIMULINK_EXPORT_EXTERN_C int  slmsg_ssEntityStartTimer(SimStruct *S, int timerId, double delay);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssEntityCancelTimer(SimStruct *S, int timerId);
SIMULINK_EXPORT_EXTERN_C int  slmsg_ssEntityGetActiveTimer(SimStruct *S);

SIMULINK_EXPORT_EXTERN_C void slmsg_ssEntityAddTrigger(SimStruct *S, ssMessageQueueLocationType qt, int queueID, int triggerID);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssEntityRemoveTrigger(SimStruct *S, ssMessageQueueLocationType qt, int queueID, int triggerID);

SIMULINK_EXPORT_EXTERN_C void *slmsg_ssMessageQueuePeekPayload(SimStruct *S, ssMessageQueueLocationType qt, int queueID);

SIMULINK_EXPORT_EXTERN_C void *slmsg_ssMessageQueuePeekPayloadAtIndex(SimStruct *S, ssMessageQueueLocationType qt, int queueID, int msgIndex);
SIMULINK_EXPORT_EXTERN_C void *slmsg_ssMessageOutputQueuePeekPayloadAtIndex(SimStruct *S, int portIdx, int msgIndex);

SIMULINK_EXPORT_EXTERN_C void slmsg_ssMessagePayloadDestroy(SimStruct *S, void *msgData);

SIMULINK_EXPORT_EXTERN_C void *slmsg_ssGetMessagePayload(SimStruct *S, ssMessage msg);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssSetMessagePayload(SimStruct *S, ssMessage msg, const void *payload);
SIMULINK_EXPORT_EXTERN_C void slmsg_ssMessageDestroy(SimStruct *S, ssMessage msgObj);

SIMULINK_EXPORT_EXTERN_C boolean_T slmsg_ssIsMessageQueueEmpty(SimStruct *S, ssMessageQueueLocationType qt, int portId);
SIMULINK_EXPORT_EXTERN_C boolean_T slmsg_ssIsMessageQueueFull(SimStruct *S, ssMessageQueueLocationType qt, int portIdx, bool isOutputPort, bool throwErrQueryNumMsgOnMultirateQueue);
SIMULINK_EXPORT_EXTERN_C size_t slmsg_ssGetNumDroppedMessagesFromLocalQueue(SimStruct *S, int queueId);
SIMULINK_EXPORT_EXTERN_C size_t slmsg_ssGetRecommendedCapacityForLocalQueue(SimStruct *S, int queueId);

SIMULINK_EXPORT_EXTERN_C ssMessageQueueOverflowDiagnostic slmsg_ssGetMessageQueueOverflowDiagnostic(SimStruct *S, ssMessageQueueLocationType queueLocType, int queueID);

#endif
