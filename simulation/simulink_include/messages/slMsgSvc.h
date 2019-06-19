/* Copyright 2014-2018 The MathWorks, Inc. */
/* 
 * Copyright 2014-2018 The MathWorks, Inc.
 *
 * File: slMsgSvc.h
 *
 * Abstract:
 *    Simulink Coder interface for Simulink and Stateflow messages
 *
 *    Local switches:
 *    - define SLMSG_USE_STD_MEMCPY to allow use of standard memcpy/memset
 *      implementations - otherwise local implementations are used
 *    - define SLMSG_USE_EXCEPTION to allow run-time exceptions
 *    - define SLMSG_ALLOW_SYSTEM_ALLOC to include malloc/free code
 *      note that this also turns on SLMSG_USE_EXCEPTION
 */

#ifndef _slMsgSvc_h_
#define _slMsgSvc_h_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SL_INTERNAL

#   define SLMSG_ALLOW_SYSTEM_ALLOC
#   define SLMSG_USE_EXCEPTION
#   include "tmwtypes.h"

#   include "package.h"
#   ifdef BUILDING_SL_MESSAGES
#       define SL_MESSAGES_EXPORT_FCN_C    DLL_EXPORT_SYM
#   else
#       define SL_MESSAGES_EXPORT_FCN_C    DLL_IMPORT_SYM
#       define SL_MESSAGES_EXPORT_VAR      extern DLL_IMPORT_SYM
#   endif

#else

#   include "rtwtypes.h"
#   define SL_MESSAGES_EXPORT_FCN_C

#endif

#if defined(SL_INTERNAL)  || defined(MATLAB_MEX_FILE)
#   ifndef SLMSG_INCLUDE_TASK_TRANSITION_QUEUE
#       define SLMSG_INCLUDE_TASK_TRANSITION_QUEUE
#   endif
#endif

typedef ulong_T slMsgId;
typedef int_T slMsgQueueId;
typedef ulong_T slMsgDataSize;
typedef int_T slMsgMemPoolId;
typedef ulong_T slMsgPoolSize;

#ifndef __SLMSG_MEMPOOLMGR_DEFINED__
#define __SLMSG_MEMPOOLMGR_DEFINED__
typedef struct __slmsg_MemPoolMgr_T __slmsg_MemPoolMgr;
typedef struct __slmsg_MemPool_T slMsgMemPool;
#endif

#define DEF_EVT_PRIORITY (20)
#define SLMSG_UNSPECIFIED (-1)

#ifndef NULL
#  define NULL (0)
#endif

typedef enum _slMsgQueueType {
    SLMSG_QUEUE_UNUSED = 0,
    SLMSG_FIFO_QUEUE,
    SLMSG_LIFO_QUEUE,
    SLMSG_PRIORITY_QUEUE_ASCENDING,
    SLMSG_PRIORITY_QUEUE_DESCENDING,
    SLMSG_SYSPRIORITY_QUEUE_ASCENDING,
    SLMSG_SYSPRIORITY_QUEUE_DESCENDING,
    SLMSG_SRSW_LOCK_FREE_FIFO_QUEUE,
    SLMSG_HASHMAP_QUEUE
} slMsgQueueType;

typedef enum _slMsgQueueDropPolicy {
    SLMSG_DROP_NONE,
    SLMSG_DROP_HEAD_OF_QUEUE,
    SLMSG_DROP_TAIL_OF_QUEUE
} slMsgQueueDropPolicy;

typedef enum _slMsgPriorityMode {
    SLMSG_ASCENDING_PRIORITY,
    SLMSG_DESCENDING_PRIORITY
} slMsgPriorityMode;

typedef struct __slmsg_PoolListNode_T __slmsg_PoolListNode;
typedef struct __slmsg_PoolListNode_T slMsgLink;
typedef struct __slmsg_MemChunk_T __slmsg_MemChunk;
typedef struct __slmsg_MemPool_T __slmsg_MemPool;
typedef struct _slMessage slMessage;

/* Node of double linked list */
struct __slmsg_PoolListNode_T
{
    __slmsg_PoolListNode *pPrev, *pNext;
};

/* ------------------------------------------------------------------------
 * Memory chunk
 * 
 * A contiguously allocated block of memory from which units of a fixed
 * size can be allocated. It is also a linked list and holds a pointer to
 * the next chunk if the owning memory pool is allowed to grow.
 * ------------------------------------------------------------------------
 */
struct __slmsg_MemChunk_T
{
    void *fMemBlock;              /* Chunk memory block */
    slMsgPoolSize fBlockSize;     /* Chunk memory size */
    ulong_T fNumUnits;            /* Number of units of memory in block */
    __slmsg_MemChunk *fPrevChunk; /* Previous chunk - linked list */
};

/* ------------------------------------------------------------------------
 * Memory Pool
 * 
 * A memory pool that can allocate and free memory units of a fixed size.
 * The pool allocates a chunk of memory and allocates from it. Freed
 * memory is returned to the pool. 
 * 
 * The pool can be configured to grow if it does not have enough memory
 * or alternatively do a system allocation.
 * ------------------------------------------------------------------------
 */
struct __slmsg_MemPool_T
{
    __slmsg_MemChunk *fMemChunk; /* Memory chunk */
    ulong_T fMaxBlockSize;

    __slmsg_MemChunk fNonMallocMemChunk;
    __slmsg_PoolListNode* fAllocListHead; /* Alloc list */
    __slmsg_PoolListNode* fFreeListHead;  /* Free list */

    ulong_T fUnitSize;  /* Memory unit size */
    int_T   fDataType;  /* Data type of unit; set to -1 for built-in and +ve value for others*/
    int(*fConstruct)(void *data); /* Construct the data in-place in the memory provided */
    int(*fDestruct)(void *data); /* Destruct the data in-place in the memory provided */
    int(*fDeepCopy)(void *dst, const void* src); /* deep copy from src to destination */

    __slmsg_MemPool *fNextPool; /* Next memory pool */

    uint16_T fStatNumAlloc; /* Number of units allocated */
    uint16_T fStatNumFree;  /* Number of units freed */
    
    ulong_T fMemChunkSize; /* size of memchunk*/
    
    /* Whether or not memory blocks used for pooling are owned 
     * externally by some other utility or owned by this memory manager.
     * 
     * If true it implies:
     * - pools cannot grow if they run out of memory
     * - pools cannot do system allocation if they run out of memory
     * - new pools will not be created on demand at runtime
     */
    boolean_T fCanMalloc;
};

/* ------------------------------------------------------------------------
 * Memory Pool Manager
 * 
 * Manage one or more fixed size memory pools. All pools are held as a 
 * linked list and this manager keeps track of the first one.
 * ------------------------------------------------------------------------
 */
struct __slmsg_MemPoolMgr_T
{       
    /* Assert that all allocated memory was freed */
    uint8_T fAssertLeakFree; 

    int_T fNumPools; 
    __slmsg_MemPool* fPools;
};

/* Type: slMessage --------------------------------------------------------
 * Abstract:
 *     Data structure for a Simulink message
 */
struct _slMessage
{
    void *fData;
    slMessage* fNext;
    slMessage* fPrev;

#ifndef SLMSG_PRODUCTION_CODE
    slMsgId fId;
    uint_T fPriority;
    slMsgDataSize fDataSize;
    slMsgQueueId fQueueId;
    void *fAppData;
    void (*fAppDeleter)(slMessage *);
#endif
};

typedef struct __slMessage_wrapper{
    slMessage msg;
    slMsgLink link;
} slMsgWrapper;

typedef struct _slMsgManager slMsgManager;

/* Type: slMsgQueue -------------------------------------------------------
 * Abstract:
 *     Data structure for a message queue
 *     
 *     A queue holds static properties that define queuing behavior as
 *     well as messages that are contained in the queue at runtime as a
 *     linked list.
 */
typedef struct _slMsgQueue 
{
    slMsgQueueId fId;
    int_T fCapacity;
    int_T fLength;
    slMsgQueueType fType;
    slMsgQueueDropPolicy fDropPolicy;
    slMsgDataSize fDataSize;
    slMsgMemPoolId readerMessageMemPoolId;
    slMsgMemPoolId writerMessageMemPoolId;
    slMsgMemPoolId readerPayloadMemPoolId;
    slMsgMemPoolId writerPayloadMemPoolId;
    int_T         fPriorityDataType;
    slMsgDataSize fPriorityDataOffset; /* offset in bytes to priority field */
    slMessage *fHead;
    slMessage *fTail;
    volatile slMsgId _nextMsgId;

    slMsgManager* fMgr;
    uint_T        fIsOverflow;

#ifndef SLMSG_PRODUCTION_CODE
    ulong_T _fNumDropped;
    ulong_T _fComputedNecessaryCapacity;
    ulong_T _fCurrentNecessaryCapacity;

    /* message logging */
    const void *_fInstrumentDropObj;
    const void *_fInstrumentSendObj;
    const void *_fInstrumentPopObj;
#endif

#ifdef SLMSG_INCLUDE_TASK_TRANSITION_QUEUE
    /*For SRSW lock-free queue only*/
    struct {
        uint8_T *fCircularArray;
        volatile uint8_T fCircularHead;
        volatile uint8_T fCircularTail;
        slMsgDataSize fCircularChunkSize;
        uint8_T fCircularCapacity;
    } fSRSWFIFOQueue;
#endif
} slMsgQueue;

typedef slMsgQueue* slMsgQueuePtr ;

/* Type: slMsgManager -----------------------------------------------------
 * Abstract:
 *     Manager of all messages and queues during runtime
 *     
 *     The message manager holds all of the messages alive in a model at
 *     any given time, as well as all of the queue data structures.
 */
struct _slMsgManager 
{
    slMsgQueue *fQueues;
    slMsgDataSize fNumQueues;

    __slmsg_MemPoolMgr fPoolMgr;
    slMsgDataSize fNumPools;

    boolean_T fMdlRefSupport;

    boolean_T fUseGlobalMsgIds;
    slMsgId fNextMsgId;
    uint_T fNextMsgPriority;
};


#if defined(SLMSG_USE_EXCEPTION) || defined(SLMSG_ALLOW_SYSTEM_ALLOC)

/* ------------------------------------------------------------------------
 *                         Runtime Exception 
 * --------------------------------------------------------------------- */

typedef enum {
    __slmsg_Except_Assertion_Failed = 0,
    __slmsg_Except_Bad_Alloc
} __slmsg_Except_Type;

SL_MESSAGES_EXPORT_FCN_C void __slmsg_Except_raise(__slmsg_Except_Type eType, const char_T *file, int_T line);
#define __slmsg_RAISE(e) __slmsg_Except_raise((e), __FILE__, __LINE__)

#undef __slmsg_assert
#ifdef NDEBUG
#define __slmsg_assert(e) ((void)0)
#else
#define __slmsg_assert(e) ((void)((e)||(__slmsg_RAISE(__slmsg_Except_Assertion_Failed),0)))
#endif

#endif /* SLMSG_USE_EXCEPTION || SLMSG_ALLOW_SYSTEM_ALLOC */

#ifdef SLMSG_ALLOW_SYSTEM_ALLOC
/* ------------------------------------------------------------------------
 *                           Memory Management
 * --------------------------------------------------------------------- */

SL_MESSAGES_EXPORT_FCN_C void *__slmsg_Mem_alloc  (slMsgPoolSize nbytes, const char_T *file, int_T line);
SL_MESSAGES_EXPORT_FCN_C void *__slmsg_Mem_calloc (slMsgPoolSize count, slMsgPoolSize nbytes, const char_T *file, int_T line);
SL_MESSAGES_EXPORT_FCN_C void __slmsg_Mem_free    (void *ptr, const char_T *file, int_T line);
SL_MESSAGES_EXPORT_FCN_C void *__slmsg_Mem_resize (void *ptr, slMsgPoolSize nbytes, const char_T *file, int_T line);

#define __slmsg_SYSTEM_ALLOC(nbytes)         __slmsg_Mem_alloc((nbytes), __FILE__, __LINE__)
#define __slmsg_SYSTEM_CALLOC(count, nbytes) __slmsg_Mem_calloc((count), (nbytes), __FILE__, __LINE__)
#define __slmsg_SYSTEM_FREE(ptr)             ((void)(__slmsg_Mem_free((ptr), __FILE__, __LINE__), (ptr) = 0))

#endif /* SLMSG_ALLOW_SYSTEM_ALLOC */

#ifndef SL_INTERNAL
/* ------------------------------------------------------------------------
 *                           Public APIs
 * --------------------------------------------------------------------- */

/* Create and initialize the message runtime services */
void slMsgSvcInitMsgManager(void* msgMgr);

/* Finalize message runtime services, delete all messages and queues */
void slMsgSvcFinalizeMsgManager(void *msgMgr);

/* Terminate message runtime services, delete all messages and queues */
void slMsgSvcTerminateMsgManager(void *msgMgr);

/* Initialize the memory pool */
void slMsgSvcInitMemPool(void *msgMgr,
                         slMsgMemPoolId poolId,
                         int_T numUnits,
                         slMsgDataSize dataSize,
                         void *memBuffer,
                         boolean_T mallocAllowed);

/* Set the number of message queues that will be used */
void slMsgSvcSetNumMsgQueues(void *msgMgr, int_T num, void *queueArray);
/* Set the number of message pools that will be used */
void slMsgSvcSetNumMemPools(void *msgMgr, int_T num, void *poolArray);

/* Create a FIFO message queue with specified properties 
 * Use capacity=SLMSG_UNSPECIFIED to request infinite capacity
 */
void slMsgSvcCreateFIFOMsgQueue(void *msgMgr,
                                int_T id,
                                int_T capacity,
                                slMsgDataSize dataSize,
                                int_T dropPolicy,
                                slMsgMemPoolId messageMemPoolId,
                                slMsgMemPoolId payloadMemPoolId);

/* Create a SRSW lock-free FIFO message queue with specified properties 
 * Capacity can only be in [1, MAX_uint8_T]; out of range capacity will be
 * set to MAX_uint8_T
 */
void slMsgSvcCreateSRSWFIFOMsgQueue(void *msgMgr, 
                                    int_T id, 
                                    int_T capacity, 
                                    slMsgDataSize dataSize, 
                                    void* sharedArray,
                                    slMsgMemPoolId readerMessageMemPoolId,
                                    slMsgMemPoolId writerMessageMemPoolId,
                                    slMsgMemPoolId readerPayloadMemPoolId,
                                    slMsgMemPoolId writerPayloadMemPoolId);

/* Create a LIFO message queue with specified properties
 * Use capacity=SLMSG_UNSPECIFIED to request infinite capacity
 */
void slMsgSvcCreateLIFOMsgQueue(void *msgMgr, 
                                int_T id,
                                int_T capacity,
                                slMsgDataSize dataSize,
                                int_T dropPolicy,
                                slMsgMemPoolId messageMemPoolId,
                                slMsgMemPoolId payloadMemPoolId);

/* Create a Priority message queue with specified properties
 * Use capacity=SLMSG_UNSPECIFIED to request infinite capacity
 */
void slMsgSvcCreatePriorityMsgQueue(void *msgMgr, 
                                    int_T id,
                                    int_T capacity,
                                    int_T priorityMode,
                                    slMsgDataSize dataSize,
                                    int_T priorityDataType,
                                    slMsgDataSize priorityDataOffset,
                                    int_T dropPolicy,
                                    slMsgMemPoolId messageMemPoolId,
                                    slMsgMemPoolId payloadMemPoolId);

/* Create a new message and send it to the specified queue */
uint_T slMsgSvcSendMsgToQueue(void *msgMgr, void *msgptr, slMsgQueueId queueId);

 
/* isOverflowed is set to the overflow status before the pop-operation; then
 * pop one message at the top of the specified queue;
 */
void* slMsgSvcPopMsgFromQueue(void *msgMgr, slMsgQueueId queueId, uint_T* isOverflowed);

/* Return the overflow status of a given queue */
uint_T slMsgSvcQueueIsOverflow(void *msgMgr, slMsgQueueId queueId);

/* Return the data held by the specified message */
void* slMsgSvcGetMsgData(void *msgptr);

/* Return number of messages present in specified queue */
int slMsgSvcGetNumMsgsInQueue(void *msgMgr, slMsgQueueId queueId);

/* Destroy the specified message */
void slMsgSvcDestroyMsg(void *msgMgr, void *msgptr);

/* Create a new message */
void* slMsgSvcCreateMsg(void *msgMgr, const void* data, slMsgDataSize dataSize, slMsgQueueId queueId);


#endif /* SL_INTERNAL */


#ifdef __cplusplus
}
#endif

#endif /* _slMsgSvc_h_ */

